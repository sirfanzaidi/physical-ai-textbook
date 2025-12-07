"""ChromaDB vector database service for RAG system."""

import os
import logging
from typing import List, Optional, Dict, Any
import json
import time
from pathlib import Path

logger = logging.getLogger(__name__)


class VectorDBService:
    """Service for managing ChromaDB collections and vector operations."""

    def __init__(
        self,
        db_path: str = "./data/embeddings",
        collection_name: str = "physical_ai_textbook",
        embedding_dimension: int = 384
    ):
        """Initialize ChromaDB service.

        Args:
            db_path: Path to persist ChromaDB data
            collection_name: Name of the default collection
            embedding_dimension: Expected dimension of embeddings (384 for all-MiniLM)
        """
        self.db_path = db_path or os.getenv("CHROMADB_PATH", "./data/embeddings")
        self.collection_name = collection_name
        self.embedding_dimension = embedding_dimension

        # Create db directory if it doesn't exist
        Path(self.db_path).mkdir(parents=True, exist_ok=True)

        self._client = None
        self._collection = None
        self._is_connected = False
        logger.info(f"VectorDBService initialized (path={self.db_path}, collection={collection_name})")

    def _ensure_connected(self) -> None:
        """Lazy load ChromaDB client on first use."""
        if self._is_connected:
            return

        try:
            import chromadb
            logger.info(f"Connecting to ChromaDB at {self.db_path}")
            self._client = chromadb.PersistentClient(path=self.db_path)
            self._is_connected = True
            logger.info("ChromaDB client connected")
        except ImportError:
            raise ImportError("chromadb not installed. Install with: pip install chromadb")
        except Exception as e:
            logger.error(f"Failed to connect to ChromaDB: {e}")
            raise

    def get_or_create_collection(self, collection_name: Optional[str] = None):
        """Get or create a collection.

        Args:
            collection_name: Name of collection (default: self.collection_name)

        Returns:
            ChromaDB collection object
        """
        self._ensure_connected()
        name = collection_name or self.collection_name

        try:
            collection = self._client.get_or_create_collection(
                name=name,
                metadata={"embedding_dimension": self.embedding_dimension}
            )
            logger.info(f"Using collection: {name}")
            return collection
        except Exception as e:
            logger.error(f"Failed to create/get collection {name}: {e}")
            raise

    def add_chunks(
        self,
        chunks: List[Dict[str, Any]],
        collection_name: Optional[str] = None
    ) -> int:
        """Add chunks to collection.

        Args:
            chunks: List of dicts with 'id', 'embedding', 'metadata', 'document' keys
            collection_name: Target collection (default: self.collection_name)

        Returns:
            Number of chunks added

        Raises:
            ValueError: If chunk format is invalid
            RuntimeError: If operation fails
        """
        if not chunks:
            return 0

        self._ensure_connected()
        collection = self.get_or_create_collection(collection_name)

        try:
            # Prepare data for ChromaDB
            ids = []
            embeddings = []
            documents = []
            metadatas = []

            for chunk in chunks:
                ids.append(chunk["id"])
                embeddings.append(chunk["embedding"])
                documents.append(chunk["document"])
                metadatas.append(chunk.get("metadata", {}))

            # Add to collection
            start_time = time.time()
            collection.add(
                ids=ids,
                embeddings=embeddings,
                documents=documents,
                metadatas=metadatas
            )
            elapsed = time.time() - start_time
            logger.info(f"Added {len(chunks)} chunks in {elapsed:.2f}s")
            return len(chunks)
        except Exception as e:
            logger.error(f"Failed to add chunks: {e}")
            raise RuntimeError(f"Failed to add chunks: {e}")

    def search(
        self,
        query_embedding: List[float],
        n_results: int = 5,
        collection_name: Optional[str] = None,
        where: Optional[Dict] = None
    ) -> Dict[str, Any]:
        """Search for similar chunks using embedding.

        Args:
            query_embedding: Query vector (384-dims for all-MiniLM)
            n_results: Number of results to return
            collection_name: Collection to search
            where: Optional filter conditions

        Returns:
            Dict with 'ids', 'documents', 'metadatas', 'distances'
        """
        self._ensure_connected()
        collection = self.get_or_create_collection(collection_name)

        try:
            start_time = time.time()
            results = collection.query(
                query_embeddings=[query_embedding],
                n_results=n_results,
                where=where
            )
            elapsed = time.time() - start_time

            # Convert distances to relevance scores (cosine similarity: 1 - distance)
            if results["distances"] and results["distances"][0]:
                relevance_scores = [1 - d for d in results["distances"][0]]
            else:
                relevance_scores = []

            logger.debug(f"Search completed in {elapsed:.2f}s, found {len(results['ids'][0]) if results['ids'] else 0} results")

            return {
                "ids": results["ids"][0] if results["ids"] else [],
                "documents": results["documents"][0] if results["documents"] else [],
                "metadatas": results["metadatas"][0] if results["metadatas"] else [],
                "relevance_scores": relevance_scores,
                "search_time_ms": int(elapsed * 1000)
            }
        except Exception as e:
            logger.error(f"Search failed: {e}")
            raise RuntimeError(f"Search failed: {e}")

    def delete_collection(self, collection_name: Optional[str] = None) -> bool:
        """Delete a collection (for blue-green swap cleanup).

        Args:
            collection_name: Name of collection to delete

        Returns:
            True if deleted, False if not found
        """
        self._ensure_connected()
        name = collection_name or self.collection_name

        try:
            self._client.delete_collection(name=name)
            logger.info(f"Deleted collection: {name}")
            return True
        except Exception as e:
            logger.warning(f"Collection not found or error deleting {name}: {e}")
            return False

    def list_collections(self) -> List[str]:
        """List all collections in the database.

        Returns:
            List of collection names
        """
        self._ensure_connected()
        try:
            collections = self._client.list_collections()
            names = [c.name for c in collections]
            logger.debug(f"Collections: {names}")
            return names
        except Exception as e:
            logger.error(f"Failed to list collections: {e}")
            return []

    def get_collection_count(self, collection_name: Optional[str] = None) -> int:
        """Get number of chunks in collection.

        Args:
            collection_name: Collection to check

        Returns:
            Number of chunks
        """
        self._ensure_connected()
        collection = self.get_or_create_collection(collection_name)

        try:
            count = collection.count()
            logger.debug(f"Collection {collection.name} contains {count} chunks")
            return count
        except Exception as e:
            logger.error(f"Failed to count chunks: {e}")
            return 0

    def get_db_stats(self) -> Dict[str, Any]:
        """Get statistics about the database.

        Returns:
            Dict with collection info and sizes
        """
        self._ensure_connected()

        stats = {
            "path": self.db_path,
            "collections": {}
        }

        for collection_name in self.list_collections():
            try:
                # Get approximate size from directory
                db_dir = Path(self.db_path)
                if db_dir.exists():
                    total_size = sum(f.stat().st_size for f in db_dir.rglob("*") if f.is_file())
                else:
                    total_size = 0

                stats["collections"][collection_name] = {
                    "chunk_count": self.get_collection_count(collection_name),
                    "db_size_bytes": total_size
                }
            except Exception as e:
                logger.warning(f"Failed to get stats for {collection_name}: {e}")

        return stats
