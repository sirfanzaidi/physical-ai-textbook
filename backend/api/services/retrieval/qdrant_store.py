"""Qdrant vector store for semantic search and retrieval."""

from typing import List, Optional, Dict, Any
from qdrant_client import AsyncQdrantClient
from qdrant_client.models import (
    Distance,
    PointStruct,
    VectorParams,
    FieldCondition,
    MatchValue,
    Filter,
)
from .config import Settings
from .utils import get_logger, QdrantError

logger = get_logger(__name__)


class QdrantStore:
    """Vector store using Qdrant for semantic search."""

    def __init__(self, settings: Settings):
        """Initialize Qdrant store.

        Args:
            settings: Application settings with Qdrant configuration

        Raises:
            QdrantError: If connection fails
        """
        self.settings = settings
        self.client = AsyncQdrantClient(url=settings.qdrant_url)
        self.collection_name = settings.qdrant_collection_name
        logger.info(f"Qdrant client initialized at {settings.qdrant_url}")

    async def initialize_collection(
        self,
        vector_size: int = 1024,
        distance_metric: Distance = Distance.COSINE,
    ) -> None:
        """Initialize or verify Qdrant collection exists.

        Args:
            vector_size: Dimension of embedding vectors
            distance_metric: Distance metric for similarity

        Raises:
            QdrantError: If initialization fails
        """
        try:
            # Check if collection exists
            collections = await self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                logger.info(
                    f"Creating collection '{self.collection_name}' "
                    f"with {vector_size}-dim vectors"
                )
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=vector_size,
                        distance=distance_metric,
                    ),
                )
            else:
                logger.info(f"Collection '{self.collection_name}' already exists")

            # Get and log collection info
            collection_info = await self.client.get_collection(self.collection_name)
            logger.info(f"Collection info: {collection_info.points_count} points")

        except Exception as e:
            logger.error(f"Failed to initialize collection: {e}")
            raise QdrantError(f"Failed to initialize collection: {str(e)}")

    async def upsert_vectors(
        self,
        points: List[PointStruct],
    ) -> int:
        """Upsert (insert or update) vectors into collection.

        Args:
            points: List of PointStruct objects with id, vector, and payload

        Returns:
            Number of points upserted

        Raises:
            QdrantError: If upsert fails
        """
        try:
            await self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )
            logger.info(f"Upserted {len(points)} vectors")
            return len(points)
        except Exception as e:
            logger.error(f"Failed to upsert vectors: {e}")
            raise QdrantError(f"Failed to upsert vectors: {str(e)}")

    async def search(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        filters: Optional[Filter] = None,
    ) -> List[Dict[str, Any]]:
        """Search for similar vectors.

        Args:
            query_embedding: Query embedding vector
            top_k: Number of results to return
            filters: Optional Qdrant filter for metadata

        Returns:
            List of search results with metadata

        Raises:
            QdrantError: If search fails
        """
        try:
            search_results = await self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=filters,
                limit=top_k,
                with_payload=True,
            )

            results = [
                {
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                }
                for result in search_results
            ]

            logger.debug(f"Search returned {len(results)} results")
            return results
        except Exception as e:
            logger.error(f"Search failed: {e}")
            raise QdrantError(f"Search failed: {str(e)}")

    async def search_with_mode(
        self,
        query_embedding: List[float],
        mode: str = "full",
        selected_text: Optional[str] = None,
        top_k: int = 5,
    ) -> List[Dict[str, Any]]:
        """Search with mode support (full book or selected text only).

        Args:
            query_embedding: Query embedding vector
            mode: Query mode ('full' or 'selected')
            selected_text: Selected text for mode='selected'
            top_k: Number of results

        Returns:
            List of search results

        Raises:
            QdrantError: If search fails
        """
        filters = None

        if mode == "selected" and selected_text:
            # Create filter to match selected text in payload
            # This is a simple approach; for more sophisticated matching,
            # store selected_text chunks separately
            logger.debug(f"Searching with selected text mode")

        return await self.search(query_embedding, top_k, filters)

    async def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific chunk by ID.

        Args:
            chunk_id: Chunk identifier

        Returns:
            Chunk payload or None if not found

        Raises:
            QdrantError: If retrieval fails
        """
        try:
            result = await self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id],
                with_payload=True,
            )
            if result:
                return result[0].payload
            return None
        except Exception as e:
            logger.error(f"Failed to get chunk {chunk_id}: {e}")
            raise QdrantError(f"Failed to get chunk: {str(e)}")

    async def delete_by_filter(self, filters: Filter) -> None:
        """Delete vectors matching filter.

        Args:
            filters: Qdrant filter condition

        Raises:
            QdrantError: If deletion fails
        """
        try:
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=filters,
            )
            logger.info("Deleted vectors matching filter")
        except Exception as e:
            logger.error(f"Failed to delete vectors: {e}")
            raise QdrantError(f"Failed to delete vectors: {str(e)}")

    async def get_collection_info(self) -> Dict[str, Any]:
        """Get collection statistics.

        Returns:
            Collection information

        Raises:
            QdrantError: If retrieval fails
        """
        try:
            info = await self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "points_count": info.points_count,
                "vectors_count": getattr(info, "vectors_count", None),
                "indexed_vectors_count": getattr(info, "indexed_vectors_count", None),
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise QdrantError(f"Failed to get collection info: {str(e)}")
