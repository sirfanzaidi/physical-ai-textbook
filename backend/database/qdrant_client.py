"""
Qdrant vector database client wrapper.

Provides high-level interface for vector storage and semantic search operations.
"""

from typing import List, Optional, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
import structlog

from backend.utils.errors import QdrantError

logger = structlog.get_logger(__name__)


class QdrantVectorStore:
    """Wrapper for Qdrant vector database operations."""

    def __init__(self, url: str, api_key: str, collection_name: str):
        """
        Initialize Qdrant client.

        Args:
            url: Qdrant instance URL
            api_key: API key for authentication
            collection_name: Name of the collection to use

        Raises:
            QdrantError: If connection fails
        """
        try:
            self.client = QdrantClient(
                url=url,
                api_key=api_key,
            )
            self.collection_name = collection_name
            logger.info(
                "qdrant_client_initialized",
                url=url,
                collection=collection_name,
            )
        except Exception as e:
            raise QdrantError(
                f"Failed to initialize Qdrant client: {str(e)}",
                error_code="QDRANT_INIT_FAILED",
                details={"url": url, "error": str(e)},
            )

    def upsert_vectors(
        self,
        chunk_ids: List[str],
        vectors: List[List[float]],
        metadata: List[Dict[str, Any]],
    ) -> int:
        """
        Upsert vectors with metadata into Qdrant.

        Args:
            chunk_ids: List of chunk identifiers
            vectors: List of embedding vectors (1024-dimensional for Cohere embed-v4.0)
            metadata: List of metadata dicts (must match chunk_ids and vectors length)

        Returns:
            Number of vectors upserted

        Raises:
            QdrantError: If operation fails
        """
        if not (len(chunk_ids) == len(vectors) == len(metadata)):
            raise QdrantError(
                "Mismatched lengths: chunk_ids, vectors, and metadata must have same length",
                error_code="INVALID_UPSERT_DATA",
                details={
                    "chunk_ids_len": len(chunk_ids),
                    "vectors_len": len(vectors),
                    "metadata_len": len(metadata),
                },
            )

        try:
            points = [
                PointStruct(
                    id=hash(chunk_id) % (2**31),  # Convert string ID to positive int
                    vector=vector,
                    payload=meta,
                )
                for chunk_id, vector, meta in zip(chunk_ids, vectors, metadata)
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )

            logger.info(
                "vectors_upserted",
                collection=self.collection_name,
                count=len(points),
            )
            return len(points)

        except Exception as e:
            raise QdrantError(
                f"Failed to upsert vectors: {str(e)}",
                error_code="QDRANT_UPSERT_FAILED",
                details={"collection": self.collection_name, "error": str(e)},
            )

    def search(
        self,
        query_vector: List[float],
        top_k: int = 20,
        score_threshold: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant.

        Args:
            query_vector: Query embedding vector (1536-dimensional for embed-v4.0)
            top_k: Number of top results to return (default: 20 for reranking)
            score_threshold: Minimum similarity score threshold

        Returns:
            List of search results with scores and metadata

        Raises:
            QdrantError: If search fails
        """
        try:
            # Use query_points for qdrant-client >= 1.16
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                score_threshold=score_threshold,
            ).points

            results = [
                {
                    "chunk_id": result.payload.get("chunk_id"),
                    "score": result.score,
                    "metadata": result.payload,
                }
                for result in search_results
            ]

            logger.info(
                "vector_search_completed",
                collection=self.collection_name,
                top_k=top_k,
                results_found=len(results),
            )
            return results

        except Exception as e:
            raise QdrantError(
                f"Failed to search vectors: {str(e)}",
                error_code="QDRANT_SEARCH_FAILED",
                details={"collection": self.collection_name, "error": str(e)},
            )

    def delete_by_book(self, book_id: str) -> int:
        """
        Delete all vectors for a specific book.

        Args:
            book_id: Book identifier

        Returns:
            Number of vectors deleted

        Raises:
            QdrantError: If operation fails
        """
        try:
            delete_result = self.client.delete(
                collection_name=self.collection_name,
                points_selector={
                    "filter": {
                        "must": [
                            {
                                "key": "book_id",
                                "match": {"value": book_id},
                            }
                        ]
                    }
                },
            )

            logger.info(
                "book_vectors_deleted",
                collection=self.collection_name,
                book_id=book_id,
                count=delete_result.deleted,
            )
            return delete_result.deleted

        except Exception as e:
            raise QdrantError(
                f"Failed to delete vectors by book: {str(e)}",
                error_code="QDRANT_DELETE_FAILED",
                details={"collection": self.collection_name, "book_id": book_id, "error": str(e)},
            )

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get collection information and statistics.

        Returns:
            Collection info dict

        Raises:
            QdrantError: If operation fails
        """
        try:
            collection = self.client.get_collection(self.collection_name)
            return {
                "name": collection.config.params.vectors.get_type(),
                "vector_size": collection.config.params.vectors.get_type(),
                "point_count": collection.points_count,
            }
        except Exception as e:
            raise QdrantError(
                f"Failed to get collection info: {str(e)}",
                error_code="QDRANT_INFO_FAILED",
                details={"collection": self.collection_name, "error": str(e)},
            )

    def health_check(self) -> bool:
        """
        Check if Qdrant is accessible and collection exists.

        Returns:
            True if healthy, False otherwise
        """
        try:
            self.client.get_collection(self.collection_name)
            logger.info("qdrant_health_check_passed", collection=self.collection_name)
            return True
        except Exception as e:
            logger.warning(
                "qdrant_health_check_failed",
                collection=self.collection_name,
                error=str(e),
            )
            return False


# Singleton instance
_qdrant_store: Optional[QdrantVectorStore] = None


def get_qdrant_store(
    url: str = None, api_key: str = None, collection_name: str = None
) -> QdrantVectorStore:
    """
    Get or create Qdrant vector store instance.

    Args:
        url: Qdrant URL (only needed for initialization)
        api_key: Qdrant API key (only needed for initialization)
        collection_name: Collection name (only needed for initialization)

    Returns:
        QdrantVectorStore instance
    """
    global _qdrant_store
    if _qdrant_store is None:
        if not all([url, api_key, collection_name]):
            raise QdrantError(
                "Missing required parameters for Qdrant initialization",
                error_code="QDRANT_INIT_MISSING_PARAMS",
                details={"url": url is not None, "api_key": api_key is not None},
            )
        _qdrant_store = QdrantVectorStore(url, api_key, collection_name)
    return _qdrant_store
