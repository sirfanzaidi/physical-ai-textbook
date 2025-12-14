"""
Vector-based retrieval for RAG Chatbot.

Searches for semantically similar chunks using Qdrant.
"""

from typing import List, Dict, Any, Optional
import structlog

from database.qdrant_client import QdrantVectorStore
from utils.errors import RetrievalError

logger = structlog.get_logger(__name__)


class SemanticRetriever:
    """Retrieves semantically similar chunks from vector store."""

    def __init__(self, vector_store: QdrantVectorStore):
        """
        Initialize semantic retriever.

        Args:
            vector_store: Initialized QdrantVectorStore instance
        """
        self.vector_store = vector_store
        logger.info("semantic_retriever_initialized")

    def retrieve(
        self,
        query_embedding: List[float],
        book_id: str,
        top_k: int = 20,
        score_threshold: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """
        Retrieve semantically similar chunks for a query.

        Args:
            query_embedding: Query embedding vector (1024-dimensional)
            book_id: Specific book to search in
            top_k: Number of top results to return (default: 20 for reranking)
            score_threshold: Minimum similarity score threshold

        Returns:
            List of retrieved chunks with scores and metadata

        Raises:
            RetrievalError: If retrieval fails
        """
        try:
            # Search in vector store
            search_results = self.vector_store.search(
                query_vector=query_embedding,
                top_k=top_k,
                score_threshold=score_threshold,
            )

            # Filter results to specific book
            filtered_results = [
                r for r in search_results
                if r.get("metadata", {}).get("book_id") == book_id
            ]

            logger.info(
                "chunks_retrieved",
                book_id=book_id,
                top_k=top_k,
                retrieved_count=len(filtered_results),
                total_results=len(search_results),
            )
            return filtered_results

        except Exception as e:
            raise RetrievalError(
                f"Failed to retrieve chunks: {str(e)}",
                error_code="RETRIEVAL_FAILED",
                details={"book_id": book_id, "top_k": top_k, "error": str(e)},
            )

    def retrieve_all_books(
        self,
        query_embedding: List[float],
        top_k: int = 20,
        score_threshold: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """
        Retrieve semantically similar chunks across all books.

        Args:
            query_embedding: Query embedding vector
            top_k: Number of top results
            score_threshold: Minimum similarity threshold

        Returns:
            List of retrieved chunks with scores and metadata

        Raises:
            RetrievalError: If retrieval fails
        """
        try:
            search_results = self.vector_store.search(
                query_vector=query_embedding,
                top_k=top_k,
                score_threshold=score_threshold,
            )

            logger.info(
                "chunks_retrieved_all_books",
                top_k=top_k,
                retrieved_count=len(search_results),
            )
            return search_results

        except Exception as e:
            raise RetrievalError(
                f"Failed to retrieve chunks from all books: {str(e)}",
                error_code="RETRIEVAL_FAILED",
                details={"top_k": top_k, "error": str(e)},
            )

    def retrieve_by_book(
        self,
        query_embedding: List[float],
        book_id: str,
        threshold: float = 0.5,
    ) -> List[Dict[str, Any]]:
        """
        Retrieve chunks above a similarity threshold for a specific book.

        Args:
            query_embedding: Query embedding vector
            book_id: Book to search in
            threshold: Minimum similarity score

        Returns:
            List of chunks above threshold

        Raises:
            RetrievalError: If retrieval fails
        """
        try:
            search_results = self.vector_store.search(
                query_vector=query_embedding,
                top_k=100,  # Get many results for filtering
                score_threshold=threshold,
            )

            # Filter by book and threshold
            filtered_results = [
                r for r in search_results
                if (r.get("metadata", {}).get("book_id") == book_id and
                    r.get("score", 0) >= threshold)
            ]

            logger.info(
                "chunks_retrieved_by_book",
                book_id=book_id,
                threshold=threshold,
                count=len(filtered_results),
            )
            return filtered_results

        except Exception as e:
            raise RetrievalError(
                f"Failed to retrieve chunks by book: {str(e)}",
                error_code="RETRIEVAL_FAILED",
                details={"book_id": book_id, "threshold": threshold, "error": str(e)},
            )
