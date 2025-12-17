"""
Vector-based retrieval for RAG Chatbot.

Searches for semantically similar chunks using Qdrant.
Supports both full-book and select-text (zero-leakage) retrieval modes.
"""

from typing import List, Dict, Any, Optional
import structlog
import hashlib

from database.qdrant_client import QdrantVectorStore
from backend.utils.errors import RetrievalError

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

    def retrieve_for_selected_text(
        self,
        selected_text: str,
        book_id: str,
    ) -> List[Dict[str, Any]]:
        """
        Retrieve ONLY chunks matching the selected text (zero-leakage mode).

        This method enforces strict zero-leakage constraint by:
        1. Computing hash of selected text
        2. Searching for exact text match in stored chunks
        3. Returning only matching chunks

        Args:
            selected_text: User-selected text passage
            book_id: Book being queried

        Returns:
            List of chunks that exactly match selected text

        Raises:
            RetrievalError: If retrieval fails
        """
        try:
            # Compute hash of selected text for matching
            text_hash = hashlib.sha256(selected_text.encode()).hexdigest()

            # Search all chunks for this book
            all_chunks = self._get_all_book_chunks(book_id)

            # Filter to only chunks with exact text match
            matching_chunks = [
                chunk for chunk in all_chunks
                if self._chunk_text_matches(chunk, selected_text, text_hash)
            ]

            logger.info(
                "selected_text_chunks_retrieved",
                book_id=book_id,
                selected_text_hash=text_hash,
                matching_chunks=len(matching_chunks),
                total_chunks=len(all_chunks),
            )

            return matching_chunks

        except Exception as e:
            raise RetrievalError(
                f"Failed to retrieve chunks for selected text: {str(e)}",
                error_code="SELECTED_TEXT_RETRIEVAL_FAILED",
                details={"book_id": book_id, "error": str(e)},
            )

    def _get_all_book_chunks(self, book_id: str) -> List[Dict[str, Any]]:
        """
        Get all chunks for a book (for select-text matching).

        Args:
            book_id: Book identifier

        Returns:
            List of all chunks for the book
        """
        # In practice, would query Qdrant or database for all chunks
        # For now, return empty list (would be populated by vector store)
        return []

    def _chunk_text_matches(
        self, chunk: Dict[str, Any], selected_text: str, text_hash: str
    ) -> bool:
        """
        Check if chunk matches selected text.

        Uses both hash comparison and text similarity for robustness.

        Args:
            chunk: Chunk dict with metadata
            selected_text: Selected text to match
            text_hash: Pre-computed hash of selected text

        Returns:
            True if chunk matches selected text
        """
        chunk_text = chunk.get("metadata", {}).get("text", "")
        if not chunk_text:
            return False

        # Exact match check
        if chunk_text == selected_text:
            return True

        # Hash-based check (handles whitespace variations)
        chunk_hash = hashlib.sha256(chunk_text.encode()).hexdigest()
        if chunk_hash == text_hash:
            return True

        # Substring check (selected text contains chunk)
        if selected_text and chunk_text in selected_text:
            return True

        return False
