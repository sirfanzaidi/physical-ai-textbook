"""
Unit tests for semantic retriever.

Tests vector search and filtering logic.
"""

import pytest
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from retrieval.retriever import SemanticRetriever
from database.qdrant_client import QdrantVectorStore
from backend.utils.errors import RetrievalError


class TestSemanticRetriever:
    """Test SemanticRetriever functionality."""

    @pytest.fixture
    def mock_vector_store(self):
        """Create mock vector store."""
        class MockVectorStore:
            def search(self, query_vector, top_k, score_threshold=None):
                return [
                    {
                        "chunk_id": "chunk_1",
                        "score": 0.9,
                        "metadata": {
                            "book_id": "book_1",
                            "text": "Sample text 1",
                            "page_num": 1,
                        },
                    },
                    {
                        "chunk_id": "chunk_2",
                        "score": 0.8,
                        "metadata": {
                            "book_id": "book_1",
                            "text": "Sample text 2",
                            "page_num": 2,
                        },
                    },
                    {
                        "chunk_id": "chunk_3",
                        "score": 0.7,
                        "metadata": {
                            "book_id": "book_2",
                            "text": "Different book",
                            "page_num": 1,
                        },
                    },
                ]

        return MockVectorStore()

    @pytest.fixture
    def retriever(self, mock_vector_store):
        """Create retriever with mock vector store."""
        return SemanticRetriever(mock_vector_store)

    def test_retriever_initialization(self, retriever):
        """Test retriever initializes correctly."""
        assert retriever.vector_store is not None

    def test_retrieve_filters_by_book_id(self, retriever):
        """Test retrieve filters results by book_id."""
        query_embedding = [0.1] * 1024
        results = retriever.retrieve(
            query_embedding=query_embedding,
            book_id="book_1",
            top_k=10,
        )

        # Should only return results for book_1
        assert len(results) == 2
        assert all(r.get("metadata", {}).get("book_id") == "book_1" for r in results)

    def test_retrieve_empty_results(self, retriever):
        """Test retrieve with book_id that has no results."""
        query_embedding = [0.1] * 1024
        results = retriever.retrieve(
            query_embedding=query_embedding,
            book_id="nonexistent",
            top_k=10,
        )

        assert results == []

    def test_retrieve_with_top_k(self, retriever):
        """Test retrieve respects top_k parameter."""
        query_embedding = [0.1] * 1024
        results = retriever.retrieve(
            query_embedding=query_embedding,
            book_id="book_1",
            top_k=5,
        )

        assert len(results) <= 5

    def test_retrieve_returns_metadata(self, retriever):
        """Test retrieve includes metadata in results."""
        query_embedding = [0.1] * 1024
        results = retriever.retrieve(
            query_embedding=query_embedding,
            book_id="book_1",
            top_k=10,
        )

        for result in results:
            assert "chunk_id" in result
            assert "score" in result
            assert "metadata" in result

    def test_retrieve_all_books(self, retriever):
        """Test retrieve_all_books returns all results."""
        query_embedding = [0.1] * 1024
        results = retriever.retrieve_all_books(
            query_embedding=query_embedding,
            top_k=10,
        )

        # Should return all 3 chunks
        assert len(results) == 3

    def test_retrieve_by_book_with_threshold(self, retriever):
        """Test retrieve_by_book with similarity threshold."""
        query_embedding = [0.1] * 1024
        results = retriever.retrieve_by_book(
            query_embedding=query_embedding,
            book_id="book_1",
            threshold=0.8,
        )

        # Only chunks with score >= 0.8
        assert len(results) == 2
        assert all(r.get("score", 0) >= 0.8 for r in results)


class TestRetrieverDataStructures:
    """Test data structures used by retriever."""

    def test_search_result_structure(self):
        """Test search result has expected structure."""
        result = {
            "chunk_id": "test_chunk",
            "score": 0.95,
            "metadata": {
                "book_id": "book_1",
                "text": "Sample",
                "page_num": 5,
            },
        }

        assert result["chunk_id"] == "test_chunk"
        assert result["score"] == 0.95
        assert result["metadata"]["book_id"] == "book_1"

    def test_query_embedding_dimensions(self):
        """Test expected query embedding dimensions."""
        # Cohere embed-v4.0 produces 1024-dimensional vectors
        query_embedding = [0.1] * 1024
        assert len(query_embedding) == 1024


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
