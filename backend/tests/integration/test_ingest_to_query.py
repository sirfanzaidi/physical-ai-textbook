"""
Integration tests for complete RAG pipeline.

Tests end-to-end flow from book ingestion to query response.
Note: Requires real Cohere, Qdrant, and optional Neon credentials.
"""

import pytest
import sys
import os
import io

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from ingestion.chunker import SemanticChunker
from ingestion.embedder import CohereEmbedder
from ingestion.main import BookIngestionPipeline
from database.qdrant_client import QdrantVectorStore
from generation.prompts import get_system_prompt, get_user_prompt
from utils.errors import IngestError, ValidationError


class TestIngestionPipeline:
    """Test book ingestion pipeline end-to-end."""

    @pytest.fixture
    def sample_text(self):
        """Create sample text for testing."""
        return """# Chapter 1: Introduction to AI

Artificial Intelligence has revolutionized modern computing. Machine learning models can now
process vast amounts of data and extract meaningful patterns. Deep learning, a subset of machine
learning, uses neural networks with multiple layers to learn hierarchical representations.

## Section 1.1: Key Concepts

The fundamental concepts of AI include:
- Supervised Learning: Learning from labeled data
- Unsupervised Learning: Finding patterns in unlabeled data
- Reinforcement Learning: Learning through interaction and rewards

Each of these approaches has its own applications and limitations.

## Section 1.2: Applications

AI is used in many fields:
1. Computer Vision: Image recognition, object detection
2. Natural Language Processing: Text analysis, machine translation
3. Robotics: Autonomous systems, human-robot interaction
4. Healthcare: Disease diagnosis, drug discovery

The future of AI promises even more exciting applications."""

    @pytest.fixture
    def chunker(self):
        """Create chunker for testing."""
        return SemanticChunker(
            chunk_size=300,
            chunk_max_size=500,
            chunk_overlap=200,
        )

    def test_chunking_produces_valid_chunks(self, chunker, sample_text):
        """Test chunking produces valid chunk structure."""
        chunks = chunker.chunk(
            text=sample_text,
            book_id="test_book",
            page_num=1,
            chapter_name="Chapter 1",
        )

        assert len(chunks) > 0

        for chunk in chunks:
            # Verify required fields
            assert chunk["chunk_id"]
            assert chunk["book_id"] == "test_book"
            assert chunk["text"]
            assert chunk["token_count"] > 0
            assert "sequence" in chunk

            # Verify metadata preservation
            assert chunk["page_num"] == 1
            assert chunk["chapter_name"] == "Chapter 1"

    def test_chunks_respect_token_limits(self, chunker, sample_text):
        """Test chunks are within token limits."""
        chunks = chunker.chunk(text=sample_text, book_id="test_book")

        for chunk in chunks:
            tokens = chunk["token_count"]
            # Chunks should be reasonably sized
            assert 10 < tokens < 1000

    def test_total_token_calculation(self, chunker, sample_text):
        """Test total token calculation is correct."""
        chunks = chunker.chunk(text=sample_text, book_id="test_book")
        total_tokens = chunker.calculate_total_tokens(chunks)

        assert total_tokens > 0
        assert total_tokens == sum(c["token_count"] for c in chunks)

    def test_chunk_sequence_ordering(self, chunker, sample_text):
        """Test chunks maintain sequence numbers."""
        chunks = chunker.chunk(text=sample_text, book_id="test_book")

        for i, chunk in enumerate(chunks):
            assert chunk["sequence"] == i


class TestPromptGeneration:
    """Test prompt generation for different modes."""

    @pytest.fixture
    def sample_context(self):
        """Create sample context."""
        return """AI has revolutionized computing. Machine learning models can process vast amounts
of data. Deep learning uses neural networks with multiple layers."""

    @pytest.fixture
    def sample_query(self):
        """Create sample query."""
        return "What is machine learning?"

    def test_rag_system_prompt(self):
        """Test RAG system prompt is generated."""
        prompt = get_system_prompt(mode="rag")
        assert prompt
        assert len(prompt) > 0

    def test_select_text_system_prompt(self):
        """Test select-text system prompt is generated."""
        prompt = get_system_prompt(mode="select_text")
        assert prompt
        assert len(prompt) > 0

    def test_rag_user_prompt(self, sample_context, sample_query):
        """Test RAG user prompt formatting."""
        prompt = get_user_prompt(
            mode="rag",
            context=sample_context,
            query=sample_query,
        )

        assert sample_context in prompt
        assert sample_query in prompt
        assert "CONTEXT:" in prompt
        assert "QUESTION:" in prompt

    def test_select_text_user_prompt(self, sample_context, sample_query):
        """Test select-text user prompt formatting."""
        prompt = get_user_prompt(
            mode="select_text",
            selected_text=sample_context,
            query=sample_query,
        )

        assert sample_context in prompt
        assert sample_query in prompt
        assert "PASSAGE:" in prompt
        assert "QUESTION:" in prompt


class TestFileValidation:
    """Test file validation during ingestion."""

    @pytest.fixture
    def chunker(self):
        """Create chunker."""
        return SemanticChunker()

    @pytest.fixture
    def mock_vector_store(self):
        """Create mock vector store."""
        class MockVectorStore:
            def upsert_vectors(self, chunk_ids, vectors, metadata):
                return len(chunk_ids)

        return MockVectorStore()

    @pytest.fixture
    def pipeline(self, chunker, mock_vector_store):
        """Create ingestion pipeline."""
        return BookIngestionPipeline(
            chunker=chunker,
            embedder=None,  # Not needed for validation tests
            vector_store=mock_vector_store,
            max_pages=500,
        )

    def test_file_type_detection(self, pipeline):
        """Test file type detection."""
        assert pipeline._detect_file_type("test.pdf") == "pdf"
        assert pipeline._detect_file_type("test.txt") == "txt"
        assert pipeline._detect_file_type("test.md") == "md"

    def test_unsupported_file_type_raises_error(self, pipeline):
        """Test unsupported file type raises error."""
        with pytest.raises(ValidationError):
            pipeline._detect_file_type("test.docx")

    def test_empty_file_validation(self, pipeline):
        """Test empty file validation."""
        with pytest.raises(ValidationError):
            pipeline._validate_file(b"", "test.txt", "txt", 0.001)

    def test_too_large_file_validation(self, pipeline):
        """Test file size validation."""
        with pytest.raises(ValidationError):
            pipeline._validate_file(b"x" * (51 * 1024 * 1024), "test.txt", "txt", 51)

    def test_valid_file_passes_validation(self, pipeline):
        """Test valid file passes validation."""
        # Should not raise
        pipeline._validate_file(b"x" * 1000, "test.txt", "txt", 0.001)

    def test_text_extraction_from_string(self, pipeline):
        """Test text extraction from text file."""
        test_content = "Hello world. This is a test."
        result = pipeline._extract_text_file(test_content.encode("utf-8"), "test.txt")

        assert result["text"] == test_content
        assert result["metadata"]["title"] == "test"
        assert result["metadata"]["file_type"] == "txt"

    def test_text_extraction_invalid_encoding(self, pipeline):
        """Test text extraction with invalid encoding."""
        # Invalid UTF-8 bytes
        invalid_bytes = b"\x80\x81\x82\x83"

        with pytest.raises(IngestError):
            pipeline._extract_text_file(invalid_bytes, "test.txt")


class TestPipelineStructure:
    """Test pipeline structure and integration."""

    def test_ingestion_pipeline_initialization(self):
        """Test pipeline initializes with required components."""
        chunker = SemanticChunker()

        class MockEmbedder:
            pass

        class MockVectorStore:
            pass

        pipeline = BookIngestionPipeline(
            chunker=chunker,
            embedder=MockEmbedder(),
            vector_store=MockVectorStore(),
        )

        assert pipeline.chunker is not None
        assert pipeline.embedder is not None
        assert pipeline.vector_store is not None
        assert pipeline.max_pages == 500

    def test_pipeline_configuration_validation(self):
        """Test pipeline validates max_pages configuration."""
        chunker = SemanticChunker()

        class MockEmbedder:
            pass

        class MockVectorStore:
            pass

        # Should initialize with custom max_pages
        pipeline = BookIngestionPipeline(
            chunker=chunker,
            embedder=MockEmbedder(),
            vector_store=MockVectorStore(),
            max_pages=1000,
        )

        assert pipeline.max_pages == 1000


class TestDataFlow:
    """Test data flow through pipeline stages."""

    def test_chunk_metadata_flows_through_pipeline(self):
        """Test metadata is preserved through pipeline."""
        chunker = SemanticChunker()
        sample_text = "Test paragraph. " * 30

        chunks = chunker.chunk(
            text=sample_text,
            book_id="book_123",
            page_num=5,
            chapter_name="Chapter 5",
            section_name="Section A",
        )

        # Verify metadata flows
        for chunk in chunks:
            assert chunk["book_id"] == "book_123"
            assert chunk["page_num"] == 5
            assert chunk["chapter_name"] == "Chapter 5"
            assert chunk["section_name"] == "Section A"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
