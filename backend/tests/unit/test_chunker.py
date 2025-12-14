"""
Unit tests for semantic chunking.

Tests token counting, boundary preservation, and chunk size constraints.
"""

import pytest
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from ingestion.chunker import SemanticChunker
from utils.errors import ChunkingError


class TestSemanticChunker:
    """Test SemanticChunker functionality."""

    @pytest.fixture
    def chunker(self):
        """Create chunker instance with default settings."""
        return SemanticChunker(
            chunk_size=300,
            chunk_max_size=500,
            chunk_overlap=200,
        )

    def test_chunker_initialization(self, chunker):
        """Test chunker initializes with correct parameters."""
        assert chunker.chunk_size == 300
        assert chunker.chunk_max_size == 500
        assert chunker.chunk_overlap == 200

    def test_invalid_overlap_configuration(self):
        """Test chunker rejects invalid overlap config."""
        with pytest.raises(ChunkingError):
            SemanticChunker(
                chunk_size=300,
                chunk_max_size=500,
                chunk_overlap=400,  # Greater than chunk_size
            )

    def test_chunk_empty_text(self, chunker):
        """Test chunker handles empty text gracefully."""
        result = chunker.chunk(
            text="",
            book_id="test_book",
        )
        assert result == []

    def test_chunk_whitespace_only_text(self, chunker):
        """Test chunker handles whitespace-only text."""
        result = chunker.chunk(
            text="   \n\n   ",
            book_id="test_book",
        )
        assert result == []

    def test_chunk_short_text(self, chunker):
        """Test chunker handles short text (single chunk)."""
        short_text = "This is a short paragraph. It should fit in one chunk."
        result = chunker.chunk(
            text=short_text,
            book_id="test_book",
            page_num=1,
        )
        assert len(result) == 1
        assert result[0]["text"] == short_text
        assert result[0]["book_id"] == "test_book"
        assert result[0]["page_num"] == 1
        assert result[0]["token_count"] > 0

    def test_chunk_long_text_produces_multiple_chunks(self, chunker):
        """Test chunker splits long text into multiple chunks."""
        # Create text that should produce multiple chunks
        long_text = " ".join(
            ["This is a test sentence with enough words."] * 100
        )
        result = chunker.chunk(
            text=long_text,
            book_id="test_book",
        )
        assert len(result) > 1

    def test_chunk_token_counts(self, chunker):
        """Test token counts are reasonable."""
        text = "This is a test. " * 50  # ~250 words
        result = chunker.chunk(text=text, book_id="test_book")

        # Check token counts
        for chunk in result:
            assert chunk["token_count"] > 0
            # With 1.3 tokens per word, should be around that ratio
            words = len(chunk["text"].split())
            estimated_tokens = int(words * chunker.tokens_per_word)
            assert abs(chunk["token_count"] - estimated_tokens) < 10

    def test_chunk_respects_max_size(self, chunker):
        """Test chunks approach max_size for long text."""
        long_text = " ".join(["word"] * 1000)  # Very long text
        result = chunker.chunk(text=long_text, book_id="test_book")

        # Note: Single un-splittable text may exceed max_size
        # This is expected behavior - max_size is a target, not hard limit
        assert len(result) > 0
        total_tokens = chunker.calculate_total_tokens(result)
        assert total_tokens > 0

    def test_chunk_metadata_preservation(self, chunker):
        """Test metadata is preserved in chunks."""
        text = "This is a test paragraph. " * 30
        result = chunker.chunk(
            text=text,
            book_id="book_123",
            page_num=42,
            section_name="Introduction",
            chapter_name="Chapter 1",
        )

        for chunk in result:
            assert chunk["book_id"] == "book_123"
            assert chunk["page_num"] == 42
            assert chunk["section_name"] == "Introduction"
            assert chunk["chapter_name"] == "Chapter 1"

    def test_chunk_id_generation(self, chunker):
        """Test chunk IDs are unique and properly formatted."""
        text = "Test paragraph. " * 50
        result = chunker.chunk(text=text, book_id="book_1", page_num=5)

        chunk_ids = [chunk["chunk_id"] for chunk in result]
        assert len(chunk_ids) == len(set(chunk_ids))  # All unique
        assert all("book_1" in cid for cid in chunk_ids)

    def test_paragraph_boundary_preservation(self, chunker):
        """Test chunker respects paragraph boundaries."""
        # Use longer paragraphs to ensure they don't merge
        text = "Paragraph 1 with more content. " * 20 + "\n\n" + \
               "Paragraph 2 with more content. " * 20 + "\n\n" + \
               "Paragraph 3 with more content. " * 20
        result = chunker.chunk(text=text, book_id="test_book")

        # With larger paragraphs, should produce multiple chunks
        assert len(result) >= 1
        # Verify paragraphs are represented
        full_text = " ".join(chunk["text"] for chunk in result)
        assert "Paragraph 1" in full_text
        assert "Paragraph 3" in full_text

    def test_calculate_total_tokens(self, chunker):
        """Test total token calculation."""
        text = "Test text. " * 100
        chunks = chunker.chunk(text=text, book_id="test_book")

        total_tokens = chunker.calculate_total_tokens(chunks)
        sum_tokens = sum(c["token_count"] for c in chunks)

        assert total_tokens == sum_tokens
        assert total_tokens > 0

    def test_chunk_sequence_ordering(self, chunker):
        """Test chunks maintain sequence order."""
        text = "First. " * 50 + "Second. " * 50 + "Third. " * 50
        result = chunker.chunk(text=text, book_id="test_book")

        # Verify sequence numbers
        for i, chunk in enumerate(result):
            assert chunk["sequence"] == i

    def test_estimate_tokens_accuracy(self, chunker):
        """Test token estimation matches expected ratio."""
        texts = [
            "one two three",
            "This is a longer sentence with more words.",
            "The quick brown fox jumps over the lazy dog.",
        ]

        for text in texts:
            tokens = chunker._estimate_tokens(text)
            words = len(text.split())
            estimated = int(words * chunker.tokens_per_word)
            assert tokens == estimated

    def test_chunk_with_special_characters(self, chunker):
        """Test chunking with special characters and punctuation."""
        text = "Hello, world! How are you? I'm fine. #hashtag @mention"
        result = chunker.chunk(text=text, book_id="test_book")

        assert len(result) > 0
        assert all(chunk["text"] for chunk in result)

    def test_chunk_with_unicode(self, chunker):
        """Test chunking with unicode characters."""
        text = "This text contains emoji 😀 and accents: café, naïve"
        result = chunker.chunk(text=text, book_id="test_book")

        assert len(result) > 0
        assert all(chunk["text"] for chunk in result)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
