"""
Semantic text chunking for RAG Chatbot.

Splits documents into semantic chunks respecting token limits and overlap.
"""

from typing import List, Dict, Any, Optional
import re
import structlog

from utils.errors import ChunkingError

logger = structlog.get_logger(__name__)


class SemanticChunker:
    """Chunks text documents into semantic units."""

    def __init__(
        self,
        chunk_size: int = 300,
        chunk_max_size: int = 500,
        chunk_overlap: int = 200,
        tokens_per_word: float = 1.3,
    ):
        """
        Initialize semantic chunker.

        Args:
            chunk_size: Target chunk size in tokens (300)
            chunk_max_size: Maximum chunk size in tokens (500)
            chunk_overlap: Overlap between chunks in tokens (200)
            tokens_per_word: Estimated tokens per word (default: 1.3 for Cohere)
        """
        self.chunk_size = chunk_size
        self.chunk_max_size = chunk_max_size
        self.chunk_overlap = chunk_overlap
        self.tokens_per_word = tokens_per_word

        if chunk_overlap >= chunk_size:
            raise ChunkingError(
                f"chunk_overlap ({chunk_overlap}) must be less than chunk_size ({chunk_size})",
                error_code="INVALID_CHUNKING_CONFIG",
            )

    def chunk(
        self,
        text: str,
        book_id: str,
        page_num: Optional[int] = None,
        section_name: Optional[str] = None,
        chapter_name: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """
        Chunk text into semantic units.

        Args:
            text: Text to chunk
            book_id: Source book identifier
            page_num: Page number (optional)
            section_name: Section name (optional)
            chapter_name: Chapter name (optional)

        Returns:
            List of chunk dicts with text, metadata, and token count

        Raises:
            ChunkingError: If chunking fails
        """
        try:
            if not text or not text.strip():
                return []

            # Split by semantic boundaries (paragraphs first, then sentences)
            chunks = self._split_semantic(text)

            # Merge chunks to meet size requirements
            merged_chunks = self._merge_chunks(chunks)

            # Create chunk objects with metadata
            chunk_objects = []
            for i, chunk_text in enumerate(merged_chunks):
                chunk_id = f"{book_id}_{page_num or 0}_{i:04d}"
                token_count = self._estimate_tokens(chunk_text)

                chunk_objects.append(
                    {
                        "chunk_id": chunk_id,
                        "book_id": book_id,
                        "text": chunk_text,
                        "page_num": page_num,
                        "section_name": section_name,
                        "chapter_name": chapter_name,
                        "token_count": token_count,
                        "sequence": i,
                    }
                )

            logger.info(
                "text_chunked",
                book_id=book_id,
                chunk_count=len(chunk_objects),
                avg_tokens=int(sum(c["token_count"] for c in chunk_objects) / len(chunk_objects)) if chunk_objects else 0,
            )
            return chunk_objects

        except ChunkingError:
            raise
        except Exception as e:
            raise ChunkingError(
                f"Failed to chunk text: {str(e)}",
                error_code="CHUNKING_FAILED",
                details={"book_id": book_id, "error": str(e)},
            )

    def _split_semantic(self, text: str) -> List[str]:
        """
        Split text by semantic boundaries (paragraphs, then sentences).

        Args:
            text: Text to split

        Returns:
            List of semantic chunks
        """
        # Split by double newlines (paragraphs)
        paragraphs = text.split("\n\n")
        paragraphs = [p.strip() for p in paragraphs if p.strip()]

        # If we have very long paragraphs, split by sentences
        chunks = []
        for para in paragraphs:
            if self._estimate_tokens(para) > self.chunk_max_size:
                sentences = self._split_sentences(para)
                chunks.extend(sentences)
            else:
                chunks.append(para)

        return chunks

    def _split_sentences(self, text: str) -> List[str]:
        """
        Split text by sentence boundaries.

        Args:
            text: Text to split

        Returns:
            List of sentences
        """
        # Pattern for sentence endings (period, question mark, exclamation)
        sentences = re.split(r'(?<=[.!?])\s+', text)
        return [s.strip() for s in sentences if s.strip()]

    def _merge_chunks(self, chunks: List[str]) -> List[str]:
        """
        Merge semantic chunks to meet size requirements.

        Args:
            chunks: List of semantic chunks

        Returns:
            List of chunks within size limits
        """
        merged = []
        current_chunk = ""

        for chunk in chunks:
            candidate = current_chunk + " " + chunk if current_chunk else chunk
            candidate_tokens = self._estimate_tokens(candidate)

            if candidate_tokens <= self.chunk_size:
                # Chunk fits within target size
                current_chunk = candidate
            elif candidate_tokens <= self.chunk_max_size:
                # Chunk exceeds target but within max
                current_chunk = candidate
            else:
                # Chunk exceeds max size
                if current_chunk:
                    merged.append(current_chunk)
                current_chunk = chunk

        if current_chunk:
            merged.append(current_chunk)

        return merged

    def _estimate_tokens(self, text: str) -> int:
        """
        Estimate token count for text.

        Args:
            text: Text to estimate

        Returns:
            Estimated token count
        """
        words = len(text.split())
        return int(words * self.tokens_per_word)

    def calculate_total_tokens(self, chunks: List[Dict[str, Any]]) -> int:
        """
        Calculate total tokens across all chunks.

        Args:
            chunks: List of chunk dicts

        Returns:
            Total token count
        """
        return sum(chunk.get("token_count", 0) for chunk in chunks)
