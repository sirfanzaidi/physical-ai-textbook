"""Text chunking service for splitting chapters into RAG-friendly chunks."""

import logging
import re
from typing import List, Optional, Tuple
import time

logger = logging.getLogger(__name__)


class ChunkingService:
    """Service for chunking textbook chapters into semantic pieces for RAG."""

    def __init__(
        self,
        chunk_size: int = 400,  # tokens
        overlap_size: int = 100,  # tokens
        min_chunk_size: int = 100,
        max_chunk_size: int = 600
    ):
        """Initialize chunking service.

        Args:
            chunk_size: Target size in tokens (default: 400)
            overlap_size: Overlap between chunks in tokens (default: 100)
            min_chunk_size: Minimum chunk size (default: 100)
            max_chunk_size: Maximum chunk size (default: 600)
        """
        self.chunk_size = chunk_size
        self.overlap_size = overlap_size
        self.min_chunk_size = min_chunk_size
        self.max_chunk_size = max_chunk_size
        logger.info(
            f"ChunkingService initialized (size={chunk_size}, overlap={overlap_size}, "
            f"min={min_chunk_size}, max={max_chunk_size})"
        )

    def _estimate_tokens(self, text: str) -> int:
        """Estimate token count using simple heuristic.

        Uses approximation: ~4 characters = 1 token (based on English)

        Args:
            text: Text to estimate

        Returns:
            Approximate token count
        """
        # Remove extra whitespace
        text = " ".join(text.split())
        # Rough approximation: 1 token per 4 characters
        return max(1, len(text) // 4)

    def _split_on_sentence_boundaries(self, text: str) -> List[str]:
        """Split text on sentence boundaries.

        Args:
            text: Text to split

        Returns:
            List of sentences
        """
        # Split on sentence boundaries (. ! ? followed by space and capital letter)
        sentences = re.split(r'(?<=[.!?])\s+(?=[A-Z])', text)
        # Filter empty sentences
        return [s.strip() for s in sentences if s.strip()]

    def _split_on_paragraph_boundaries(self, text: str) -> List[str]:
        """Split text on paragraph boundaries.

        Args:
            text: Text to split

        Returns:
            List of paragraphs
        """
        # Split on blank lines (double newline)
        paragraphs = re.split(r'\n\s*\n', text)
        # Filter empty paragraphs
        return [p.strip() for p in paragraphs if p.strip()]

    def chunk_text(
        self,
        text: str,
        chapter_number: int,
        chapter_title: str,
        section_title: Optional[str] = None
    ) -> List[dict]:
        """Chunk a chapter/section into RAG-friendly pieces.

        Strategy:
        1. Split on paragraph boundaries
        2. Combine paragraphs into chunks of target size
        3. Add overlap between consecutive chunks

        Args:
            text: Chapter or section text to chunk
            chapter_number: Chapter number (1-6)
            chapter_title: Chapter title for metadata
            section_title: Optional section title within chapter

        Returns:
            List of dicts with 'id', 'content', 'metadata', 'token_count'
        """
        if not text or not text.strip():
            logger.warning(f"Empty text for chapter {chapter_number}")
            return []

        start_time = time.time()

        # Step 1: Split on paragraph boundaries for better semantic coherence
        paragraphs = self._split_on_paragraph_boundaries(text)
        if not paragraphs:
            logger.warning(f"No paragraphs found in chapter {chapter_number}")
            return []

        # Step 2: Combine paragraphs into chunks
        chunks = []
        current_chunk = ""
        current_tokens = 0
        overlap_text = ""  # For overlapping with next chunk
        chunk_index = 0

        for para in paragraphs:
            para_tokens = self._estimate_tokens(para)

            # If adding this paragraph would exceed max size, save current chunk
            if current_tokens + para_tokens > self.max_chunk_size and current_chunk:
                # Store overlap for next chunk
                sentences = self._split_on_sentence_boundaries(current_chunk)
                overlap_text = " ".join(sentences[-3:]) if len(sentences) > 3 else current_chunk

                chunks.append({
                    "id": f"ch{chapter_number}_chunk{chunk_index}",
                    "content": current_chunk.strip(),
                    "metadata": {
                        "chapter_number": chapter_number,
                        "chapter_title": chapter_title,
                        "section_title": section_title,
                        "chunk_index": chunk_index,
                        "token_count": current_tokens
                    },
                    "token_count": current_tokens
                })
                chunk_index += 1

                # Start new chunk with overlap
                current_chunk = overlap_text + " " + para if overlap_text else para
                current_tokens = self._estimate_tokens(overlap_text) + para_tokens if overlap_text else para_tokens

            # Add paragraph to current chunk
            else:
                if current_chunk:
                    current_chunk += "\n\n" + para
                else:
                    current_chunk = para
                current_tokens += para_tokens

        # Add final chunk
        if current_chunk and self._estimate_tokens(current_chunk) >= self.min_chunk_size:
            chunks.append({
                "id": f"ch{chapter_number}_chunk{chunk_index}",
                "content": current_chunk.strip(),
                "metadata": {
                    "chapter_number": chapter_number,
                    "chapter_title": chapter_title,
                    "section_title": section_title,
                    "chunk_index": chunk_index,
                    "token_count": current_tokens
                },
                "token_count": current_tokens
            })

        elapsed = time.time() - start_time
        logger.info(
            f"Chunked chapter {chapter_number} ({len(paragraphs)} paragraphs) "
            f"into {len(chunks)} chunks in {elapsed:.2f}s"
        )

        # Validate chunks
        for chunk in chunks:
            if chunk["token_count"] < self.min_chunk_size:
                logger.warning(f"Chunk {chunk['id']} is small: {chunk['token_count']} tokens")
            if chunk["token_count"] > self.max_chunk_size:
                logger.warning(f"Chunk {chunk['id']} exceeds max: {chunk['token_count']} tokens")

        return chunks

    def chunk_multiple(
        self,
        chapters: List[Tuple[int, str, str, str]]
    ) -> List[dict]:
        """Chunk multiple chapters.

        Args:
            chapters: List of (chapter_number, chapter_title, text, section_title) tuples

        Returns:
            Combined list of chunks from all chapters
        """
        all_chunks = []
        for chapter_number, chapter_title, text, section_title in chapters:
            chunks = self.chunk_text(text, chapter_number, chapter_title, section_title)
            all_chunks.extend(chunks)

        logger.info(f"Chunked {len(chapters)} chapters into {len(all_chunks)} total chunks")
        return all_chunks

    def get_chunk_stats(self, chunks: List[dict]) -> dict:
        """Get statistics about chunks.

        Args:
            chunks: List of chunks

        Returns:
            Stats dict with sizes, counts, etc.
        """
        if not chunks:
            return {
                "total_chunks": 0,
                "total_tokens": 0,
                "avg_chunk_size": 0,
                "min_chunk_size": 0,
                "max_chunk_size": 0
            }

        token_counts = [c["token_count"] for c in chunks]
        total_tokens = sum(token_counts)

        return {
            "total_chunks": len(chunks),
            "total_tokens": total_tokens,
            "avg_chunk_size": total_tokens // len(chunks) if chunks else 0,
            "min_chunk_size": min(token_counts),
            "max_chunk_size": max(token_counts),
            "chunks_by_chapter": self._group_chunks_by_chapter(chunks)
        }

    @staticmethod
    def _group_chunks_by_chapter(chunks: List[dict]) -> dict:
        """Group chunks by chapter.

        Args:
            chunks: List of chunks

        Returns:
            Dict with chapter_number -> chunk_count
        """
        grouped = {}
        for chunk in chunks:
            chapter = chunk["metadata"]["chapter_number"]
            if chapter not in grouped:
                grouped[chapter] = 0
            grouped[chapter] += 1
        return grouped
