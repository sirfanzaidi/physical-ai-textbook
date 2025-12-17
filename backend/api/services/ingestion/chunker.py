"""Semantic text chunking utilities."""

from api.utils import get_logger

logger = get_logger(__name__)


def find_sentence_boundary(text: str, position: int, search_range: int = 50) -> int:
    """Find nearest sentence boundary to a position.

    Args:
        text: The full text to search
        position: Target position to find boundary near
        search_range: How far to search in each direction

    Returns:
        Position of the nearest sentence boundary
    """
    if position >= len(text):
        return len(text)

    sentence_enders = ".!?"
    start_search = max(0, position - search_range)
    end_search = min(len(text), position + search_range)

    best_boundary = position
    min_distance = search_range + 1

    for i in range(start_search, end_search):
        if i < len(text) - 1 and text[i] in sentence_enders and text[i + 1].isspace():
            distance = abs(i + 1 - position)
            if distance < min_distance:
                min_distance = distance
                best_boundary = i + 1

    return best_boundary


def chunk_text_semantically(
    text: str,
    min_size: int = 800,
    max_size: int = 1200,
    overlap: int = 175,
) -> list[str]:
    """Split text into semantic chunks respecting sentence boundaries.

    Args:
        text: Full text to chunk
        min_size: Minimum chunk size in characters
        max_size: Maximum chunk size in characters
        overlap: Overlap between consecutive chunks

    Returns:
        List of text chunks
    """
    if not text or len(text) < min_size:
        return [text] if text else []

    chunks: list[str] = []
    target_size = (min_size + max_size) // 2  # Target middle of range
    position = 0

    while position < len(text):
        # Calculate end position for this chunk
        end_position = position + target_size

        if end_position >= len(text):
            # Last chunk - include everything remaining
            chunk = text[position:].strip()
            if chunk:
                chunks.append(chunk)
            break

        # Find sentence boundary near target end
        boundary = find_sentence_boundary(text, end_position)

        # Ensure chunk is within size limits
        chunk_end = boundary
        if chunk_end - position > max_size:
            # Too long - truncate at max_size, find sentence boundary there
            chunk_end = find_sentence_boundary(text, position + max_size, search_range=30)
        elif chunk_end - position < min_size:
            # Too short - extend to min_size
            chunk_end = find_sentence_boundary(text, position + min_size, search_range=30)

        # Extract chunk
        chunk = text[position:chunk_end].strip()
        if chunk:
            chunks.append(chunk)

        # Move position with overlap (go back by overlap amount)
        position = max(position + 1, chunk_end - overlap)

    # Filter out very short chunks that might be artifacts
    chunks = [c for c in chunks if len(c) >= min_size // 2]

    logger.debug(f"Created {len(chunks)} chunks from {len(text)} characters")
    return chunks
