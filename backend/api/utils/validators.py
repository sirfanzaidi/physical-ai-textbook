"""Input validation utilities for RAG API."""

from typing import Optional
from .errors import ValidationError


def validate_query_length(query: str, max_length: int = 5000) -> None:
    """Validate query length.

    Args:
        query: Query text to validate
        max_length: Maximum allowed length

    Raises:
        ValidationError: If query is invalid
    """
    if not query or not query.strip():
        raise ValidationError("Query cannot be empty")
    if len(query) > max_length:
        raise ValidationError(f"Query exceeds maximum length of {max_length} characters")


def validate_selected_text(selected_text: Optional[str], min_length: int = 10) -> None:
    """Validate selected text.

    Args:
        selected_text: Selected text to validate
        min_length: Minimum allowed length

    Raises:
        ValidationError: If selected text is invalid
    """
    if selected_text is None:
        return

    if not selected_text.strip():
        raise ValidationError("Selected text cannot be empty or whitespace only")
    if len(selected_text) < min_length:
        raise ValidationError(
            f"Selected text must be at least {min_length} characters (got {len(selected_text)})"
        )


def validate_mode(mode: str) -> None:
    """Validate query mode.

    Args:
        mode: Query mode (full or selected)

    Raises:
        ValidationError: If mode is invalid
    """
    valid_modes = {"full", "selected"}
    if mode not in valid_modes:
        raise ValidationError(f"Invalid mode '{mode}'. Must be one of {valid_modes}")


def validate_selected_text_with_mode(mode: str, selected_text: Optional[str]) -> None:
    """Validate selected_text is provided when mode is 'selected'.

    Args:
        mode: Query mode
        selected_text: Selected text

    Raises:
        ValidationError: If validation fails
    """
    if mode == "selected" and not selected_text:
        raise ValidationError("selected_text is required when mode='selected'")


def validate_book_id(book_id: str) -> None:
    """Validate book ID format.

    Args:
        book_id: Book identifier to validate

    Raises:
        ValidationError: If book ID is invalid
    """
    if not book_id or not book_id.strip():
        raise ValidationError("book_id cannot be empty")
    if len(book_id) > 100:
        raise ValidationError("book_id exceeds maximum length of 100 characters")


def validate_chunk_size(chunk_size: int, chunk_overlap: int) -> None:
    """Validate chunking parameters.

    Args:
        chunk_size: Size of chunks
        chunk_overlap: Overlap between chunks

    Raises:
        ValidationError: If parameters are invalid
    """
    if chunk_size <= 0:
        raise ValidationError("chunk_size must be positive")
    if chunk_overlap < 0:
        raise ValidationError("chunk_overlap cannot be negative")
    if chunk_overlap >= chunk_size:
        raise ValidationError("chunk_overlap must be less than chunk_size")
