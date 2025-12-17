"""Utilities package."""

from .errors import (
    RAGError,
    ValidationError,
    RetrievalError,
    GenerationError,
    OpenRouterError,
    QdrantError,
    ConfigurationError,
)
from .validators import (
    validate_query_length,
    validate_selected_text,
    validate_mode,
    validate_selected_text_with_mode,
    validate_book_id,
    validate_chunk_size,
)
from .logging import get_logger, TimingContext

__all__ = [
    "RAGError",
    "ValidationError",
    "RetrievalError",
    "GenerationError",
    "OpenRouterError",
    "QdrantError",
    "ConfigurationError",
    "validate_query_length",
    "validate_selected_text",
    "validate_mode",
    "validate_selected_text_with_mode",
    "validate_book_id",
    "validate_chunk_size",
    "get_logger",
    "TimingContext",
]
