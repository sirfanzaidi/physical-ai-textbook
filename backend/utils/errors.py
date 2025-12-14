"""
Custom exception classes for RAG Chatbot application.

Defines domain-specific exceptions for different layers: external APIs, databases,
validation, and pipeline operations.
"""


class RAGChatbotException(Exception):
    """Base exception for all RAG Chatbot errors."""

    def __init__(self, message: str, error_code: str = None, details: dict = None):
        """
        Initialize exception with message, code, and details.

        Args:
            message: Human-readable error message
            error_code: Machine-readable error code for categorization
            details: Additional context dict for logging/debugging
        """
        self.message = message
        self.error_code = error_code or self.__class__.__name__
        self.details = details or {}
        super().__init__(self.message)


# ===== External Service Errors =====


class CoherAPIError(RAGChatbotException):
    """Raised when Cohere API call fails."""

    pass


class QdrantError(RAGChatbotException):
    """Raised when Qdrant vector database operation fails."""

    pass


class NeonError(RAGChatbotException):
    """Raised when Neon PostgreSQL operation fails."""

    pass


# ===== Validation Errors =====


class ValidationError(RAGChatbotException):
    """Raised when input validation fails."""

    pass


class QueryValidationError(ValidationError):
    """Raised when query validation fails."""

    pass


class BookValidationError(ValidationError):
    """Raised when book metadata validation fails."""

    pass


class SelectTextValidationError(ValidationError):
    """Raised when selected text validation fails."""

    pass


# ===== Pipeline Operation Errors =====


class IngestError(RAGChatbotException):
    """Raised during book ingestion pipeline."""

    pass


class ChunkingError(IngestError):
    """Raised during text chunking operation."""

    pass


class EmbeddingError(IngestError):
    """Raised during embedding generation."""

    pass


class RetrievalError(RAGChatbotException):
    """Raised during retrieval pipeline."""

    pass


class RerankError(RetrievalError):
    """Raised during reranking operation."""

    pass


class GenerationError(RAGChatbotException):
    """Raised during generation pipeline."""

    pass


class PromptError(GenerationError):
    """Raised when prompt construction fails."""

    pass


# ===== Business Logic Errors =====


class ZeroLeakageError(ValidationError):
    """Raised when select-text zero-leakage constraint is violated."""

    pass


class ConfidenceThresholdError(GenerationError):
    """Raised when response confidence is below threshold."""

    pass
