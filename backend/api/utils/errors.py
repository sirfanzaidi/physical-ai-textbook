"""Custom exception classes for RAG API."""


class RAGError(Exception):
    """Base exception for RAG system."""

    def __init__(self, message: str, status_code: int = 500, error_type: str = "RAGError"):
        """Initialize RAG error.

        Args:
            message: Error message
            status_code: HTTP status code
            error_type: Type of error for client
        """
        super().__init__(message)
        self.message = message
        self.status_code = status_code
        self.error_type = error_type


class ValidationError(RAGError):
    """Raised when input validation fails."""

    def __init__(self, message: str):
        super().__init__(message, status_code=400, error_type="ValidationError")


class RetrievalError(RAGError):
    """Raised when retrieval from vector DB fails."""

    def __init__(self, message: str):
        super().__init__(message, status_code=503, error_type="RetrievalError")


class GenerationError(RAGError):
    """Raised when LLM generation fails."""

    def __init__(self, message: str):
        super().__init__(message, status_code=502, error_type="GenerationError")


class OpenRouterError(RAGError):
    """Raised when OpenRouter API call fails."""

    def __init__(self, message: str, status_code: int = 502):
        super().__init__(message, status_code=status_code, error_type="OpenRouterError")


class QdrantError(RAGError):
    """Raised when Qdrant connection or operation fails."""

    def __init__(self, message: str):
        super().__init__(message, status_code=503, error_type="QdrantError")


class ConfigurationError(RAGError):
    """Raised when configuration is missing or invalid."""

    def __init__(self, message: str):
        super().__init__(message, status_code=500, error_type="ConfigurationError")
