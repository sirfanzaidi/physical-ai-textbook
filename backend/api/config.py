"""API Configuration settings using Pydantic."""

from pydantic import field_validator
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # API Settings - CORS origins as comma-separated string in .env
    cors_origins: str = "http://localhost:3000,http://localhost:3002,http://localhost:8000"

    # OpenRouter Configuration
    openrouter_api_key: str = ""
    openrouter_base_url: str = "https://openrouter.ai/api/v1"
    embedding_model: str = "qwen/qwen3-embedding-4b"
    embedding_api_key: str = ""
    embedding_base_url: str = "https://openrouter.ai/api/v1"
    generation_model: str = "gpt-3.5-turbo"

    # Qdrant Configuration
    qdrant_url: str = "http://localhost:6333"
    qdrant_api_key: str = ""
    qdrant_collection_name: str = "physical_ai_textbook"

    # RAG Configuration
    embedding_dimension: int = 2560  # Qwen embeddings produce 2560-dim vectors
    chunk_size: int = 1000
    chunk_overlap: int = 200
    top_k_retrieval: int = 5
    min_selected_text_length: int = 10
    max_query_length: int = 5000
    streaming_chunk_size: int = 50
    response_timeout_seconds: int = 30

    # Logging
    log_level: str = "INFO"

    class Config:
        """Pydantic config."""

        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore unknown env vars
