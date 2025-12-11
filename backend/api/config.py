"""API Configuration settings using Pydantic."""

from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Cohere API (for embeddings)
    cohere_api_key: str

    # OpenAI API (for answer generation)
    openai_api_key: str

    # Qdrant Vector Database
    qdrant_url: str
    qdrant_api_key: str

    # Textbook
    textbook_base_url: str

    # Collection & Embedding
    collection_name: str = "book_chunks"
    embedding_dimension: int = 1024

    # Retrieval Settings
    retrieval_limit: int = 5

    # Generation Settings (OpenAI)
    openai_model: str = "gpt-4o-mini"  # Free tier model
    max_tokens: int = 500
    temperature: float = 0.0

    # API Settings
    cors_origins: list[str] = [
        "https://physical-ai-textbook-two.vercel.app",
        "http://localhost:3000",
        "http://localhost:8000",
    ]

    # Logging
    log_level: str = "INFO"

    class Config:
        """Pydantic config."""

        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore unknown env vars
