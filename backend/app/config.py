"""
Configuration management for RAG Chatbot backend.

Loads environment variables and provides typed configuration settings.
"""

from pydantic_settings import BaseSettings
from typing import List, Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Cohere API
    cohere_api_key: str = "test_cohere_api_key_placeholder"

    # Qdrant Vector Database
    qdrant_url: str = "http://localhost:6333"
    qdrant_api_key: str = "test_qdrant_api_key_placeholder"
    qdrant_collection: str = "book_vectors"

    # Neon PostgreSQL
    database_url: Optional[str] = None

    # FastAPI
    environment: str = "development"
    log_level: str = "INFO"
    debug: bool = False

    # Chunking
    chunking_strategy: str = "semantic"
    chunk_size: int = 300
    chunk_max_size: int = 500
    chunk_overlap: int = 200

    # API
    api_timeout: int = 30
    query_timeout: int = 5
    max_query_length: int = 5000

    # Security
    allowed_origins: List[str] = []
    cors_allow_credentials: bool = True
    https_only: bool = False

    def __init__(self, **data):
        super().__init__(**data)
        # Parse CORS_ORIGINS from environment if not set via allowed_origins
        import os
        from dotenv import load_dotenv

        load_dotenv()

        if not self.allowed_origins:
            cors_origins = os.getenv("CORS_ORIGINS")
            if cors_origins:
                self.allowed_origins = [origin.strip() for origin in cors_origins.split(",")]
            else:
                # Fallback defaults
                self.allowed_origins = [
                    "http://localhost:3000",
                    "http://localhost:3002",
                    "https://physical-ai-textbook-two.vercel.app"
                ]

    # Book Indexing
    max_book_pages: int = 500
    supported_formats: List[str] = ["pdf", "txt", "md"]
    index_batch_size: int = 10

    # Logging
    query_logging_enabled: bool = True
    log_dir: str = "./logs"
    accuracy_metrics_file: str = "./metrics/accuracy.json"

    # Select-Text Feature
    select_text_min_chars: int = 10
    select_text_constraint: bool = True

    # Retrieval Parameters
    retrieval_top_k: int = 20
    retrieval_threshold: float = 0.5
    rerank_top_k: int = 8

    class Config:
        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore extra environment variables


def get_settings() -> Settings:
    """Get application settings."""
    return Settings()
