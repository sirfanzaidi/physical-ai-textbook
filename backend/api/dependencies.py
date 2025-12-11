"""Dependency injection for FastAPI endpoints."""

import logging
from functools import lru_cache
from api.config import Settings
from api.services.embedding import get_embedding_service
from api.services.retrieval import get_retrieval_service

logger = logging.getLogger(__name__)

# Global client instances (singletons)
_embedding_client = None
_qdrant_manager = None


@lru_cache()
def get_settings() -> Settings:
    """Get application settings (cached)."""
    logger.info("Loading settings from environment")
    return Settings()


async def get_embedding_client():
    """
    Get embedding client (singleton).

    Returns:
        EmbeddingClient for query embeddings
    """
    global _embedding_client
    if _embedding_client is None:
        settings = get_settings()
        _embedding_client = await get_embedding_service(settings.cohere_api_key, batch_size=1)
        logger.info("Embedding client initialized")
    return _embedding_client


async def get_qdrant_manager():
    """
    Get Qdrant manager (singleton).

    Returns:
        QdrantManager for semantic search
    """
    global _qdrant_manager
    if _qdrant_manager is None:
        settings = get_settings()
        _qdrant_manager = await get_retrieval_service(
            qdrant_url=settings.qdrant_url,
            qdrant_api_key=settings.qdrant_api_key,
            collection_name=settings.collection_name,
        )
        logger.info(f"Qdrant manager initialized for collection '{settings.collection_name}'")
    return _qdrant_manager


def get_huggingface_api_key() -> str:
    """
    Get Hugging Face API key for LLM generation.

    Optional - if not provided, uses free public Hugging Face endpoints.

    Returns:
        Hugging Face API key (empty string if not configured)
    """
    settings = get_settings()
    api_key = settings.huggingface_api_key or ""
    if api_key:
        logger.info("Using Hugging Face API key for increased rate limits")
    else:
        logger.info("Using free Hugging Face public endpoint (30k requests/month)")
    return api_key
