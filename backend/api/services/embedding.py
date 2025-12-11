"""Embedding service wrapper for Cohere embeddings."""

import logging
from main import EmbeddingClient
import cohere

logger = logging.getLogger(__name__)


async def get_embedding_service(api_key: str, batch_size: int = 1) -> EmbeddingClient:
    """
    Create an embedding service for query embeddings.

    Args:
        api_key: Cohere API key
        batch_size: Batch size for embedding requests (default 1 for single queries)

    Returns:
        EmbeddingClient configured for search queries
    """
    logger.info("Initializing Cohere embedding client for queries")
    client = cohere.ClientV2(api_key=api_key)
    return EmbeddingClient(
        client=client,
        model="embed-english-v3.0",
        input_type="search_query",  # For queries, not documents
        batch_size=batch_size,
    )
