"""Retrieval service wrapper for Qdrant semantic search."""

import logging
from main import QdrantManager
from qdrant_client import QdrantClient

logger = logging.getLogger(__name__)


async def get_retrieval_service(
    qdrant_url: str, qdrant_api_key: str, collection_name: str = "book_chunks"
) -> QdrantManager:
    """
    Create a retrieval service for semantic search.

    Args:
        qdrant_url: Qdrant server URL
        qdrant_api_key: Qdrant API key
        collection_name: Name of the collection

    Returns:
        QdrantManager configured for semantic search
    """
    logger.info(f"Initializing Qdrant client for collection '{collection_name}'")
    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    return QdrantManager(client=client, collection_name=collection_name)
