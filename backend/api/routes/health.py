"""Health check endpoint."""

import logging
from fastapi import APIRouter
from ..models import HealthResponse

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.

    Returns:
        HealthResponse with status and actual Qdrant collection count
    """
    from .chat import _qdrant_store

    qdrant_connected = False
    collection_exists = False
    collection_count = 0
    cohere_available = True
    status = "healthy"
    message = "RAG system is healthy"

    # Check if Qdrant store is initialized
    if _qdrant_store is not None:
        try:
            # Get actual collection info from Qdrant
            collection_info = await _qdrant_store.get_collection_info()

            if collection_info:
                qdrant_connected = True
                collection_exists = True
                collection_count = collection_info.get("points_count", 0)

                if collection_count == 0:
                    message = "RAG system is healthy but no content ingested yet"
            else:
                message = "Qdrant collection not found"
                status = "degraded"
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            message = "Qdrant connection failed"
            status = "degraded"
    else:
        message = "RAG system not initialized yet"
        status = "degraded"

    logger.info(f"Health check: {status} - {collection_count} chunks in database")

    return HealthResponse(
        status=status,
        qdrant_connected=qdrant_connected,
        collection_exists=collection_exists,
        collection_count=collection_count,
        cohere_available=cohere_available,
        message=message
    )
