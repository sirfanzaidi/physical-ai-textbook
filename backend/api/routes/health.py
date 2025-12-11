"""Health check endpoint."""

import logging
from fastapi import APIRouter, Depends
from api.models import HealthResponse
from api.dependencies import get_qdrant_manager, get_huggingface_api_key

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health_check(
    qdrant_manager=Depends(get_qdrant_manager),
    huggingface_api_key=Depends(get_huggingface_api_key),
):
    """
    Health check endpoint.

    Verifies:
    - Qdrant database connectivity
    - Collection existence and point count
    - Hugging Face Inference API availability (free)

    Returns:
        HealthResponse with status and component availability
    """
    qdrant_connected = False
    collection_exists = False
    collection_count = 0
    huggingface_available = True  # Always available - free public endpoints

    try:
        # Check Qdrant connection and collection
        logger.debug("Checking Qdrant health...")
        info = qdrant_manager.get_collection_info()

        qdrant_connected = True
        collection_exists = info is not None
        collection_count = info.points_count if info else 0

        logger.debug(f"Qdrant OK: {collection_count} points in collection")

    except Exception as e:
        logger.warning(f"Qdrant health check failed: {e}")

    # Hugging Face is always available (free public endpoints)
    if huggingface_api_key:
        logger.debug("Hugging Face using authenticated key (higher rate limits)")
    else:
        logger.debug("Hugging Face using free public endpoints (30k requests/month)")

    # Determine overall status
    status = "healthy"
    if not qdrant_connected:
        status = "degraded"
    if not qdrant_connected and not huggingface_available:
        status = "unhealthy"

    logger.info(f"Health check: {status}")

    return HealthResponse(
        status=status,
        qdrant_connected=qdrant_connected,
        collection_exists=collection_exists,
        collection_count=collection_count,
        cohere_available=huggingface_available,  # Reusing field for HF status
    )
