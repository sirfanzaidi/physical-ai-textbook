"""Health check endpoint."""

import logging
from fastapi import APIRouter, Depends
from api.models import HealthResponse
from api.dependencies import get_qdrant_manager, get_openai_client

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health_check(
    qdrant_manager=Depends(get_qdrant_manager),
    openai_client=Depends(get_openai_client),
):
    """
    Health check endpoint.

    Verifies:
    - Qdrant database connectivity
    - Collection existence and point count
    - OpenAI API availability

    Returns:
        HealthResponse with status and component availability
    """
    qdrant_connected = False
    collection_exists = False
    collection_count = 0
    openai_available = False

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

    try:
        # Check OpenAI availability (simple client check)
        logger.debug("Checking OpenAI availability...")
        openai_available = openai_client is not None
        logger.debug("OpenAI OK")

    except Exception as e:
        logger.warning(f"OpenAI health check failed: {e}")

    # Determine overall status
    status = "healthy"
    if not qdrant_connected or not openai_available:
        status = "degraded"
    if not qdrant_connected and not openai_available:
        status = "unhealthy"

    logger.info(f"Health check: {status}")

    return HealthResponse(
        status=status,
        qdrant_connected=qdrant_connected,
        collection_exists=collection_exists,
        collection_count=collection_count,
        cohere_available=openai_available,
    )
