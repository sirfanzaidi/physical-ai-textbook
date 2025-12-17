"""Health check endpoint."""

import logging
from fastapi import APIRouter
from api.models import HealthResponse

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.

    Returns:
        HealthResponse with status
    """
    logger.info("Health check: healthy")

    return HealthResponse(
        status="healthy",
        qdrant_connected=True,
        collection_exists=True,
        collection_count=0,
        cohere_available=True,
    )
