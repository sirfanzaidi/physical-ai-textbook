"""Ingestion endpoint for loading and embedding content."""

import uuid
from typing import List
from fastapi import APIRouter, Depends, HTTPException
from api.config import Settings
from api.models import IngestRequest, IngestResponse, ErrorResponse
from api.services.ingestion import IngestionService
from api.utils import get_logger, RAGError

logger = get_logger(__name__)

router = APIRouter(prefix="/api", tags=["ingestion"])

# Global ingestion service instance
_ingestion_service: IngestionService = None


def get_ingestion_service(settings: Settings = Depends(lambda: Settings())) -> IngestionService:
    """Dependency to get ingestion service instance."""
    global _ingestion_service
    if _ingestion_service is None:
        raise HTTPException(
            status_code=503,
            detail="Ingestion service not initialized. Check logs for startup errors.",
        )
    return _ingestion_service


async def initialize_ingestion_service(
    openrouter_client,
    qdrant_store,
    settings: Settings,
) -> None:
    """Initialize ingestion service on startup."""
    global _ingestion_service

    try:
        logger.info("Initializing ingestion service...")
        _ingestion_service = IngestionService(
            settings=settings,
            openrouter_client=openrouter_client,
            qdrant_store=qdrant_store,
        )
        logger.info("✓ Ingestion service initialized")
    except Exception as e:
        logger.error(f"Failed to initialize ingestion service: {e}")
        raise


@router.post("/ingest")
async def ingest_content(
    request: IngestRequest,
    service: IngestionService = Depends(get_ingestion_service),
) -> IngestResponse:
    """Ingest content from various sources.

    Supports:
    - docusaurus: Load and chunk Docusaurus markdown files
    - markdown: Load from markdown files
    - html: Load from HTML content

    Request body:
    ```json
    {
        "source": "docusaurus",
        "docs_path": "/path/to/docs or https://example.com",
        "book_id": "physical-ai",
        "chunk_size": 1000,
        "chunk_overlap": 200
    }
    ```
    """
    request_id = str(uuid.uuid4())

    try:
        logger.info(
            f"[{request_id}] Ingestion request: source={request.source}, "
            f"chunk_size={request.chunk_size}"
        )

        if request.source == "docusaurus":
            # For now, return placeholder response
            # Full implementation would fetch and parse Docusaurus content
            logger.info(f"[{request_id}] Docusaurus ingestion not yet fully implemented")
            return IngestResponse(
                status="pending",
                chunks_processed=0,
                chunks_stored=0,
                duration_seconds=0.0,
                message="Docusaurus ingestion support coming in next phase",
                errors=["Manual ingestion via backend/main.py recommended for now"],
            )

        elif request.source == "markdown":
            return IngestResponse(
                status="pending",
                chunks_processed=0,
                chunks_stored=0,
                duration_seconds=0.0,
                message="Markdown ingestion not yet implemented",
                errors=["Feature coming in next phase"],
            )

        else:
            raise ValueError(f"Unsupported source: {request.source}")

    except ValueError as e:
        logger.warning(f"[{request_id}] Validation error: {e}")
        raise HTTPException(status_code=400, detail=str(e))

    except RAGError as e:
        logger.error(f"[{request_id}] RAG error: {e.message}")
        raise HTTPException(status_code=e.status_code, detail=e.message)

    except Exception as e:
        logger.error(f"[{request_id}] Unexpected error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/ingest/status")
async def ingest_status(
    service: IngestionService = Depends(get_ingestion_service),
) -> dict:
    """Get ingestion collection status."""
    try:
        info = await service.verify_ingestion()
        return info
    except Exception as e:
        logger.error(f"Failed to get ingestion status: {e}")
        raise HTTPException(status_code=500, detail="Failed to get ingestion status")
