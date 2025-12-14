"""
API routes for RAG Chatbot.

Defines endpoints for book ingestion, chat queries, and management.
"""

from fastapi import APIRouter, UploadFile, File, HTTPException, Depends
from fastapi.responses import JSONResponse
import structlog
from typing import Optional

from app.config import Settings, get_settings
from app.api.models import (
    IngestRequest,
    IngestResponse,
    ChatRequest,
    ChatResponse,
    Citation,
    HealthResponse,
    ErrorResponse,
)

logger = structlog.get_logger(__name__)

# Create router for API endpoints
router = APIRouter(prefix="/api", tags=["rag-chatbot"])


# ===== Health Check =====


@router.get(
    "/health",
    response_model=HealthResponse,
    summary="Health check endpoint",
    description="Verify service health and external service connectivity",
)
async def health_check(settings: Settings = Depends(get_settings)) -> HealthResponse:
    """
    Health check endpoint.

    Returns service status and external service connectivity info.

    Returns:
        HealthResponse with status and service info
    """
    return HealthResponse(
        status="healthy",
        environment=settings.environment,
        services={
            "cohere": "connected",
            "qdrant": "connected",
            "neon": "connected" if settings.database_url else "not_configured",
        },
    )


# ===== Book Ingestion =====


@router.post(
    "/ingest",
    response_model=IngestResponse,
    summary="Ingest a book file",
    description="Upload and process a book file (PDF, TXT, or MD) for RAG indexing",
    status_code=200,
)
async def ingest_book(
    file: UploadFile = File(...),
    book_id: Optional[str] = None,
    settings: Settings = Depends(get_settings),
) -> IngestResponse:
    """
    Ingest a book file for RAG indexing.

    The endpoint will:
    1. Parse the file (PDF, TXT, or Markdown)
    2. Split into semantic chunks
    3. Generate embeddings
    4. Store in Qdrant vector database
    5. Store metadata in Neon PostgreSQL

    Args:
        file: Book file to ingest (PDF, TXT, or MD)
        book_id: Optional book identifier (auto-generated if not provided)
        settings: Application settings (injected)

    Returns:
        IngestResponse with ingestion results

    Raises:
        HTTPException: If ingestion fails (400, 422, 500)
    """
    # Stub: Implementation in Phase 3
    logger.info("ingest_endpoint_called", file=file.filename)

    return IngestResponse(
        success=True,
        book_id=book_id or f"book_{file.filename.replace('.', '_')}",
        chunk_count=0,  # Placeholder
        total_tokens=0,  # Placeholder
        message="Ingestion pipeline stub (not yet implemented)",
    )


# ===== Chat Queries =====


@router.post(
    "/chat",
    response_model=ChatResponse,
    summary="Query a book with RAG",
    description="Send a natural language query about a book and receive augmented responses",
    status_code=200,
)
async def chat(
    request: ChatRequest,
    settings: Settings = Depends(get_settings),
) -> ChatResponse:
    """
    Process a RAG chat query.

    Two query modes:
    - "full": Retrieve relevant context from entire book
    - "selected": Answer using only user-selected text (zero-leakage constraint)

    Args:
        request: ChatRequest with query, book_id, mode, and optional selected_text
        settings: Application settings (injected)

    Returns:
        ChatResponse with answer, citations, confidence, and latency

    Raises:
        HTTPException: If query fails (400, 422, 500)
    """
    # Stub: Implementation in Phase 3
    logger.info(
        "chat_endpoint_called",
        query_length=len(request.query),
        book_id=request.book_id,
        mode=request.mode,
    )

    return ChatResponse(
        response="Chat pipeline stub (not yet implemented)",
        citations=[
            Citation(
                page_num=None,
                section_name="Not Implemented",
                chapter_name=None,
                text_snippet=None,
            )
        ],
        confidence=0.0,
        latency_ms=0.0,
    )


# ===== Book Management =====


@router.get(
    "/books",
    summary="List indexed books",
    description="Get list of all books currently indexed in the system",
    status_code=200,
)
async def list_books(
    settings: Settings = Depends(get_settings),
) -> dict:
    """
    List all indexed books.

    Returns:
        Dict with list of books and their metadata
    """
    # Stub: Implementation in Phase 3
    logger.info("list_books_endpoint_called")

    return {
        "books": [],
        "count": 0,
        "message": "Book listing stub (not yet implemented)",
    }


@router.delete(
    "/books/{book_id}",
    summary="Delete a book",
    description="Remove a book and all its indexed content from the system",
    status_code=200,
)
async def delete_book(
    book_id: str,
    settings: Settings = Depends(get_settings),
) -> dict:
    """
    Delete a book and its indexed content.

    Removes:
    - All embeddings from Qdrant
    - All metadata from Neon
    - Associated chunk data

    Args:
        book_id: Book identifier to delete
        settings: Application settings (injected)

    Returns:
        Dict with deletion status
    """
    # Stub: Implementation in Phase 3
    logger.info("delete_book_endpoint_called", book_id=book_id)

    return {
        "success": True,
        "book_id": book_id,
        "message": "Book deletion stub (not yet implemented)",
    }


# ===== Error Handlers =====


@router.get("/error-test")
async def error_test() -> dict:
    """Test endpoint for error handling (development only)."""
    raise HTTPException(
        status_code=500,
        detail="Test error for debugging error handling",
    )
