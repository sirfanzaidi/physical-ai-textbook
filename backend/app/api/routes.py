"""
API routes for RAG Chatbot.

Defines endpoints for book ingestion, chat queries, and management.
"""

from fastapi import APIRouter, UploadFile, File, HTTPException, Depends
from fastapi.responses import JSONResponse
import structlog
from typing import Optional
import asyncio

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
from ingestion.main import BookIngestionPipeline
from ingestion.chunker import SemanticChunker
from ingestion.embedder import CohereEmbedder
from database.qdrant_client import get_qdrant_store
from database.postgres_client import get_postgres_store
from generation.rag_chat import RAGChatbot
from retrieval.retriever import SemanticRetriever
from retrieval.reranker import CohereReranker
from retrieval.augmenter import DocumentAugmenter
from utils.errors import IngestError, GenerationError, ValidationError

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
    try:
        # Generate book_id if not provided
        if not book_id:
            book_id = f"book_{file.filename.split('.')[0].replace(' ', '_').lower()}"

        # Read file content
        file_content = await file.read()

        # Initialize pipeline components
        chunker = SemanticChunker(
            chunk_size=settings.chunk_size,
            chunk_max_size=settings.chunk_max_size,
            chunk_overlap=settings.chunk_overlap,
        )

        embedder = CohereEmbedder(api_key=settings.cohere_api_key)

        vector_store = get_qdrant_store(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            collection_name=settings.qdrant_collection,
        )

        metadata_store = None
        if settings.database_url:
            metadata_store = get_postgres_store(settings.database_url)

        # Create and execute ingestion pipeline
        pipeline = BookIngestionPipeline(
            chunker=chunker,
            embedder=embedder,
            vector_store=vector_store,
            metadata_store=metadata_store,
            max_pages=settings.max_book_pages,
        )

        result = pipeline.ingest_file(
            file_content=file_content,
            file_name=file.filename,
            book_id=book_id,
        )

        return IngestResponse(
            success=True,
            book_id=result["book_id"],
            chunk_count=result["chunk_count"],
            total_tokens=result["total_tokens"],
            message=result["message"],
        )

    except ValidationError as e:
        logger.warning("ingestion_validation_failed", file=file.filename, error=e.message)
        raise HTTPException(status_code=422, detail=str(e.message))
    except IngestError as e:
        logger.error("ingestion_failed", file=file.filename, error=e.message)
        raise HTTPException(status_code=400, detail=str(e.message))
    except Exception as e:
        logger.error("ingestion_unexpected_error", file=file.filename, error=str(e))
        raise HTTPException(status_code=500, detail="Internal server error during ingestion")


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
    try:
        # Validate request
        if request.mode == "selected" and not request.selected_text:
            raise ValidationError(
                "selected_text required for 'selected' mode",
                error_code="MISSING_SELECTED_TEXT",
            )

        # Initialize RAG pipeline components
        embedder = CohereEmbedder(api_key=settings.cohere_api_key)

        vector_store = get_qdrant_store(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            collection_name=settings.qdrant_collection,
        )

        retriever = SemanticRetriever(vector_store)
        reranker = CohereReranker(api_key=settings.cohere_api_key)
        augmenter = DocumentAugmenter()

        chatbot = RAGChatbot(
            api_key=settings.cohere_api_key,
            embedder=embedder,
            retriever=retriever,
            reranker=reranker,
            augmenter=augmenter,
        )

        # Execute RAG query
        result = chatbot.chat(
            query=request.query,
            book_id=request.book_id,
            mode=request.mode,
            selected_text=request.selected_text,
            top_k_retrieve=settings.retrieval_top_k,
            top_k_rerank=settings.rerank_top_k,
        )

        # Format response
        citations = [
            Citation(
                page_num=citation.get("page_num"),
                section_name=citation.get("section_name"),
                chapter_name=citation.get("chapter_name"),
                text_snippet=citation.get("text_snippet"),
            )
            for citation in result.get("citations", [])
        ]

        return ChatResponse(
            response=result["response"],
            citations=citations,
            confidence=result["confidence"],
            latency_ms=result["latency_ms"],
        )

    except ValidationError as e:
        logger.warning("chat_validation_failed", query=request.query, error=e.message)
        raise HTTPException(status_code=422, detail=str(e.message))
    except GenerationError as e:
        logger.error("chat_generation_failed", book_id=request.book_id, error=e.message)
        raise HTTPException(status_code=400, detail=str(e.message))
    except Exception as e:
        logger.error("chat_unexpected_error", book_id=request.book_id, error=str(e))
        raise HTTPException(status_code=500, detail="Internal server error during chat")


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
