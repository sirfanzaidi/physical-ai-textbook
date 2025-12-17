"""Chat endpoint for RAG query processing."""

import uuid
import json
from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from api.config import Settings
from api.models import ChatRequest, ChatResponse, ErrorResponse
from api.services import OpenRouterClient, QdrantStore, RAGChatbot
from api.utils import (
    get_logger,
    validate_query_length,
    validate_selected_text,
    validate_mode,
    validate_selected_text_with_mode,
    RAGError,
)

logger = get_logger(__name__)

router = APIRouter(prefix="/api", tags=["chat"])

# Global service instances (will be initialized in main app startup)
_openrouter_client: OpenRouterClient = None
_qdrant_store: QdrantStore = None
_rag_chatbot: RAGChatbot = None


def get_rag_chatbot(settings: Settings = Depends(lambda: Settings())) -> RAGChatbot:
    """Dependency to get RAG chatbot instance."""
    global _rag_chatbot
    if _rag_chatbot is None:
        raise HTTPException(
            status_code=503,
            detail="RAG system not initialized. Check logs for startup errors.",
        )
    return _rag_chatbot


async def initialize_services(settings: Settings) -> None:
    """Initialize service instances on startup."""
    global _openrouter_client, _qdrant_store, _rag_chatbot

    try:
        logger.info("Initializing RAG services...")

        # Initialize OpenRouter client
        _openrouter_client = OpenRouterClient(settings)
        logger.info("✓ OpenRouter client initialized")

        # Initialize Qdrant store
        _qdrant_store = QdrantStore(settings)
        await _qdrant_store.initialize_collection(vector_size=1024)
        logger.info("✓ Qdrant store initialized")

        # Initialize RAG chatbot
        _rag_chatbot = RAGChatbot(settings, _openrouter_client, _qdrant_store)
        logger.info("✓ RAG chatbot initialized")

        logger.info("All RAG services initialized successfully")

    except Exception as e:
        logger.error(f"Failed to initialize RAG services: {e}")
        raise


@router.post("/chat-stream", response_class=StreamingResponse)
async def chat_stream(
    request: ChatRequest,
    chatbot: RAGChatbot = Depends(get_rag_chatbot),
    settings: Settings = Depends(lambda: Settings()),
) -> StreamingResponse:
    """Stream chat responses with RAG augmentation.

    Query format:
    ```json
    {
        "query": "What is reinforcement learning?",
        "mode": "full",
        "book_id": "physical-ai",
        "top_k": 5
    }
    ```

    Or with selected text:
    ```json
    {
        "query": "Explain this concept",
        "mode": "selected",
        "selected_text": "The human brain uses electrical signals...",
        "book_id": "physical-ai"
    }
    ```
    """
    request_id = str(uuid.uuid4())

    try:
        logger.info(f"[{request_id}] Chat request: mode={request.mode}, top_k={request.top_k}")

        # Validate input
        validate_query_length(request.query, max_length=5000)
        validate_mode(request.mode)
        validate_selected_text(request.selected_text, min_length=10)
        validate_selected_text_with_mode(request.mode, request.selected_text)

        logger.debug(f"[{request_id}] Input validation passed")

        # Generate streaming response
        async def generate():
            try:
                chunk_index = 0

                async for chunk_dict in chatbot.answer_query_stream(
                    query=request.query,
                    mode=request.mode,
                    selected_text=request.selected_text,
                    top_k=request.top_k,
                ):
                    # Convert chunk to JSON line
                    if chunk_dict["type"] == "citations":
                        json_line = json.dumps({
                            "type": "citations",
                            "citations": [
                                {
                                    "chunk_id": c["chunk_id"],
                                    "chapter_name": c["chapter_name"],
                                    "section_name": c["section_name"],
                                    "source_url": c["source_url"],
                                    "page_num": c["page_num"],
                                }
                                for c in chunk_dict["citations"]
                            ],
                        })
                    elif chunk_dict["type"] == "text":
                        json_line = json.dumps({
                            "type": "text",
                            "content": chunk_dict["content"],
                            "index": chunk_index,
                        })
                        chunk_index += 1
                    elif chunk_dict["type"] == "metadata":
                        json_line = json.dumps({
                            "type": "metadata",
                            "latency_ms": chunk_dict["latency_ms"],
                            "model": chunk_dict["model"],
                            "mode": chunk_dict["mode"],
                            "request_id": request_id,
                        })
                    else:
                        continue

                    yield json_line + "\n"

                logger.info(f"[{request_id}] Chat response streamed successfully")

            except RAGError as e:
                logger.error(f"[{request_id}] RAG error: {e.message}")
                error_response = json.dumps({
                    "type": "error",
                    "error": e.error_type,
                    "message": e.message,
                    "request_id": request_id,
                })
                yield error_response + "\n"

            except Exception as e:
                logger.error(f"[{request_id}] Unexpected error: {e}")
                error_response = json.dumps({
                    "type": "error",
                    "error": "InternalServerError",
                    "message": "An unexpected error occurred while processing your query.",
                    "request_id": request_id,
                })
                yield error_response + "\n"

        return StreamingResponse(
            generate(),
            media_type="application/x-ndjson",
            headers={
                "X-Request-ID": request_id,
                "Cache-Control": "no-cache",
            },
        )

    except ValueError as e:
        logger.warning(f"[{request_id}] Validation error: {e}")
        raise HTTPException(status_code=400, detail=str(e))

    except RAGError as e:
        logger.error(f"[{request_id}] RAG error: {e.message}")
        raise HTTPException(status_code=e.status_code, detail=e.message)

    except Exception as e:
        logger.error(f"[{request_id}] Unexpected error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.post("/chat")
async def chat(
    request: ChatRequest,
    chatbot: RAGChatbot = Depends(get_rag_chatbot),
    settings: Settings = Depends(lambda: Settings()),
) -> ChatResponse:
    """Non-streaming chat endpoint for compatibility."""
    request_id = str(uuid.uuid4())

    try:
        logger.info(f"[{request_id}] Chat request: mode={request.mode}")

        # Validate input
        validate_query_length(request.query)
        validate_mode(request.mode)
        validate_selected_text(request.selected_text)
        validate_selected_text_with_mode(request.mode, request.selected_text)

        # Get answer
        result = await chatbot.answer_query(
            query=request.query,
            mode=request.mode,
            selected_text=request.selected_text,
            top_k=request.top_k,
        )

        response = ChatResponse(
            response_text=result["response_text"],
            citations=result["citations"],
            sources=result["sources"],
            latency_ms=result["latency_ms"],
            model=result["model"],
        )

        logger.info(f"[{request_id}] Chat response generated successfully")
        return response

    except ValueError as e:
        logger.warning(f"[{request_id}] Validation error: {e}")
        raise HTTPException(status_code=400, detail=str(e))

    except RAGError as e:
        logger.error(f"[{request_id}] RAG error: {e.message}")
        raise HTTPException(status_code=e.status_code, detail=e.message)

    except Exception as e:
        logger.error(f"[{request_id}] Unexpected error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")
