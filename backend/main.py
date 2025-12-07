"""
Physical AI & Humanoid Robotics Textbook - RAG Chatbot Backend

FastAPI application providing:
- /chat endpoint for RAG queries
- /health endpoint for health checks
- /reindex endpoint for admin re-indexing
- /validate endpoint for RAG accuracy validation
"""

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
import logging
import os
from datetime import datetime
import time

# Import data models from src/models
from src.models import (
    ChatbotQuery,
    ChatbotResponse,
    Citation,
    ReindexRequest,
    ReindexResponse,
    ValidationRequest,
    ValidationResponse,
    ValidationResult,
    HealthResponse,
)

# Import services from src/services
from src.services import EmbeddingService, VectorDBService, ChunkingService, LLMService

# Configure logging
logging.basicConfig(level=os.getenv("LOG_LEVEL", "DEBUG"))
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title=os.getenv("API_TITLE", "Physical AI RAG Chatbot"),
    version=os.getenv("API_VERSION", "1.0.0"),
    description=os.getenv("API_DESCRIPTION", "RAG-powered chatbot for Physical AI textbook"),
)

# Configure CORS for local development
cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================================================
# Global Service Instances
# ============================================================================

_embedding_service: EmbeddingService = None
_vector_db_service: VectorDBService = None
_chunking_service: ChunkingService = None
_llm_service: LLMService = None
_startup_time: float = None


def get_embedding_service() -> EmbeddingService:
    """Get or initialize embedding service (lazy loading)."""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService(
            model_name=os.getenv("EMBEDDING_MODEL", "sentence-transformers/all-MiniLM-L6-v2"),
            batch_size=int(os.getenv("EMBEDDING_BATCH_SIZE", "32")),
        )
    return _embedding_service


def get_vector_db_service() -> VectorDBService:
    """Get or initialize vector database service (lazy loading)."""
    global _vector_db_service
    if _vector_db_service is None:
        _vector_db_service = VectorDBService(
            db_path=os.getenv("CHROMADB_PATH", "./data/embeddings"),
            collection_name="physical_ai_textbook",
            embedding_dimension=384,
        )
    return _vector_db_service


def get_chunking_service() -> ChunkingService:
    """Get or initialize chunking service (lazy loading)."""
    global _chunking_service
    if _chunking_service is None:
        _chunking_service = ChunkingService(
            chunk_size=int(os.getenv("CHUNK_SIZE", "400")),
            overlap_size=int(os.getenv("CHUNK_OVERLAP", "100")),
        )
    return _chunking_service


def get_llm_service() -> LLMService:
    """Get or initialize LLM service (lazy loading)."""
    global _llm_service
    if _llm_service is None:
        _llm_service = LLMService(
            model_name=os.getenv("LLM_MODEL", "template")
        )
    return _llm_service

# ============================================================================
# Endpoints
# ============================================================================

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint with dependency status."""
    try:
        embedding_service = get_embedding_service()
        vector_db_service = get_vector_db_service()

        dependencies = {
            "embedding_model": "ok" if embedding_service else "error",
            "vector_db": "ok",
        }

        # Try to get stats from vector DB
        db_stats = vector_db_service.get_db_stats()
        total_chunks = sum(c.get("chunk_count", 0) for c in db_stats.get("collections", {}).values())

        return HealthResponse(
            status="ok",
            version=os.getenv("API_VERSION", "1.0.0"),
            timestamp=datetime.utcnow(),
            dependencies=dependencies,
            uptime_seconds=time.time() - _startup_time if _startup_time else None,
            total_chunks_indexed=total_chunks,
            vector_db_size_mb=sum(c.get("db_size_bytes", 0) for c in db_stats.get("collections", {}).values()) / (1024 * 1024),
        )
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return HealthResponse(
            status="degraded",
            version=os.getenv("API_VERSION", "1.0.0"),
            timestamp=datetime.utcnow(),
            dependencies={"error": str(e)},
        )

@app.post("/chat", response_model=ChatbotResponse)
async def chat(query: ChatbotQuery) -> ChatbotResponse:
    """
    RAG Chatbot Query Endpoint

    Accepts a user query with optional selected text context.
    Returns a response sourced from book content with citations.

    Query Process:
    1. Embed user query using sentence-transformers/all-MiniLM-L6-v2
    2. Search ChromaDB for top-5 relevant chunks (200-500 tokens each)
    3. Generate response from retrieved chunks
    4. Return response with citations

    Constraints:
    - Response MUST be sourced from book text only (no hallucinations)
    - If no relevant chunks found, respond: "I don't have information about this in the textbook"
    - Response latency target: <2 seconds p95
    """

    logger.info(f"Query received: {query.question[:50]}...")
    overall_start = time.time()

    try:
        embedding_service = get_embedding_service()
        vector_db_service = get_vector_db_service()

        # Step 1: Embed query
        embed_start = time.time()
        query_embedding = embedding_service.embed_single(query.question)
        embed_time = int((time.time() - embed_start) * 1000)
        logger.debug(f"Query embedded in {embed_time}ms")

        # Step 2: Search ChromaDB
        search_start = time.time()
        search_results = vector_db_service.search(
            query_embedding=query_embedding,
            n_results=int(os.getenv("RAG_TOP_K", "5")),
        )
        search_time = search_results["search_time_ms"]
        logger.debug(f"Search completed in {search_time}ms, found {len(search_results['ids'])} results")

        # Step 3: Build response from retrieved chunks
        if not search_results["ids"]:
            return ChatbotResponse(
                response_text="I don't have information about this in the textbook. Please try a more specific question.",
                citations=[],
                confidence_score=0.0,
                source_chunk_ids=[],
                query_embedding_time_ms=embed_time,
                search_time_ms=search_time,
                generation_time_ms=0,
                total_time_ms=int((time.time() - overall_start) * 1000),
            )

        # Step 4: Create citations from search results
        citations = []
        for idx, (chunk_id, document, metadata, relevance) in enumerate(zip(
            search_results["ids"],
            search_results["documents"],
            search_results["metadatas"],
            search_results["relevance_scores"]
        )):
            if relevance >= float(os.getenv("RAG_RELEVANCE_THRESHOLD", "0.3")):
                citations.append(Citation(
                    chunk_id=chunk_id,
                    chapter_number=metadata.get("chapter_number", 0),
                    chapter_title=metadata.get("chapter_title", "Unknown"),
                    section_title=metadata.get("section_title"),
                    excerpt=document[:150] + "..." if len(document) > 150 else document,
                    relevance_score=relevance,
                ))

        # Step 5: Generate response using LLM service
        gen_start = time.time()
        llm_service = get_llm_service()

        # Prepare chunks for LLM
        chunks_for_llm = []
        for doc, metadata in zip(search_results["documents"], search_results["metadatas"]):
            chunks_for_llm.append({"content": doc, "metadata": metadata})

        response_text = llm_service.generate_response(
            query=query.question,
            chunks=chunks_for_llm,
            max_length=500
        )
        gen_time = int((time.time() - gen_start) * 1000)

        total_time = int((time.time() - overall_start) * 1000)

        return ChatbotResponse(
            response_text=response_text,
            citations=citations[:int(os.getenv("MAX_CITATIONS", "3"))],
            confidence_score=search_results["relevance_scores"][0] if search_results["relevance_scores"] else 0.0,
            source_chunk_ids=search_results["ids"][:int(os.getenv("RAG_TOP_K", "5"))],
            query_embedding_time_ms=embed_time,
            search_time_ms=search_time,
            generation_time_ms=gen_time,
            total_time_ms=total_time,
        )

    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/reindex", response_model=ReindexResponse)
async def reindex(request: ReindexRequest, background_tasks: BackgroundTasks) -> ReindexResponse:
    """
    Admin Endpoint: Re-index RAG Database

    Triggers re-indexing of chapters into ChromaDB.

    Modes:
    - 'delta': Only re-index modified chapters (fast, 7-13 sec)
    - 'full': Re-index all chapters (30-60 sec)

    Implementation:
    1. Detect modified chapters (git diff)
    2. Create new collection (green)
    3. Index chapters
    4. Validate with test queries (18+ per chapter)
    5. Atomic alias swap: prod → green
    6. Delete old collection (blue)

    During re-indexing, chatbot is available (serving old collection until swap).
    """

    logger.info(f"Re-index request received: mode={request.mode}")

    try:
        # TODO: Implement re-indexing logic with blue-green swap
        # 1. Detect changes
        # 2. Create new collection
        # 3. Index chapters
        # 4. Validate
        # 5. Atomic swap
        # 6. Clean up

        # Placeholder response (Phase 2 implementation)
        return ReindexResponse(
            status="pending",
            message="Re-indexing not yet implemented. This is a placeholder.",
            chapters_processed=0,
            timestamp=datetime.utcnow(),
        )

    except Exception as e:
        logger.error(f"Error during re-indexing: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/validate", response_model=ValidationResponse)
async def validate(request: ValidationRequest) -> ValidationResponse:
    """
    Admin Endpoint: Run RAG Accuracy Validation Suite

    Tests RAG chatbot accuracy by running 18+ predefined queries
    (minimum 3 per chapter, 6 chapters = 18 queries minimum).

    Target: ≥90% accuracy (all responses sourced from book, no hallucinations)

    Validation Process:
    1. Load test query set from fixtures/rag_test_queries.json
    2. For each query:
       - Execute chatbot query
       - Check if response sources are from book
       - Compare against expected ground truth
       - Record pass/fail
    3. Report pass rate and detailed results
    4. Block deployment if accuracy <90%

    Returns:
    - Total queries tested
    - Passed queries count
    - Overall accuracy percentage
    - Individual results (verbose mode)
    """

    logger.info(f"Validation request received (min {request.min_queries_per_chapter} per chapter)")

    try:
        # TODO: Implement validation suite
        # 1. Load test queries
        # 2. Execute queries
        # 3. Check accuracy
        # 4. Return results

        # Placeholder response (Phase 2 implementation)
        return ValidationResponse(
            status="pending",
            total_queries=0,
            passed_queries=0,
            accuracy_pct=0.0,
            results=[],
            timestamp=datetime.utcnow(),
        )

    except Exception as e:
        logger.error(f"Error during validation: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

# ============================================================================
# Error Handlers
# ============================================================================

@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """Global exception handler for unhandled errors"""
    logger.error(f"Unhandled exception: {str(exc)}")
    return {
        "error": "Internal server error",
        "detail": str(exc),
        "timestamp": datetime.utcnow(),
    }

# ============================================================================
# Startup & Shutdown Events
# ============================================================================

@app.on_event("startup")
async def startup_event():
    """Initialize services on startup"""
    global _startup_time
    _startup_time = time.time()

    logger.info("FastAPI server starting...")
    logger.info(f"Environment: {os.getenv('ENVIRONMENT', 'development')}")
    logger.info(f"Debug: {os.getenv('DEBUG', 'false')}")

    try:
        # Initialize services (lazy loading will happen on first use)
        logger.info("Services will be initialized on first request (lazy loading)")
        logger.info(f"ChromaDB path: {os.getenv('CHROMADB_PATH', './data/embeddings')}")
        logger.info(f"Embedding model: {os.getenv('EMBEDDING_MODEL', 'sentence-transformers/all-MiniLM-L6-v2')}")
        logger.info(f"API running on port 8000")
        logger.info("Visit http://localhost:8000/docs for API documentation")
    except Exception as e:
        logger.error(f"Startup error: {e}")
        raise

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up on shutdown"""
    logger.info("FastAPI server shutting down...")
    # Services are kept in memory; they clean up their own resources
    logger.info("Shutdown complete")

# ============================================================================
# Entry Point
# ============================================================================

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=os.getenv("ENVIRONMENT") == "development",
        log_level=os.getenv("LOG_LEVEL", "debug").lower(),
    )
