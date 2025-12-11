"""FastAPI application for RAG chatbot."""

import logging
import sys
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from api.config import Settings
from api.routes import chat, health

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    stream=sys.stdout,
)

logger = logging.getLogger(__name__)

# Load settings
settings = Settings()

# Create FastAPI app
app = FastAPI(
    title="Physical AI Textbook RAG Chatbot API",
    description="Semantic search and Q&A for Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
)

# Add CORS middleware
logger.info(f"Configuring CORS for origins: {settings.cors_origins}")
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)

# Include routers
logger.info("Registering routes...")
app.include_router(chat.router)
app.include_router(health.router)


@app.on_event("startup")
async def startup_event():
    """Startup event handler."""
    logger.info("=" * 70)
    logger.info("Starting Physical AI Textbook RAG Chatbot API")
    logger.info("=" * 70)
    logger.info(f"Collection name: {settings.collection_name}")
    logger.info(f"Retrieval limit: {settings.retrieval_limit}")
    logger.info(f"OpenAI model: {settings.openai_model}")
    logger.info(f"Max tokens: {settings.max_tokens}")
    logger.info("API documentation available at /api/docs")
    logger.info("=" * 70)


@app.on_event("shutdown")
async def shutdown_event():
    """Shutdown event handler."""
    logger.info("Shutting down API...")


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI Textbook RAG Chatbot API",
        "docs": "/api/docs",
        "health": "/api/health",
        "chat": "POST /api/chat",
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
