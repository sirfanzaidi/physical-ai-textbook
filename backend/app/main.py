"""
FastAPI application for RAG Chatbot backend.

Main entry point for the chatbot API with middleware configuration,
CORS settings, and route registration.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging
import structlog

from .config import get_settings

# Configure logging
logging.basicConfig(level=logging.INFO)
structlog.configure(
    processors=[
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.stdlib.PositionalArgumentsFormatter(),
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.processors.UnicodeDecoder(),
        structlog.processors.JSONRenderer(),
    ],
    context_class=dict,
    logger_factory=structlog.stdlib.LoggerFactory(),
    cache_logger_on_first_use=True,
)

logger = structlog.get_logger()

# Initialize settings
settings = get_settings()

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for published books",
    version="0.1.0",
    debug=settings.debug,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins,
    allow_credentials=settings.cors_allow_credentials,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def startup_event():
    """Initialize app on startup."""
    logger.info(
        "startup_event",
        environment=settings.environment,
        debug=settings.debug,
    )


@app.on_event("shutdown")
async def shutdown_event():
    """Clean up on shutdown."""
    logger.info("shutdown_event")


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "environment": settings.environment,
    }


# Include API routes
from .api import routes

app.include_router(routes.router)

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level=settings.log_level.lower(),
    )
