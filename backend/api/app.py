"""FastAPI application."""

import logging
import sys
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from api.config import Settings
from api.routes import health, chat
from api.routes.chat import initialize_services

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
    title="Physical AI Textbook API",
    description="API for Physical AI & Humanoid Robotics textbook with RAG",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
)

# Add CORS middleware - parse comma-separated origins
cors_origins_list = [origin.strip() for origin in settings.cors_origins.split(",")]
logger.info(f"Configuring CORS for origins: {cors_origins_list}")
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["*"],
)

# Include routers
logger.info("Registering routes...")
app.include_router(health.router)
app.include_router(chat.router)


@app.on_event("startup")
async def startup_event():
    """Startup event handler."""
    logger.info("=" * 70)
    logger.info("Starting Physical AI Textbook API with RAG")
    logger.info("=" * 70)
    logger.info("API documentation available at /api/docs")

    try:
        # Initialize RAG services
        await initialize_services(settings)
        logger.info("RAG services initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize RAG services: {e}")
        logger.warning("API will run with limited functionality")

    logger.info("=" * 70)


@app.on_event("shutdown")
async def shutdown_event():
    """Shutdown event handler."""
    logger.info("Shutting down API...")


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI Textbook API",
        "docs": "/api/docs",
        "health": "/api/health",
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
