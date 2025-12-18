"""FastAPI application."""

import logging
import sys
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .config import Settings
from .routes import health, chat, ingest, auth, users
from .routes.chat import initialize_services
from .routes.ingest import initialize_ingestion_service
from .services.user_service import UserService
import psycopg2
from psycopg2 import pool

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
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)

# Database connection pool (global for auth services)
db_pool = None

# Include routers
logger.info("Registering routes...")
app.include_router(health.router)
app.include_router(chat.router)
app.include_router(ingest.router)
app.include_router(auth.router)
app.include_router(users.router)


@app.on_event("startup")
async def startup_event():
    """Startup event handler."""
    global db_pool

    logger.info("=" * 70)
    logger.info("Starting Physical AI Textbook API with RAG & Authentication")
    logger.info("=" * 70)
    logger.info("API documentation available at /api/docs")

    try:
        # Initialize authentication services
        logger.info("Initializing authentication services...")

        # Initialize database connection pool for auth services
        if settings.database_url:
            try:
                db_pool = psycopg2.pool.SimpleConnectionPool(1, 10, settings.database_url)
                logger.info("Database connection pool initialized")

                # Get a test connection to verify database is accessible
                test_conn = db_pool.getconn()
                try:
                    with test_conn.cursor() as cur:
                        cur.execute("SELECT 1")
                    logger.info("Database connection verified")
                finally:
                    db_pool.putconn(test_conn)

                # Initialize user service with database pool
                # (UserService will get fresh connections from the pool as needed)
                user_service = UserService(db_pool)
                auth.set_services(user_service, settings)
                users.set_services(user_service, settings)

            except Exception as e:
                logger.warning(f"Database initialization failed: {e}")
                logger.warning("Authentication will be disabled until database is available")
        else:
            logger.warning("DATABASE_URL not set - authentication disabled")

        # Initialize RAG services (chat + retrieval + generation)
        await initialize_services(settings)
        logger.info("RAG services initialized successfully")

        # Import here to get the initialized services
        from .routes.chat import _openrouter_client, _qdrant_store

        # Initialize ingestion service
        await initialize_ingestion_service(
            openrouter_client=_openrouter_client,
            qdrant_store=_qdrant_store,
            settings=settings,
        )
        logger.info("Ingestion service initialized successfully")

    except Exception as e:
        logger.error(f"Failed to initialize services: {e}")
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
