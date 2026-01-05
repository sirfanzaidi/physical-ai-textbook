"""FastAPI application."""

import logging
import sys
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .config import Settings
from .routes import health, chat, ingest, auth
from .routes.chat import initialize_services
from .routes.ingest import initialize_ingestion_service
from .services.user_service import UserService
import psycopg2.pool

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
    description="API for Physical AI & Humanoid Robotics textbook with RAG (Authentication Removed)",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
    openapi_url="/api/openapi.json",
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

# Include routers
logger.info("Registering routes...")
app.include_router(health.router)
app.include_router(chat.router)
app.include_router(ingest.router)

# Debug: Log auth router before including it
logger.info(f"Auth router has {len(auth.router.routes)} routes before including")
for route in auth.router.routes:
    logger.info(f"  - Route path: {route.path}, methods: {getattr(route, 'methods', 'N/A')}")

app.include_router(auth.router)
logger.info(f"Auth router included")

logger.info("Route registration complete")


@app.on_event("startup")
async def startup_event():
    """Startup event handler."""

    logger.info("=" * 70)
    logger.info("Starting Physical AI Textbook API with RAG and Authentication")
    logger.info("=" * 70)
    logger.info("API documentation available at /api/docs")

    # Debug: Log all registered routes
    all_paths = [r.path for r in app.routes if hasattr(r, 'path')]
    auth_paths = [p for p in all_paths if '/auth' in p]
    logger.info(f"Total routes registered: {len(all_paths)}")
    logger.info(f"Auth routes registered: {len(auth_paths)}")
    if auth_paths:
        for path in auth_paths:
            logger.info(f"  Auth route: {path}")
    else:
        logger.warning("NO AUTH ROUTES FOUND!")

    try:
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

        # Initialize authentication services
        if settings.database_url:
            try:
                # Create database connection pool
                db_pool = psycopg2.pool.SimpleConnectionPool(
                    minconn=1,
                    maxconn=10,
                    dsn=settings.database_url
                )
                user_service = UserService(db_pool)
                auth.set_services(user_service, settings)
                logger.info("Authentication services initialized successfully")
            except Exception as db_error:
                logger.error(f"Failed to initialize database pool: {db_error}")
                logger.warning("Authentication features will be unavailable")
        else:
            logger.warning("DATABASE_URL not configured - authentication features will be unavailable")

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


@app.get("/debug/routes")
async def debug_routes():
    """Debug endpoint to show all registered routes."""
    routes_info = []
    for route in app.routes:
        if hasattr(route, 'path'):
            routes_info.append({
                "path": route.path,
                "methods": list(route.methods) if hasattr(route, 'methods') else None,
                "name": route.name if hasattr(route, 'name') else None
            })

    auth_routes = [r for r in routes_info if '/auth' in r['path']]

    return {
        "total_routes": len(routes_info),
        "auth_routes_count": len(auth_routes),
        "auth_routes": auth_routes,
        "all_routes": routes_info
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)

