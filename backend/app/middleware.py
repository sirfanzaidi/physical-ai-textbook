"""
Middleware for FastAPI application.

Includes error handling, CORS, request logging, and custom middleware.
"""

from fastapi import Request, Response
from fastapi.responses import JSONResponse
import time
import structlog
import traceback
from typing import Callable


logger = structlog.get_logger()


class LoggingMiddleware:
    """Middleware to log HTTP requests and responses."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, request: Request, call_next: Callable) -> Response:
        """Log request and response with timing."""
        request_id = request.headers.get("X-Request-ID", "")
        start_time = time.time()

        try:
            response = await call_next(request)
            process_time = time.time() - start_time

            logger.info(
                "request_completed",
                method=request.method,
                path=request.url.path,
                status_code=response.status_code,
                process_time_ms=process_time * 1000,
                request_id=request_id,
            )

            response.headers["X-Process-Time"] = str(process_time)
            return response

        except Exception as e:
            process_time = time.time() - start_time
            logger.error(
                "request_failed",
                method=request.method,
                path=request.url.path,
                error=str(e),
                process_time_ms=process_time * 1000,
                request_id=request_id,
                traceback=traceback.format_exc(),
            )
            raise


async def exception_handler(request: Request, exc: Exception):
    """Global exception handler for API errors."""
    logger.error(
        "unhandled_exception",
        method=request.method,
        path=request.url.path,
        error=str(exc),
        traceback=traceback.format_exc(),
    )

    return JSONResponse(
        status_code=500,
        content={
            "detail": "Internal server error",
            "error": str(exc),
        },
    )
