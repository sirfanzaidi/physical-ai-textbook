"""
Structured logging configuration for RAG Chatbot.

Sets up structlog with JSON output for machine-parseable logs.
Integrates with FastAPI for request/response logging with timing.
"""

import structlog
import logging
import json
from datetime import datetime
from typing import Any, Dict
import sys


def setup_logging(log_level: str = "INFO", log_dir: str = "./logs") -> None:
    """
    Configure structured logging with JSON output.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_dir: Directory to store log files
    """
    # Configure structlog
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

    # Configure Python logging to use structlog
    logging.basicConfig(
        format="%(message)s",
        stream=sys.stdout,
        level=getattr(logging, log_level.upper()),
    )


def get_logger(name: str = None):
    """
    Get a structured logger instance.

    Args:
        name: Logger name (usually __name__)

    Returns:
        structlog logger instance
    """
    return structlog.get_logger(name or "rag_chatbot")


class LogContext:
    """Context manager for structured logging with request-scoped data."""

    def __init__(self, request_id: str = None, **context):
        """
        Initialize logging context.

        Args:
            request_id: Unique request identifier
            **context: Additional context fields
        """
        self.request_id = request_id
        self.context = context
        self.logger = get_logger()

    def __enter__(self):
        """Enter context and bind request data to logger."""
        context_dict = {"request_id": self.request_id}
        context_dict.update(self.context)
        structlog.contextvars.clear_contextvars()
        for key, value in context_dict.items():
            structlog.contextvars.bind_contextvars(**{key: value})
        return self.logger

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exit context and clear request data."""
        structlog.contextvars.clear_contextvars()
        return False


def log_query(
    logger,
    query: str,
    book_id: str,
    mode: str,
    request_id: str = None,
) -> None:
    """
    Log a user query for audit trail.

    Args:
        logger: structlog logger instance
        query: User query text
        book_id: Book being queried
        mode: Query mode ("full" or "selected")
        request_id: Request ID for correlation
    """
    logger.info(
        "query_received",
        query_length=len(query),
        book_id=book_id,
        mode=mode,
        request_id=request_id,
    )


def log_retrieval_metrics(
    logger,
    book_id: str,
    chunks_retrieved: int,
    chunks_reranked: int,
    retrieval_latency_ms: float,
    rerank_latency_ms: float,
) -> None:
    """
    Log retrieval pipeline metrics.

    Args:
        logger: structlog logger instance
        book_id: Book queried
        chunks_retrieved: Number of initial chunks retrieved
        chunks_reranked: Number of chunks after reranking
        retrieval_latency_ms: Retrieval operation latency
        rerank_latency_ms: Reranking operation latency
    """
    logger.info(
        "retrieval_metrics",
        book_id=book_id,
        chunks_retrieved=chunks_retrieved,
        chunks_reranked=chunks_reranked,
        retrieval_latency_ms=round(retrieval_latency_ms, 2),
        rerank_latency_ms=round(rerank_latency_ms, 2),
    )


def log_generation_metrics(
    logger,
    response_length: int,
    confidence: float,
    generation_latency_ms: float,
    citations_count: int,
) -> None:
    """
    Log generation pipeline metrics.

    Args:
        logger: structlog logger instance
        response_length: Length of generated response
        confidence: Confidence score (0-1)
        generation_latency_ms: Generation operation latency
        citations_count: Number of citations in response
    """
    logger.info(
        "generation_metrics",
        response_length=response_length,
        confidence=round(confidence, 3),
        generation_latency_ms=round(generation_latency_ms, 2),
        citations_count=citations_count,
    )


def log_end_to_end_metrics(
    logger,
    total_latency_ms: float,
    retrieval_latency_ms: float,
    generation_latency_ms: float,
    confidence: float,
) -> None:
    """
    Log end-to-end latency for a query.

    Args:
        logger: structlog logger instance
        total_latency_ms: Total query processing time
        retrieval_latency_ms: Retrieval stage latency
        generation_latency_ms: Generation stage latency
        confidence: Overall confidence score
    """
    logger.info(
        "query_completed",
        total_latency_ms=round(total_latency_ms, 2),
        retrieval_latency_ms=round(retrieval_latency_ms, 2),
        generation_latency_ms=round(generation_latency_ms, 2),
        confidence=round(confidence, 3),
    )


def log_ingestion_metrics(
    logger,
    book_id: str,
    file_name: str,
    total_pages: int,
    chunks_created: int,
    total_tokens: int,
    ingestion_latency_ms: float,
) -> None:
    """
    Log book ingestion metrics.

    Args:
        logger: structlog logger instance
        book_id: Book identifier
        file_name: Source file name
        total_pages: Total pages in book
        chunks_created: Number of chunks created
        total_tokens: Total tokens across all chunks
        ingestion_latency_ms: Ingestion operation latency
    """
    logger.info(
        "ingestion_completed",
        book_id=book_id,
        file_name=file_name,
        total_pages=total_pages,
        chunks_created=chunks_created,
        total_tokens=total_tokens,
        ingestion_latency_ms=round(ingestion_latency_ms, 2),
    )
