"""Structured logging utilities."""

import logging
import json
import time
from typing import Optional, Any
from datetime import datetime


class JSONFormatter(logging.Formatter):
    """JSON log formatter for structured logging."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON.

        Args:
            record: Log record

        Returns:
            JSON-formatted log line
        """
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }

        if record.exc_info:
            log_data["exception"] = self.formatException(record.exc_info)

        # Add custom attributes if present
        if hasattr(record, "request_id"):
            log_data["request_id"] = record.request_id
        if hasattr(record, "latency_ms"):
            log_data["latency_ms"] = record.latency_ms
        if hasattr(record, "user_id"):
            log_data["user_id"] = record.user_id

        return json.dumps(log_data)


def get_logger(name: str, level: str = "INFO") -> logging.Logger:
    """Get configured logger instance.

    Args:
        name: Logger name (usually __name__)
        level: Log level

    Returns:
        Configured logger
    """
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, level, logging.INFO))

    # Only add handler if logger doesn't have one
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = JSONFormatter()
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger


class TimingContext:
    """Context manager for measuring operation timing."""

    def __init__(self, logger: logging.Logger, operation: str):
        """Initialize timing context.

        Args:
            logger: Logger instance
            operation: Operation name for logging
        """
        self.logger = logger
        self.operation = operation
        self.start_time = None
        self.elapsed_ms = None

    def __enter__(self) -> "TimingContext":
        """Enter context."""
        self.start_time = time.time()
        self.logger.debug(f"Starting: {self.operation}")
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """Exit context and log timing."""
        self.elapsed_ms = (time.time() - self.start_time) * 1000

        if exc_type is None:
            self.logger.info(
                f"Completed: {self.operation}",
                extra={"latency_ms": self.elapsed_ms}
            )
        else:
            self.logger.error(
                f"Failed: {self.operation} - {exc_val}",
                extra={"latency_ms": self.elapsed_ms},
                exc_info=True
            )
