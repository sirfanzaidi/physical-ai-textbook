"""Dependency injection for FastAPI endpoints."""

import logging
from functools import lru_cache
from api.config import Settings

logger = logging.getLogger(__name__)


@lru_cache()
def get_settings() -> Settings:
    """Get application settings (cached)."""
    logger.info("Loading settings from environment")
    return Settings()
