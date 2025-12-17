"""Pydantic models for API request/response schemas."""

from pydantic import BaseModel, Field


class HealthResponse(BaseModel):
    """Health check response."""

    status: str = Field(..., description="Overall health status: healthy, degraded, or unhealthy")
    qdrant_connected: bool = Field(..., description="Is Qdrant connected?")
    collection_exists: bool = Field(..., description="Does the collection exist?")
    collection_count: int = Field(..., description="Number of points in collection")
    cohere_available: bool = Field(..., description="Is Cohere API available?")
