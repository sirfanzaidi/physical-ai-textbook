"""Pydantic models for API request/response schemas."""

from pydantic import BaseModel, Field
from typing import Optional


class ChatQuery(BaseModel):
    """User chat query request."""

    question: str = Field(..., min_length=3, max_length=500, description="Question to ask about the textbook")
    selected_text: Optional[str] = Field(None, max_length=1000, description="Optional selected text for context")
    chapter_filter: Optional[int] = Field(None, ge=1, le=6, description="Optional chapter number to filter results")


class RetrievedChunk(BaseModel):
    """Retrieved chunk with full metadata for context."""

    chunk_id: str
    chapter_num: int
    chapter_title: str
    chapter_url: str
    text: str  # Full text for context assembly
    similarity_score: float


class SourceCitation(BaseModel):
    """Source citation for answer."""

    chapter_num: int
    chapter_title: str
    chapter_url: str
    snippet: str


class ChatResponse(BaseModel):
    """Chat response with answer and sources."""

    answer: str = Field(..., description="Generated answer from the chatbot")
    sources: list[SourceCitation] = Field(default_factory=list, description="Source citations")
    retrieved_chunks_count: int = Field(..., description="Number of chunks retrieved for context")


class HealthResponse(BaseModel):
    """Health check response."""

    status: str = Field(..., description="Overall health status: healthy, degraded, or unhealthy")
    qdrant_connected: bool = Field(..., description="Is Qdrant connected?")
    collection_exists: bool = Field(..., description="Does the collection exist?")
    collection_count: int = Field(..., description="Number of points in collection")
    cohere_available: bool = Field(..., description="Is Cohere API available?")
