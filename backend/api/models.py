"""Pydantic models for API request/response schemas."""

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class Citation(BaseModel):
    """Citation metadata for retrieved content."""

    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    chapter_name: str = Field(..., description="Chapter or section name")
    section_name: Optional[str] = Field(None, description="Subsection name if applicable")
    source_url: str = Field(..., description="URL to the source page in documentation")
    page_num: Optional[int] = Field(None, description="Page number if applicable")
    content_preview: Optional[str] = Field(None, description="First 200 chars of cited content")


class ChatRequest(BaseModel):
    """Request model for chat-stream endpoint."""

    query: str = Field(..., min_length=1, max_length=5000, description="User query")
    book_id: str = Field(default="physical-ai", description="Book identifier")
    mode: str = Field(
        default="full",
        pattern="^(full|selected)$",
        description="Query mode: 'full' for entire book or 'selected' for highlighted text only"
    )
    selected_text: Optional[str] = Field(
        None,
        min_length=10,
        description="Highlighted text from page (only used if mode='selected')"
    )
    top_k: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of chunks to retrieve from vector database"
    )
    conversation_id: Optional[str] = Field(
        None,
        description="Optional conversation ID for context (future feature for multi-turn)"
    )
    language: Optional[str] = Field(
        'en',
        pattern="^(en|ur)$",
        description="Language preference for the response"
    )


class ChatResponseChunk(BaseModel):
    """Streaming chunk in chat response."""

    type: str = Field(description="Type of chunk: 'text', 'citation', 'metadata'")
    content: str = Field(description="Content of the chunk")
    index: int = Field(description="Index in the stream")


class ChatResponse(BaseModel):
    """Response model for chat-stream endpoint (for non-streaming use)."""

    response_text: str = Field(..., description="Full response text from LLM")
    citations: List[Citation] = Field(default_factory=list, description="List of citations")
    sources: List[str] = Field(default_factory=list, description="List of source URLs")
    latency_ms: float = Field(..., description="Request latency in milliseconds")
    model: str = Field(..., description="Model used for generation")
    usage: Optional[dict] = Field(None, description="Token usage information from OpenRouter")


class IngestRequest(BaseModel):
    """Request model for ingestion endpoint."""

    book_id: str = Field(default="physical-ai", description="Book identifier")
    source: str = Field(..., description="Source of content: 'docusaurus', 'markdown', 'html'")
    docs_path: Optional[str] = Field(
        None,
        description="Path to documentation files or URL to crawl"
    )
    chunk_size: int = Field(default=1000, ge=100, le=5000, description="Size of text chunks")
    chunk_overlap: int = Field(default=200, ge=0, le=1000, description="Overlap between chunks")


class IngestResponse(BaseModel):
    """Response model for ingestion endpoint."""

    status: str = Field(..., description="Status: 'success', 'partial', 'failed'")
    chunks_processed: int = Field(..., description="Number of chunks processed")
    chunks_stored: int = Field(..., description="Number of chunks successfully stored in Qdrant")
    duration_seconds: float = Field(..., description="Total processing time")
    message: str = Field(..., description="Detailed status message")
    errors: List[str] = Field(default_factory=list, description="List of any errors encountered")


class HealthResponse(BaseModel):
    """Health check response."""

    status: str = Field(..., description="Overall health status")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Health check timestamp")
    qdrant_connected: bool = Field(default=False)
    collection_exists: bool = Field(default=False)
    collection_count: int = Field(default=0)
    cohere_available: bool = Field(default=False)
    message: str = Field(default="", description="Additional status message")


class ErrorResponse(BaseModel):
    """Error response model."""

    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Error message")
    status_code: int = Field(..., description="HTTP status code")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Error timestamp")
    request_id: Optional[str] = Field(None, description="Request ID for tracking")
