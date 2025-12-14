"""
Pydantic models for request/response validation.

Defines API contracts for all endpoints.
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Literal


# ===== Ingest Endpoint Models =====

class IngestRequest(BaseModel):
    """Request model for book ingestion."""

    file_name: str = Field(..., description="Name of the book file")
    file_content: bytes = Field(..., description="File content (binary)")
    book_id: Optional[str] = Field(None, description="Optional book identifier")

    class Config:
        json_schema_extra = {
            "example": {
                "file_name": "example_book.pdf",
                "book_id": "book_001",
            }
        }


class IngestResponse(BaseModel):
    """Response model for successful ingestion."""

    success: bool = Field(..., description="Whether ingestion succeeded")
    book_id: str = Field(..., description="Book identifier")
    chunk_count: int = Field(..., description="Number of chunks created")
    total_tokens: int = Field(..., description="Total tokens in all chunks")
    message: str = Field(..., description="Status message")

    class Config:
        json_schema_extra = {
            "example": {
                "success": True,
                "book_id": "book_001",
                "chunk_count": 125,
                "total_tokens": 45000,
                "message": "Book indexed successfully",
            }
        }


# ===== Chat Endpoint Models =====

class ChatRequest(BaseModel):
    """Request model for chat endpoint."""

    query: str = Field(..., min_length=1, max_length=5000, description="User query")
    book_id: str = Field(..., description="Book to query")
    mode: Literal["full", "selected"] = Field("full", description="Query mode: full book or selected text only")
    selected_text: Optional[str] = Field(None, min_length=10, description="Selected text passage for constrained query")

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What is the main theme of chapter 2?",
                "book_id": "book_001",
                "mode": "full",
            }
        }


class Citation(BaseModel):
    """Citation information for a response."""

    page_num: Optional[int] = Field(None, description="Page number")
    section_name: Optional[str] = Field(None, description="Section name")
    chapter_name: Optional[str] = Field(None, description="Chapter name")
    text_snippet: Optional[str] = Field(None, description="Source text snippet")


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""

    response: str = Field(..., description="Generated response from chatbot")
    citations: List[Citation] = Field(default_factory=list, description="Source citations")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence score (0-1)")
    latency_ms: float = Field(..., description="Response generation latency in milliseconds")

    class Config:
        json_schema_extra = {
            "example": {
                "response": "The main theme of chapter 2 is...",
                "citations": [
                    {
                        "page_num": 15,
                        "section_name": "Chapter 2",
                        "chapter_name": "Basics of Humanoid Robotics",
                        "text_snippet": "The fundamental concepts...",
                    }
                ],
                "confidence": 0.92,
                "latency_ms": 1250.5,
            }
        }


# ===== Health Endpoint Models =====

class HealthResponse(BaseModel):
    """Response model for health check endpoint."""

    status: str = Field(..., description="Service status")
    environment: str = Field(..., description="Running environment")
    services: Optional[dict] = Field(None, description="Status of external services")

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "environment": "development",
                "services": {
                    "cohere": "connected",
                    "qdrant": "connected",
                    "neon": "connected",
                }
            }
        }


# ===== Error Models =====

class ErrorResponse(BaseModel):
    """Standard error response model."""

    error: str = Field(..., description="Error type or message")
    detail: Optional[str] = Field(None, description="Detailed error description")
    status_code: int = Field(..., description="HTTP status code")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "ValidationError",
                "detail": "Query exceeds maximum length of 5000 characters",
                "status_code": 422,
            }
        }
