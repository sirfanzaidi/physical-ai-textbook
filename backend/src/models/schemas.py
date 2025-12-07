"""Pydantic data models for Physical AI Textbook RAG system."""

from typing import Optional, List
from datetime import datetime
from pydantic import BaseModel, Field


# ============================================================================
# CHUNK & METADATA (Internal representation for RAG)
# ============================================================================

class ChunkMetadata(BaseModel):
    """Metadata associated with a text chunk from the textbook."""

    chapter_number: int = Field(..., description="Chapter number (1-6)")
    chapter_title: str = Field(..., description="Chapter title (e.g., 'Introduction to Physical AI')")
    section_title: Optional[str] = Field(None, description="Section within chapter (if applicable)")
    chunk_index: int = Field(..., description="Position of chunk within chapter (0-based)")
    page_number: Optional[int] = Field(None, description="Approximate page number in original textbook")
    token_count: int = Field(..., description="Number of tokens in chunk (approx 200-500)")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    embedding_model: str = Field(
        default="sentence-transformers/all-MiniLM-L6-v2",
        description="Embedding model used to vectorize chunk"
    )


class Chunk(BaseModel):
    """A text chunk from the textbook with embedding metadata."""

    chunk_id: str = Field(..., description="Unique ID: ch{chapter}_chunk{index}")
    content: str = Field(..., description="Raw text content (200-500 tokens)")
    metadata: ChunkMetadata
    embedding: Optional[List[float]] = Field(None, description="Vector embedding (384-dims for all-MiniLM)")


# ============================================================================
# CHATBOT QUERY & RESPONSE (API contracts)
# ============================================================================

class Citation(BaseModel):
    """A citation to a source chunk in the textbook."""

    chunk_id: str = Field(..., description="Reference to source chunk")
    chapter_number: int = Field(..., description="Chapter where chunk came from")
    chapter_title: str
    section_title: Optional[str]
    excerpt: str = Field(..., description="Relevant excerpt from chunk (100-200 chars)")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Cosine similarity score (0-1)")


class ChatbotQuery(BaseModel):
    """User query to the RAG chatbot."""

    question: str = Field(..., min_length=5, max_length=500, description="User question about textbook")
    context_window: int = Field(default=3, ge=1, le=10, description="Number of chapters to search (default: all)")
    include_citations: bool = Field(default=True, description="Include source citations in response")


class ChatbotResponse(BaseModel):
    """Response from the RAG chatbot with citations."""

    response_text: str = Field(..., description="Generated answer grounded in textbook content")
    citations: List[Citation] = Field(default_factory=list, description="List of source citations")
    confidence_score: float = Field(ge=0.0, le=1.0, description="Confidence in response accuracy (0-1)")
    source_chunk_ids: List[str] = Field(default_factory=list, description="IDs of chunks used")
    query_embedding_time_ms: int = Field(default=0, description="Time to embed query")
    search_time_ms: int = Field(default=0, description="Time to search vectors")
    generation_time_ms: int = Field(default=0, description="Time to generate response")
    total_time_ms: int = Field(default=0, description="Total response time")


# ============================================================================
# RE-INDEXING (Admin API)
# ============================================================================

class ReindexRequest(BaseModel):
    """Request to re-index chapters into RAG database."""

    chapters: Optional[List[int]] = Field(None, description="Specific chapters to re-index (default: all)")
    mode: str = Field(
        default="auto",
        description="Indexing mode: 'delta' (single chapters), 'full' (all), 'auto' (detect)"
    )
    validate: bool = Field(default=True, description="Run validation after indexing")
    dry_run: bool = Field(default=False, description="Simulate indexing without persisting")


class ReindexResponse(BaseModel):
    """Response from re-indexing operation."""

    status: str = Field(..., description="Status: 'pending', 'in_progress', 'success', 'failed'")
    message: str
    chapters_processed: int
    chunks_created: int = Field(default=0)
    chunks_deleted: int = Field(default=0)
    indexing_time_ms: int = Field(default=0)
    validation_accuracy: Optional[float] = Field(None, ge=0.0, le=1.0)
    error_details: Optional[str] = None


# ============================================================================
# VALIDATION & QA (Admin API)
# ============================================================================

class ValidationResult(BaseModel):
    """Result of a single RAG validation query."""

    query: str
    expected_chapter: int
    expected_answer_keywords: List[str]
    retrieved_chunks: int
    top_chunk_id: Optional[str]
    top_chunk_relevance: float = Field(ge=0.0, le=1.0)
    passed: bool = Field(..., description="Whether validation passed (relevance >= threshold)")
    notes: Optional[str] = None


class ValidationRequest(BaseModel):
    """Request to validate RAG accuracy."""

    queries_per_chapter: int = Field(default=3, ge=1, le=10, description="Validation queries per chapter")
    accuracy_threshold: float = Field(default=0.90, ge=0.0, le=1.0, description="Minimum required accuracy")
    verbose: bool = Field(default=False, description="Include detailed results")


class ValidationResponse(BaseModel):
    """Response from RAG validation suite."""

    status: str = Field(..., description="Status: 'passed', 'failed'")
    total_queries: int
    passed_queries: int
    failed_queries: int
    accuracy_pct: float = Field(ge=0.0, le=100.0, description="Accuracy percentage")
    threshold_pct: float = Field(ge=0.0, le=100.0, description="Required threshold percentage")
    results: List[ValidationResult] = Field(default_factory=list, description="Detailed results (if verbose)")
    validation_time_ms: int = Field(default=0)
    notes: Optional[str] = None


# ============================================================================
# HEALTH & STATUS
# ============================================================================

class HealthResponse(BaseModel):
    """Health check response."""

    status: str = Field(default="ok", description="Status: 'ok', 'degraded', 'error'")
    version: str = Field(default="1.0.0")
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    dependencies: dict = Field(default_factory=dict, description="Status of key dependencies")
    uptime_seconds: Optional[float] = None
    active_collection: Optional[str] = Field(None, description="Currently active ChromaDB collection")
    total_chunks_indexed: int = Field(default=0)
    vector_db_size_mb: float = Field(default=0.0)
