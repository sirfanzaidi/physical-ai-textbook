#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook - RAG Embedding Pipeline
=================================================================

Production-ready embedding pipeline implementing 8 phases:
1. Backend setup, Cohere & Qdrant client initialization
2. Fetch 6 Docusaurus chapters from the textbook website
3. Extract clean text from HTML (strip tags, navigation)
4. Chunk text semantically (800-1200 chars, 150-200 char overlap, sentence boundaries)
5. Batch-generate Cohere embeddings (1024-dim vectors)
6. Upsert to Qdrant collection "book_chunks" with metadata
7. Verify semantic search with test queries
8. Generate console report with summary

Tech Stack:
- Python 3.11+ with async/await
- Cohere API for embeddings (embed-english-v3.0, 1024 dimensions)
- Qdrant cloud for vector storage
- Pydantic for type safety
- aiohttp for async HTTP

Author: Physical AI Textbook Team
Version: 3.0.0
"""

from __future__ import annotations

import asyncio
import logging
import os
import re
import sys
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from html.parser import HTMLParser
from typing import Any, Optional

import aiohttp
import cohere
from dotenv import load_dotenv
from pydantic import BaseModel, Field, field_validator
from qdrant_client import QdrantClient
from qdrant_client.http import models as qdrant_models
from qdrant_client.http.exceptions import UnexpectedResponse


# =============================================================================
# Configuration & Environment Setup
# =============================================================================

# Load environment variables from .env file
load_dotenv()

# Configure structured logging
logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO"),
    format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("embedding_pipeline")


class Config:
    """
    Centralized configuration loaded from environment variables.
    All sensitive values are loaded from env vars; never hardcoded.
    """

    # API Keys (required)
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")

    # Textbook Configuration
    TEXTBOOK_BASE_URL: str = os.getenv(
        "TEXTBOOK_BASE_URL", "https://physical-ai-textbook-two.vercel.app"
    )

    # Qdrant Collection Configuration
    COLLECTION_NAME: str = "book_chunks"
    EMBEDDING_DIMENSION: int = 1024  # Cohere embed-english-v3.0 dimension

    # Chunking Configuration
    MIN_CHUNK_SIZE: int = 800
    MAX_CHUNK_SIZE: int = 1200
    CHUNK_OVERLAP: int = 175  # 150-200 char overlap (midpoint)

    # Batch Processing Configuration
    EMBEDDING_BATCH_SIZE: int = 50  # Stay under Cohere rate limits
    UPSERT_BATCH_SIZE: int = 100

    # Hardcoded fallback chapter URLs (relative paths)
    CHAPTER_URLS: list[str] = [
        "/docs/introduction",
        "/docs/humanoid-robotics",
        "/docs/humanoid-robot-modeling",
        "/docs/digital-twin",
        "/docs/nvidia-isaac-sim",
        "/docs/capstone",
    ]

    # Chapter metadata mapping
    CHAPTER_METADATA: dict[str, dict[str, Any]] = {
        "/docs/introduction": {"chapter_num": 1, "chapter_title": "Introduction to Physical AI"},
        "/docs/humanoid-robotics": {
            "chapter_num": 2,
            "chapter_title": "Humanoid Robotics",
        },
        "/docs/humanoid-robot-modeling": {
            "chapter_num": 3,
            "chapter_title": "Humanoid Robot Modeling",
        },
        "/docs/digital-twin": {
            "chapter_num": 4,
            "chapter_title": "Digital Twin Simulation",
        },
        "/docs/nvidia-isaac-sim": {
            "chapter_num": 5,
            "chapter_title": "NVIDIA Isaac Sim",
        },
        "/docs/capstone": {"chapter_num": 6, "chapter_title": "Capstone Project"},
    }

    # Test queries for semantic search verification
    TEST_QUERIES: list[str] = [
        "What is physical AI and how does it differ from traditional AI?",
        "Explain the key components of a humanoid robot",
        "How do ROS 2 nodes communicate with each other?",
        "What is a digital twin and why is it useful in robotics?",
        "Describe vision-language-action models in robotics",
    ]

    @classmethod
    def validate(cls) -> list[str]:
        """Validate required configuration. Returns list of missing vars."""
        missing = []
        if not cls.COHERE_API_KEY:
            missing.append("COHERE_API_KEY")
        if not cls.QDRANT_URL:
            missing.append("QDRANT_URL")
        if not cls.QDRANT_API_KEY:
            missing.append("QDRANT_API_KEY")
        return missing


# =============================================================================
# Pydantic Models for Type Safety
# =============================================================================


class ChunkStatus(str, Enum):
    """Status of a text chunk during processing."""

    PENDING = "pending"
    EMBEDDED = "embedded"
    UPSERTED = "upserted"
    FAILED = "failed"


class TextChunk(BaseModel):
    """
    Represents a semantic chunk of text from a chapter.
    Stores both the text and its metadata for Qdrant storage.
    """

    chunk_id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    chapter_num: int = Field(..., ge=1, le=6, description="Chapter number (1-6)")
    chapter_title: str = Field(..., min_length=1, description="Chapter title")
    chapter_url: str = Field(..., description="Full URL to the chapter")
    text: str = Field(..., min_length=10, description="Chunk text content")
    text_snippet: str = Field(default="", description="First 200 chars for preview")
    char_count: int = Field(default=0, description="Character count of chunk")
    chunk_index: int = Field(default=0, description="Index within the chapter")
    status: ChunkStatus = Field(default=ChunkStatus.PENDING)
    embedding: list[float] = Field(default_factory=list)
    created_at: str = Field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )

    @field_validator("text_snippet", mode="before")
    @classmethod
    def generate_snippet(cls, v: str, info) -> str:
        """Auto-generate text snippet from text field."""
        if v:
            return v
        text = info.data.get("text", "")
        return text[:200].strip() + "..." if len(text) > 200 else text

    @field_validator("char_count", mode="before")
    @classmethod
    def calculate_char_count(cls, v: int, info) -> int:
        """Auto-calculate character count from text field."""
        if v > 0:
            return v
        text = info.data.get("text", "")
        return len(text)

    def to_qdrant_payload(self) -> dict[str, Any]:
        """Convert to Qdrant-compatible payload (excludes embedding)."""
        return {
            "chunk_id": self.chunk_id,
            "chapter_num": self.chapter_num,
            "chapter_title": self.chapter_title,
            "chapter_url": self.chapter_url,
            "text": self.text,
            "text_snippet": self.text_snippet,
            "char_count": self.char_count,
            "chunk_index": self.chunk_index,
            "created_at": self.created_at,
        }


class SearchResult(BaseModel):
    """Represents a semantic search result from Qdrant."""

    chunk_id: str
    chapter_num: int
    chapter_title: str
    chapter_url: str
    text_snippet: str
    similarity_score: float = Field(..., ge=0.0, le=1.0)


class PipelineReport(BaseModel):
    """Final report summarizing the embedding pipeline run."""

    start_time: str
    end_time: str
    elapsed_seconds: float
    chapters_processed: int
    total_chunks: int
    chunks_embedded: int
    chunks_upserted: int
    chunks_failed: int
    collection_name: str
    test_queries_run: int
    success: bool
    errors: list[str] = Field(default_factory=list)


# =============================================================================
# HTML Text Extractor
# =============================================================================


class DocusaurusTextExtractor(HTMLParser):
    """
    Custom HTML parser that extracts clean text from Docusaurus pages.
    Strips navigation, sidebars, footers, and other non-content elements.
    """

    # Tags to completely ignore (including their content)
    IGNORED_TAGS: set[str] = {
        "script",
        "style",
        "nav",
        "header",
        "footer",
        "aside",
        "noscript",
        "svg",
        "button",
        "form",
        "input",
        "select",
        "textarea",
    }

    # Class patterns that indicate navigation/non-content areas
    IGNORED_CLASS_PATTERNS: list[str] = [
        "navbar",
        "sidebar",
        "footer",
        "menu",
        "breadcrumb",
        "pagination",
        "toc",
        "table-of-contents",
        "edit-this-page",
        "theme-toggle",
        "navbar-",
    ]

    def __init__(self) -> None:
        super().__init__()
        self.text_parts: list[str] = []
        self.ignore_depth: int = 0
        self.current_tag: str = ""
        self.in_main_content: bool = False

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        """Handle opening tags, tracking ignored sections."""
        self.current_tag = tag.lower()

        # Check if we should ignore this tag
        if self.current_tag in self.IGNORED_TAGS:
            self.ignore_depth += 1
            return

        # Check class attributes for navigation patterns
        attrs_dict = dict(attrs)
        class_attr = attrs_dict.get("class", "") or ""
        id_attr = attrs_dict.get("id", "") or ""

        for pattern in self.IGNORED_CLASS_PATTERNS:
            if pattern in class_attr.lower() or pattern in id_attr.lower():
                self.ignore_depth += 1
                return

        # Track main content area (article or main tag)
        if self.current_tag in ("article", "main"):
            self.in_main_content = True

    def handle_endtag(self, tag: str) -> None:
        """Handle closing tags, updating ignore depth."""
        tag_lower = tag.lower()

        if tag_lower in self.IGNORED_TAGS:
            self.ignore_depth = max(0, self.ignore_depth - 1)
            return

        if tag_lower in ("article", "main"):
            self.in_main_content = False

    def handle_data(self, data: str) -> None:
        """Extract text data, filtering out ignored sections."""
        if self.ignore_depth > 0:
            return

        cleaned = data.strip()
        if cleaned:
            # Add paragraph breaks after certain block elements
            if self.current_tag in ("p", "div", "h1", "h2", "h3", "h4", "h5", "h6", "li"):
                self.text_parts.append(cleaned + "\n")
            else:
                self.text_parts.append(cleaned + " ")

    def get_text(self) -> str:
        """Return the extracted text, cleaned and normalized."""
        raw_text = "".join(self.text_parts)

        # Normalize whitespace
        text = re.sub(r"\s+", " ", raw_text)

        # Preserve paragraph structure
        text = re.sub(r"\n\s*\n", "\n\n", text)

        # Remove common Docusaurus artifacts
        artifacts = [
            "Skip to main content",
            "Edit this page",
            "Last updated",
            "Previous",
            "Next",
            "On this page",
            "Copyright",
        ]
        for artifact in artifacts:
            text = text.replace(artifact, "")

        return text.strip()


def extract_text_from_html(html_content: str) -> str:
    """
    Extract clean text from HTML content using the custom parser.

    Args:
        html_content: Raw HTML string from a Docusaurus page

    Returns:
        Cleaned text suitable for embedding
    """
    parser = DocusaurusTextExtractor()
    try:
        parser.feed(html_content)
        return parser.get_text()
    except Exception as e:
        logger.warning(f"HTML parsing error: {e}")
        # Fallback: basic regex-based extraction
        text = re.sub(r"<[^>]+>", " ", html_content)
        text = re.sub(r"\s+", " ", text)
        return text.strip()


# =============================================================================
# Semantic Text Chunking
# =============================================================================


def find_sentence_boundary(text: str, position: int, search_range: int = 50) -> int:
    """
    Find the nearest sentence boundary to a given position.
    Searches within search_range characters in both directions.

    Args:
        text: The full text to search
        position: Target position to find boundary near
        search_range: How far to search in each direction

    Returns:
        Position of the nearest sentence boundary
    """
    if position >= len(text):
        return len(text)

    # Sentence-ending punctuation patterns
    sentence_enders = ".!?"

    # Search backwards for sentence boundary
    start_search = max(0, position - search_range)
    end_search = min(len(text), position + search_range)

    best_boundary = position
    min_distance = search_range + 1

    for i in range(start_search, end_search):
        if i < len(text) - 1 and text[i] in sentence_enders and text[i + 1].isspace():
            distance = abs(i + 1 - position)
            if distance < min_distance:
                min_distance = distance
                best_boundary = i + 1

    return best_boundary


def chunk_text_semantically(
    text: str,
    min_size: int = Config.MIN_CHUNK_SIZE,
    max_size: int = Config.MAX_CHUNK_SIZE,
    overlap: int = Config.CHUNK_OVERLAP,
) -> list[str]:
    """
    Split text into semantic chunks respecting sentence boundaries.

    Implements smart chunking:
    - Target chunk size: 800-1200 characters
    - Overlap: 150-200 characters between chunks
    - Respects sentence boundaries when possible
    - Handles edge cases (very long sentences, short texts)

    Args:
        text: Full text to chunk
        min_size: Minimum chunk size in characters
        max_size: Maximum chunk size in characters
        overlap: Overlap between consecutive chunks

    Returns:
        List of text chunks
    """
    if not text or len(text) < min_size:
        return [text] if text else []

    chunks: list[str] = []
    target_size = (min_size + max_size) // 2  # Target middle of range
    position = 0

    while position < len(text):
        # Calculate end position for this chunk
        end_position = position + target_size

        if end_position >= len(text):
            # Last chunk - include everything remaining
            chunk = text[position:].strip()
            if chunk:
                chunks.append(chunk)
            break

        # Find sentence boundary near target end
        boundary = find_sentence_boundary(text, end_position)

        # Ensure chunk is within size limits
        chunk_end = boundary
        if chunk_end - position > max_size:
            # Too long - truncate at max_size, find sentence boundary there
            chunk_end = find_sentence_boundary(text, position + max_size, search_range=30)
        elif chunk_end - position < min_size:
            # Too short - extend to min_size
            chunk_end = find_sentence_boundary(text, position + min_size, search_range=30)

        # Extract chunk
        chunk = text[position:chunk_end].strip()
        if chunk:
            chunks.append(chunk)

        # Move position with overlap (go back by overlap amount)
        position = max(position + 1, chunk_end - overlap)

    # Filter out very short chunks that might be artifacts
    chunks = [c for c in chunks if len(c) >= min_size // 2]

    logger.debug(f"Created {len(chunks)} chunks from {len(text)} characters")
    return chunks


# =============================================================================
# Async HTTP Fetching
# =============================================================================


async def fetch_chapter_html(
    session: aiohttp.ClientSession, url: str, timeout: int = 30
) -> tuple[str, str | None]:
    """
    Fetch HTML content from a chapter URL asynchronously.

    Args:
        session: aiohttp client session
        url: Full URL to fetch
        timeout: Request timeout in seconds

    Returns:
        Tuple of (url, html_content or None if failed)
    """
    try:
        async with session.get(url, timeout=aiohttp.ClientTimeout(total=timeout)) as response:
            if response.status == 200:
                html = await response.text()
                logger.info(f"Fetched {url} ({len(html)} bytes)")
                return url, html
            else:
                logger.warning(f"Failed to fetch {url}: HTTP {response.status}")
                return url, None
    except asyncio.TimeoutError:
        logger.error(f"Timeout fetching {url}")
        return url, None
    except aiohttp.ClientError as e:
        logger.error(f"HTTP error fetching {url}: {e}")
        return url, None
    except Exception as e:
        logger.error(f"Unexpected error fetching {url}: {e}")
        return url, None


async def fetch_all_chapters(base_url: str, chapter_paths: list[str]) -> dict[str, str]:
    """
    Fetch all chapter HTML pages concurrently.

    Args:
        base_url: Base URL of the textbook website
        chapter_paths: List of relative chapter paths

    Returns:
        Dict mapping chapter path to HTML content
    """
    results: dict[str, str] = {}

    async with aiohttp.ClientSession() as session:
        tasks = []
        for path in chapter_paths:
            full_url = f"{base_url.rstrip('/')}{path}"
            tasks.append(fetch_chapter_html(session, full_url))

        # Gather all results concurrently
        responses = await asyncio.gather(*tasks, return_exceptions=True)

        for path, response in zip(chapter_paths, responses):
            if isinstance(response, Exception):
                logger.error(f"Exception fetching {path}: {response}")
                continue
            url, html = response
            if html:
                results[path] = html

    logger.info(f"Successfully fetched {len(results)}/{len(chapter_paths)} chapters")
    return results


# =============================================================================
# Embedding Generation
# =============================================================================


@dataclass
class EmbeddingClient:
    """
    Wrapper around Cohere client for batch embedding generation.
    Handles rate limiting and error recovery.
    """

    client: cohere.ClientV2
    model: str = "embed-english-v3.0"
    input_type: str = "search_document"
    batch_size: int = Config.EMBEDDING_BATCH_SIZE
    _request_count: int = field(default=0, init=False)

    async def embed_batch(self, texts: list[str]) -> list[list[float]]:
        """
        Generate embeddings for a batch of texts.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors (1024 dimensions each)
        """
        if not texts:
            return []

        embeddings: list[list[float]] = []

        # Process in batches to respect rate limits
        for i in range(0, len(texts), self.batch_size):
            batch = texts[i : i + self.batch_size]

            try:
                # Add delay between batches to respect rate limits
                if self._request_count > 0:
                    await asyncio.sleep(0.5)  # 500ms delay between batches

                response = self.client.embed(
                    model=self.model,
                    input_type=self.input_type,
                    texts=batch,
                )

                self._request_count += 1

                # Extract embeddings from response
                # Cohere API v2 returns embeddings as iterable
                batch_embeddings = self._extract_embeddings(response)
                embeddings.extend(batch_embeddings)

                logger.debug(
                    f"Embedded batch {i // self.batch_size + 1}, "
                    f"texts: {len(batch)}, total: {len(embeddings)}"
                )

            except cohere.errors.TooManyRequestsError:
                logger.warning("Rate limited, waiting 5 seconds...")
                await asyncio.sleep(5)
                # Retry this batch
                response = self.client.embed(
                    model=self.model,
                    input_type=self.input_type,
                    texts=batch,
                )
                batch_embeddings = self._extract_embeddings(response)
                embeddings.extend(batch_embeddings)

            except Exception as e:
                logger.error(f"Embedding error for batch {i}: {e}")
                # Return empty embeddings for failed texts
                embeddings.extend([[] for _ in batch])

        return embeddings

    def _extract_embeddings(self, response: Any) -> list[list[float]]:
        """Extract embedding vectors from Cohere API response."""
        try:
            # Handle Cohere 
            # v2 response format
            embeddings_data = response.embeddings

            # Check if it's the new format (iterable of tuples)
            if hasattr(embeddings_data, "__iter__"):
                result = list(embeddings_data)
                if result and isinstance(result[0], tuple):
                    # Format: [('float_', [[vec1], [vec2], ...])]
                    _, vectors = result[0]
                    return [list(v) for v in vectors]
                elif result and isinstance(result[0], list):
                    return [list(v) for v in result]

            # Fallback: direct list access
            return [list(v) for v in embeddings_data]

        except Exception as e:
            logger.error(f"Error extracting embeddings: {e}")
            return []


# =============================================================================
# Qdrant Operations
# =============================================================================


class QdrantManager:
    """
    Manages Qdrant collection operations including creation, upsertion, and search.
    """

    def __init__(
        self,
        client: QdrantClient,
        collection_name: str = Config.COLLECTION_NAME,
        vector_dimension: int = Config.EMBEDDING_DIMENSION,
    ):
        self.client = client
        self.collection_name = collection_name
        self.vector_dimension = vector_dimension

    def ensure_collection(self) -> bool:
        """
        Create collection if it doesn't exist, or verify existing collection.

        Returns:
            True if collection is ready, False on error
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name in collection_names:
                # Verify vector configuration
                collection_info = self.client.get_collection(self.collection_name)
                logger.info(
                    f"Collection '{self.collection_name}' exists with "
                    f"{collection_info.points_count} points"
                )
                return True

            # Create new collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=qdrant_models.VectorParams(
                    size=self.vector_dimension,
                    distance=qdrant_models.Distance.COSINE,
                ),
            )
            logger.info(f"Created collection '{self.collection_name}'")
            return True

        except UnexpectedResponse as e:
            logger.error(f"Qdrant API error: {e}")
            return False
        except Exception as e:
            logger.error(f"Error ensuring collection: {e}")
            return False

    def clear_collection(self) -> bool:
        """
        Delete all points in the collection (fresh start).

        Returns:
            True if successful, False on error
        """
        try:
            # Delete and recreate collection for clean slate
            self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted collection '{self.collection_name}'")

            # Recreate
            return self.ensure_collection()

        except UnexpectedResponse:
            # Collection might not exist
            return self.ensure_collection()
        except Exception as e:
            logger.error(f"Error clearing collection: {e}")
            return False

    def upsert_chunks(
        self, chunks: list[TextChunk], batch_size: int = Config.UPSERT_BATCH_SIZE
    ) -> int:
        """
        Upsert text chunks with embeddings to Qdrant.

        Args:
            chunks: List of TextChunk objects with embeddings
            batch_size: Number of points to upsert per batch

        Returns:
            Number of successfully upserted chunks
        """
        upserted = 0
        points: list[qdrant_models.PointStruct] = []

        for chunk in chunks:
            if not chunk.embedding:
                logger.warning(f"Skipping chunk {chunk.chunk_id} - no embedding")
                continue

            point = qdrant_models.PointStruct(
                id=chunk.chunk_id,
                vector=chunk.embedding,
                payload=chunk.to_qdrant_payload(),
            )
            points.append(point)

            # Batch upsert
            if len(points) >= batch_size:
                try:
                    self.client.upsert(
                        collection_name=self.collection_name,
                        points=points,
                        wait=True,
                    )
                    upserted += len(points)
                    logger.debug(f"Upserted batch of {len(points)} points")
                    points = []
                except Exception as e:
                    logger.error(f"Batch upsert error: {e}")

        # Upsert remaining points
        if points:
            try:
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points,
                    wait=True,
                )
                upserted += len(points)
                logger.debug(f"Upserted final batch of {len(points)} points")
            except Exception as e:
                logger.error(f"Final batch upsert error: {e}")

        logger.info(f"Upserted {upserted} chunks to '{self.collection_name}'")
        return upserted

    def search(
        self,
        query_embedding: list[float],
        limit: int = 5,
    ) -> list[SearchResult]:
        """
        Perform semantic search in the collection.

        Args:
            query_embedding: Query vector (1024 dimensions)
            limit: Maximum number of results

        Returns:
            List of SearchResult objects
        """
        try:
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
            )

            search_results: list[SearchResult] = []
            for point in results.points:
                payload = point.payload or {}
                search_results.append(
                    SearchResult(
                        chunk_id=payload.get("chunk_id", str(point.id)),
                        chapter_num=payload.get("chapter_num", 0),
                        chapter_title=payload.get("chapter_title", "Unknown"),
                        chapter_url=payload.get("chapter_url", ""),
                        text_snippet=payload.get("text_snippet", ""),
                        similarity_score=point.score or 0.0,
                    )
                )

            return search_results

        except Exception as e:
            logger.error(f"Search error: {e}")
            return []

    def get_collection_info(self) -> dict[str, Any]:
        """Get collection statistics."""
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "points_count": info.points_count,
                "status": info.status.value if info.status else "unknown",
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return {"name": self.collection_name, "error": str(e)}


# =============================================================================
# Main Pipeline
# =============================================================================


class EmbeddingPipeline:
    """
    Orchestrates the complete embedding pipeline across all 8 phases.
    """

    def __init__(self):
        """Initialize pipeline with configuration validation."""
        # Phase 1: Validate configuration
        missing = Config.validate()
        if missing:
            raise ValueError(f"Missing required environment variables: {missing}")

        # Initialize clients
        self.cohere_client = cohere.ClientV2(api_key=Config.COHERE_API_KEY)
        self.qdrant_client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
        )

        # Initialize managers
        self.embedding_client = EmbeddingClient(client=self.cohere_client)
        self.qdrant_manager = QdrantManager(client=self.qdrant_client)

        # Pipeline state
        self.chunks: list[TextChunk] = []
        self.errors: list[str] = []
        self.start_time: float = 0
        self.end_time: float = 0

        logger.info("Embedding pipeline initialized")

    async def run(self, clear_collection: bool = True) -> PipelineReport:
        """
        Execute the complete 8-phase embedding pipeline.

        Args:
            clear_collection: Whether to clear existing collection data

        Returns:
            PipelineReport with summary statistics
        """
        self.start_time = time.time()
        start_datetime = datetime.now(timezone.utc).isoformat()

        logger.info("=" * 60)
        logger.info("PHASE 1: Backend Setup & Client Initialization")
        logger.info("=" * 60)

        # Ensure Qdrant collection exists
        if clear_collection:
            if not self.qdrant_manager.clear_collection():
                self.errors.append("Failed to clear/create Qdrant collection")
        else:
            if not self.qdrant_manager.ensure_collection():
                self.errors.append("Failed to ensure Qdrant collection")

        logger.info("=" * 60)
        logger.info("PHASE 2: Fetch Docusaurus Chapters")
        logger.info("=" * 60)

        # Fetch all chapters
        chapter_html = await fetch_all_chapters(
            Config.TEXTBOOK_BASE_URL, Config.CHAPTER_URLS
        )

        if not chapter_html:
            self.errors.append("No chapters fetched - check network connectivity")
            return self._generate_report(start_datetime)

        logger.info("=" * 60)
        logger.info("PHASE 3: Extract Clean Text from HTML")
        logger.info("=" * 60)

        # Extract text from each chapter
        chapter_texts: dict[str, str] = {}
        for path, html in chapter_html.items():
            text = extract_text_from_html(html)
            if text and len(text) > 50:  # Lowered threshold for Docusaurus client-rendered content
                chapter_texts[path] = text
                logger.info(f"Extracted {len(text)} chars from {path}")
            else:
                logger.warning(f"Insufficient content from {path} (got {len(text) if text else 0} chars)")

        logger.info("=" * 60)
        logger.info("PHASE 4: Semantic Text Chunking")
        logger.info("=" * 60)

        # Chunk each chapter's text
        for path, text in chapter_texts.items():
            metadata = Config.CHAPTER_METADATA.get(path, {})
            chapter_num = metadata.get("chapter_num", 0)
            chapter_title = metadata.get("chapter_title", "Unknown")
            full_url = f"{Config.TEXTBOOK_BASE_URL}{path}"

            text_chunks = chunk_text_semantically(text)
            logger.info(f"Chapter {chapter_num}: {len(text_chunks)} chunks")

            for idx, chunk_text in enumerate(text_chunks):
                chunk = TextChunk(
                    chapter_num=chapter_num,
                    chapter_title=chapter_title,
                    chapter_url=full_url,
                    text=chunk_text,
                    chunk_index=idx,
                )
                self.chunks.append(chunk)

        logger.info(f"Total chunks created: {len(self.chunks)}")

        logger.info("=" * 60)
        logger.info("PHASE 5: Batch-Generate Cohere Embeddings")
        logger.info("=" * 60)

        # Generate embeddings in batches
        texts_to_embed = [chunk.text for chunk in self.chunks]
        embeddings = await self.embedding_client.embed_batch(texts_to_embed)

        # Assign embeddings to chunks
        embedded_count = 0
        for chunk, embedding in zip(self.chunks, embeddings):
            if embedding:
                chunk.embedding = embedding
                chunk.status = ChunkStatus.EMBEDDED
                embedded_count += 1
            else:
                chunk.status = ChunkStatus.FAILED

        logger.info(f"Successfully embedded: {embedded_count}/{len(self.chunks)} chunks")

        logger.info("=" * 60)
        logger.info("PHASE 6: Upsert to Qdrant Collection")
        logger.info("=" * 60)

        # Upsert chunks with embeddings
        embedded_chunks = [c for c in self.chunks if c.status == ChunkStatus.EMBEDDED]
        upserted_count = self.qdrant_manager.upsert_chunks(embedded_chunks)

        # Update chunk status
        for chunk in embedded_chunks[:upserted_count]:
            chunk.status = ChunkStatus.UPSERTED

        logger.info("=" * 60)
        logger.info("PHASE 7: Verify Semantic Search")
        logger.info("=" * 60)

        # Run test queries
        await self._run_test_queries()

        logger.info("=" * 60)
        logger.info("PHASE 8: Generate Summary Report")
        logger.info("=" * 60)

        self.end_time = time.time()
        return self._generate_report(start_datetime)

    async def _run_test_queries(self) -> None:
        """Run test queries to verify semantic search functionality."""
        for query in Config.TEST_QUERIES:
            logger.info(f"\nTest Query: '{query[:50]}...'")

            # Embed query
            query_embeddings = await self.embedding_client.embed_batch([query])
            if not query_embeddings or not query_embeddings[0]:
                logger.error("Failed to embed query")
                continue

            # Search
            results = self.qdrant_manager.search(query_embeddings[0], limit=5)

            # Log top results
            for i, result in enumerate(results, 1):
                logger.info(
                    f"  #{i} [Score: {result.similarity_score:.4f}] "
                    f"Chapter {result.chapter_num}: {result.chapter_title}"
                )
                logger.info(f"      Snippet: {result.text_snippet[:80]}...")

    def _generate_report(self, start_datetime: str) -> PipelineReport:
        """Generate final pipeline report."""
        end_datetime = datetime.now(timezone.utc).isoformat()
        elapsed = self.end_time - self.start_time if self.end_time else 0

        # Count by status
        upserted = sum(1 for c in self.chunks if c.status == ChunkStatus.UPSERTED)
        embedded = sum(1 for c in self.chunks if c.status in (ChunkStatus.EMBEDDED, ChunkStatus.UPSERTED))
        failed = sum(1 for c in self.chunks if c.status == ChunkStatus.FAILED)

        # Count unique chapters
        chapters = set(c.chapter_num for c in self.chunks)

        report = PipelineReport(
            start_time=start_datetime,
            end_time=end_datetime,
            elapsed_seconds=round(elapsed, 2),
            chapters_processed=len(chapters),
            total_chunks=len(self.chunks),
            chunks_embedded=embedded,
            chunks_upserted=upserted,
            chunks_failed=failed,
            collection_name=Config.COLLECTION_NAME,
            test_queries_run=len(Config.TEST_QUERIES),
            success=len(self.errors) == 0 and upserted > 0,
            errors=self.errors,
        )

        # Print formatted report
        self._print_report(report)

        return report

    def _print_report(self, report: PipelineReport) -> None:
        """Print formatted console report."""
        print("\n" + "=" * 60)
        print("EMBEDDING PIPELINE - FINAL REPORT")
        print("=" * 60)
        print(f"Status:              {'SUCCESS' if report.success else 'FAILED'}")
        print(f"Collection:          {report.collection_name}")
        print(f"Elapsed Time:        {report.elapsed_seconds:.2f} seconds")
        print("-" * 60)
        print(f"Chapters Processed:  {report.chapters_processed}")
        print(f"Total Chunks:        {report.total_chunks}")
        print(f"Chunks Embedded:     {report.chunks_embedded}")
        print(f"Chunks Upserted:     {report.chunks_upserted}")
        print(f"Chunks Failed:       {report.chunks_failed}")
        print(f"Test Queries Run:    {report.test_queries_run}")
        print("-" * 60)
        print(f"Start Time:          {report.start_time}")
        print(f"End Time:            {report.end_time}")

        if report.errors:
            print("-" * 60)
            print("ERRORS:")
            for error in report.errors:
                print(f"  - {error}")

        print("=" * 60 + "\n")


# =============================================================================
# Entry Point
# =============================================================================


async def main() -> int:
    """
    Main entry point for the embedding pipeline.

    Returns:
        Exit code (0 for success, 1 for failure)
    """
    try:
        pipeline = EmbeddingPipeline()
        report = await pipeline.run(clear_collection=True)

        if report.success:
            logger.info("Embedding pipeline completed successfully!")
            return 0
        else:
            logger.error("Embedding pipeline completed with errors")
            return 1

    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        print(f"\nERROR: {e}")
        print("\nPlease set the following environment variables:")
        print("  - COHERE_API_KEY: Your Cohere API key")
        print("  - QDRANT_URL: Qdrant cloud/server URL")
        print("  - QDRANT_API_KEY: Qdrant API key")
        print("  - TEXTBOOK_BASE_URL: (optional) Base URL of the textbook")
        return 1

    except KeyboardInterrupt:
        logger.info("Pipeline interrupted by user")
        return 130

    except Exception as e:
        logger.exception(f"Unexpected error: {e}")
        return 1


if __name__ == "__main__":
    # Run the async pipeline
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
