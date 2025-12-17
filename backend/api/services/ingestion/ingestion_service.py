"""Content ingestion service for embedding and storing documents."""

import uuid
from typing import Optional, Dict, Any, List
from datetime import datetime, timezone

from .config import Settings
from .services.openrouter_client import OpenRouterClient
from .services.retrieval.qdrant_store import QdrantStore
from .utils import get_logger, GenerationError
from qdrant_client.models import PointStruct

from .text_extractor import extract_text_from_html
from .chunker import chunk_text_semantically

logger = get_logger(__name__)


class IngestedChunk:
    """Represents an ingested text chunk with metadata."""

    def __init__(
        self,
        text: str,
        chapter_num: int,
        chapter_name: str,
        section_name: Optional[str] = None,
        source_url: str = "",
        page_num: Optional[int] = None,
    ):
        """Initialize chunk.

        Args:
            text: Chunk content
            chapter_num: Chapter number
            chapter_name: Chapter name
            section_name: Optional section name
            source_url: Source URL
            page_num: Optional page number
        """
        self.chunk_id = str(uuid.uuid4())
        self.text = text
        self.chapter_num = chapter_num
        self.chapter_name = chapter_name
        self.section_name = section_name
        self.source_url = source_url
        self.page_num = page_num
        self.embedding: List[float] = []
        self.created_at = datetime.now(timezone.utc).isoformat()

    def to_point_struct(self) -> PointStruct:
        """Convert to Qdrant PointStruct.

        Returns:
            PointStruct with embedding and payload
        """
        return PointStruct(
            id=self.chunk_id,
            vector=self.embedding,
            payload={
                "chunk_id": self.chunk_id,
                "chapter_num": self.chapter_num,
                "chapter_name": self.chapter_name,
                "section_name": self.section_name,
                "content": self.text,
                "source_url": self.source_url,
                "page_num": self.page_num,
                "created_at": self.created_at,
            },
        )


class IngestionService:
    """Service for ingesting and embedding content."""

    def __init__(
        self,
        settings: Settings,
        openrouter_client: OpenRouterClient,
        qdrant_store: QdrantStore,
    ):
        """Initialize ingestion service.

        Args:
            settings: Application settings
            openrouter_client: OpenRouter API client
            qdrant_store: Qdrant vector store
        """
        self.settings = settings
        self.openrouter = openrouter_client
        self.qdrant = qdrant_store
        logger.info("IngestionService initialized")

    async def ingest_html_content(
        self,
        html_content: str,
        chapter_num: int,
        chapter_name: str,
        section_name: Optional[str] = None,
        source_url: str = "",
        page_num: Optional[int] = None,
    ) -> Dict[str, Any]:
        """Ingest HTML content from a chapter.

        Args:
            html_content: Raw HTML from page
            chapter_num: Chapter number
            chapter_name: Chapter name
            section_name: Optional section name
            source_url: Source URL
            page_num: Optional page number

        Returns:
            Ingestion result with statistics
        """
        try:
            logger.info(f"Ingesting {chapter_name}...")

            # Step 1: Extract text from HTML
            text = extract_text_from_html(html_content)
            if not text or len(text) < 50:
                logger.warning(f"Insufficient content extracted from {chapter_name}")
                return {
                    "success": False,
                    "message": "Insufficient content extracted",
                    "chunks_processed": 0,
                    "chunks_stored": 0,
                }

            logger.debug(f"Extracted {len(text)} characters from {chapter_name}")

            # Step 2: Chunk text semantically
            text_chunks = chunk_text_semantically(
                text,
                min_size=self.settings.chunk_size // 2,
                max_size=self.settings.chunk_size,
                overlap=self.settings.chunk_overlap,
            )

            logger.info(f"Created {len(text_chunks)} chunks from {chapter_name}")

            # Step 3: Create IngestedChunk objects
            chunks = [
                IngestedChunk(
                    text=chunk_text,
                    chapter_num=chapter_num,
                    chapter_name=chapter_name,
                    section_name=section_name,
                    source_url=source_url,
                    page_num=page_num,
                )
                for chunk_text in text_chunks
            ]

            # Step 4: Generate embeddings
            texts = [c.text for c in chunks]
            embeddings = await self.openrouter.embed_texts_batch(texts)

            if len(embeddings) != len(chunks):
                logger.error(
                    f"Embedding count mismatch: got {len(embeddings)}, expected {len(chunks)}"
                )
                return {
                    "success": False,
                    "message": "Embedding generation failed",
                    "chunks_processed": len(chunks),
                    "chunks_stored": 0,
                }

            # Assign embeddings to chunks
            for chunk, embedding in zip(chunks, embeddings):
                chunk.embedding = embedding

            # Step 5: Upsert to Qdrant
            points = [chunk.to_point_struct() for chunk in chunks]
            await self.qdrant.upsert_vectors(points)

            logger.info(f"Successfully ingested {len(chunks)} chunks from {chapter_name}")

            return {
                "success": True,
                "message": f"Successfully ingested {chapter_name}",
                "chunks_processed": len(chunks),
                "chunks_stored": len(chunks),
                "chapter_num": chapter_num,
                "chapter_name": chapter_name,
            }

        except Exception as e:
            logger.error(f"Ingestion failed for {chapter_name}: {e}")
            raise GenerationError(f"Failed to ingest content: {str(e)}")

    async def ingest_docusaurus_chapters(
        self,
        chapters: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Ingest multiple Docusaurus chapters.

        Args:
            chapters: List of chapter dicts with html, chapter_num, chapter_name, source_url

        Returns:
            Aggregated ingestion results
        """
        total_processed = 0
        total_stored = 0
        errors = []

        for chapter in chapters:
            try:
                result = await self.ingest_html_content(
                    html_content=chapter.get("html", ""),
                    chapter_num=chapter.get("chapter_num", 0),
                    chapter_name=chapter.get("chapter_name", "Unknown"),
                    section_name=chapter.get("section_name"),
                    source_url=chapter.get("source_url", ""),
                    page_num=chapter.get("page_num"),
                )

                if result["success"]:
                    total_processed += result["chunks_processed"]
                    total_stored += result["chunks_stored"]
                else:
                    errors.append(result["message"])

            except Exception as e:
                logger.error(f"Error processing chapter: {e}")
                errors.append(str(e))

        return {
            "success": len(errors) == 0,
            "total_chapters": len(chapters),
            "total_chunks_processed": total_processed,
            "total_chunks_stored": total_stored,
            "errors": errors,
        }

    async def verify_ingestion(self) -> Dict[str, Any]:
        """Verify ingestion by checking collection status.

        Returns:
            Collection information
        """
        try:
            info = await self.qdrant.get_collection_info()
            return {
                "collection": info["name"],
                "points_count": info["points_count"],
                "status": "ready",
            }
        except Exception as e:
            logger.error(f"Failed to verify ingestion: {e}")
            return {
                "status": "error",
                "message": str(e),
            }
