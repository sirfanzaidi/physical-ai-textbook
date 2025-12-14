"""
Book ingestion pipeline orchestration.

Handles file upload, parsing, chunking, embedding, and storage.
"""

from typing import List, Dict, Any, Optional, BinaryIO, Tuple
import hashlib
import io
import structlog
from pathlib import Path

from ingestion.chunker import SemanticChunker
from ingestion.embedder import CohereEmbedder
from database.qdrant_client import QdrantVectorStore
from database.postgres_client import PostgresMetadataStore
from utils.errors import IngestError, ValidationError
from utils.logging import log_ingestion_metrics
from utils.metrics import LatencyTracker, QueryMetrics

logger = structlog.get_logger(__name__)


class BookIngestionPipeline:
    """Orchestrates complete book ingestion pipeline."""

    def __init__(
        self,
        chunker: SemanticChunker,
        embedder: CohereEmbedder,
        vector_store: QdrantVectorStore,
        metadata_store: Optional[PostgresMetadataStore] = None,
        max_pages: int = 500,
    ):
        """
        Initialize ingestion pipeline.

        Args:
            chunker: Initialized SemanticChunker
            embedder: Initialized CohereEmbedder
            vector_store: Initialized QdrantVectorStore
            metadata_store: Optional PostgresMetadataStore for metadata
            max_pages: Maximum pages allowed per book
        """
        self.chunker = chunker
        self.embedder = embedder
        self.vector_store = vector_store
        self.metadata_store = metadata_store
        self.max_pages = max_pages
        logger.info("book_ingestion_pipeline_initialized", max_pages=max_pages)

    def ingest_file(
        self,
        file_content: bytes,
        file_name: str,
        book_id: str,
        file_type: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Ingest a book file (PDF, TXT, or Markdown).

        Pipeline:
        1. Validate file
        2. Extract text and metadata
        3. Chunk into semantic units
        4. Generate embeddings
        5. Store in vector DB
        6. Store metadata in PostgreSQL

        Args:
            file_content: Binary file content
            file_name: Original file name
            book_id: Unique book identifier
            file_type: File type hint (pdf, txt, md). Auto-detected if None.

        Returns:
            Dict with ingestion results

        Raises:
            IngestError: If ingestion fails
            ValidationError: If file validation fails
        """
        pipeline_start = LatencyTracker()
        pipeline_start.__enter__()

        try:
            # Step 1: Validate file
            file_type = file_type or self._detect_file_type(file_name)
            file_size_mb = len(file_content) / (1024 * 1024)
            self._validate_file(file_content, file_name, file_type, file_size_mb)

            # Step 2: Extract text and metadata
            extraction_start = LatencyTracker()
            extraction_start.__enter__()

            extracted_data = self._extract_text(file_content, file_type, file_name)
            text = extracted_data["text"]
            metadata = extracted_data["metadata"]

            extraction_start.__exit__(None, None, None)

            logger.info(
                "text_extracted",
                book_id=book_id,
                pages=metadata.get("page_count", 0),
                extraction_latency_ms=extraction_start.elapsed_ms,
            )

            # Step 3: Chunk text
            chunking_start = LatencyTracker()
            chunking_start.__enter__()

            chunks = self.chunker.chunk(
                text=text,
                book_id=book_id,
                page_num=None,  # Page numbers extracted per-chunk
                section_name=metadata.get("title"),
                chapter_name=None,
            )

            chunking_start.__exit__(None, None, None)

            if not chunks:
                raise IngestError(
                    "No chunks produced from text extraction",
                    error_code="NO_CHUNKS_PRODUCED",
                    details={"text_length": len(text)},
                )

            total_tokens = self.chunker.calculate_total_tokens(chunks)

            logger.info(
                "text_chunked",
                book_id=book_id,
                chunk_count=len(chunks),
                total_tokens=total_tokens,
                chunking_latency_ms=chunking_start.elapsed_ms,
            )

            # Step 4: Generate embeddings
            embedding_start = LatencyTracker()
            embedding_start.__enter__()

            chunks_with_embeddings = self.embedder.embed_chunks(chunks)

            embedding_start.__exit__(None, None, None)

            logger.info(
                "embeddings_generated",
                book_id=book_id,
                chunk_count=len(chunks_with_embeddings),
                embedding_latency_ms=embedding_start.elapsed_ms,
            )

            # Step 5: Store in vector database
            storage_start = LatencyTracker()
            storage_start.__enter__()

            self._store_vectors(book_id, chunks_with_embeddings)

            storage_start.__exit__(None, None, None)

            logger.info(
                "vectors_stored",
                book_id=book_id,
                count=len(chunks_with_embeddings),
                storage_latency_ms=storage_start.elapsed_ms,
            )

            # Step 6: Store metadata
            if self.metadata_store:
                metadata_start = LatencyTracker()
                metadata_start.__enter__()

                self._store_metadata(book_id, chunks_with_embeddings)

                metadata_start.__exit__(None, None, None)

                logger.info(
                    "metadata_stored",
                    book_id=book_id,
                    count=len(chunks_with_embeddings),
                    metadata_latency_ms=metadata_start.elapsed_ms,
                )

            pipeline_start.__exit__(None, None, None)

            result = {
                "success": True,
                "book_id": book_id,
                "file_name": file_name,
                "file_type": file_type,
                "chunk_count": len(chunks_with_embeddings),
                "total_tokens": total_tokens,
                "page_count": metadata.get("page_count", 0),
                "file_size_mb": round(file_size_mb, 2),
                "pipeline_latency_ms": pipeline_start.elapsed_ms,
                "message": f"Successfully ingested {file_name} with {len(chunks_with_embeddings)} chunks",
            }

            log_ingestion_metrics(
                logger,
                book_id=book_id,
                file_name=file_name,
                total_pages=metadata.get("page_count", 0),
                chunks_created=len(chunks_with_embeddings),
                total_tokens=total_tokens,
                ingestion_latency_ms=pipeline_start.elapsed_ms,
            )

            return result

        except (IngestError, ValidationError):
            raise
        except Exception as e:
            raise IngestError(
                f"Unexpected error during ingestion: {str(e)}",
                error_code="INGESTION_UNEXPECTED_ERROR",
                details={"file_name": file_name, "book_id": book_id, "error": str(e)},
            )

    def _detect_file_type(self, file_name: str) -> str:
        """
        Detect file type from extension.

        Args:
            file_name: File name with extension

        Returns:
            File type (pdf, txt, md)

        Raises:
            ValidationError: If file type not supported
        """
        extension = Path(file_name).suffix.lower()
        type_map = {".pdf": "pdf", ".txt": "txt", ".md": "md"}

        if extension not in type_map:
            raise ValidationError(
                f"Unsupported file type: {extension}. Supported: .pdf, .txt, .md",
                error_code="UNSUPPORTED_FILE_TYPE",
                details={"file_name": file_name, "extension": extension},
            )

        return type_map[extension]

    def _validate_file(
        self, file_content: bytes, file_name: str, file_type: str, file_size_mb: float
    ) -> None:
        """
        Validate file before processing.

        Args:
            file_content: Binary file content
            file_name: File name
            file_type: File type (pdf, txt, md)
            file_size_mb: File size in MB

        Raises:
            ValidationError: If validation fails
        """
        # Check file size (reasonable limit)
        if file_size_mb > 50:
            raise ValidationError(
                f"File too large: {file_size_mb}MB. Maximum 50MB allowed.",
                error_code="FILE_TOO_LARGE",
                details={"file_name": file_name, "size_mb": file_size_mb},
            )

        # Check file not empty
        if not file_content or len(file_content) < 100:
            raise ValidationError(
                "File is empty or too small (minimum 100 bytes)",
                error_code="FILE_TOO_SMALL",
                details={"file_name": file_name, "size": len(file_content)},
            )

        logger.info("file_validated", file_name=file_name, file_type=file_type, size_mb=file_size_mb)

    def _extract_text(self, file_content: bytes, file_type: str, file_name: str) -> Dict[str, Any]:
        """
        Extract text from file.

        Args:
            file_content: Binary file content
            file_type: File type (pdf, txt, md)
            file_name: Original file name

        Returns:
            Dict with extracted text and metadata

        Raises:
            IngestError: If extraction fails
        """
        try:
            if file_type == "pdf":
                return self._extract_pdf(file_content, file_name)
            elif file_type in ("txt", "md"):
                return self._extract_text_file(file_content, file_name)
            else:
                raise IngestError(
                    f"Unknown file type: {file_type}",
                    error_code="UNKNOWN_FILE_TYPE",
                )
        except IngestError:
            raise
        except Exception as e:
            raise IngestError(
                f"Failed to extract text from {file_type} file: {str(e)}",
                error_code="TEXT_EXTRACTION_FAILED",
                details={"file_type": file_type, "error": str(e)},
            )

    def _extract_pdf(self, file_content: bytes, file_name: str) -> Dict[str, Any]:
        """
        Extract text from PDF using PyPDF2.

        Args:
            file_content: PDF binary content
            file_name: File name

        Returns:
            Dict with text and metadata
        """
        try:
            import PyPDF2

            pdf_reader = PyPDF2.PdfReader(io.BytesIO(file_content))
            page_count = len(pdf_reader.pages)

            if page_count > self.max_pages:
                raise ValidationError(
                    f"PDF has {page_count} pages. Maximum {self.max_pages} allowed.",
                    error_code="PDF_TOO_LARGE",
                    details={"pages": page_count, "max_pages": self.max_pages},
                )

            text_parts = []
            for page_num, page in enumerate(pdf_reader.pages):
                text = page.extract_text()
                if text:
                    text_parts.append(text)

            full_text = "\n\n".join(text_parts)

            return {
                "text": full_text,
                "metadata": {
                    "title": Path(file_name).stem,
                    "page_count": page_count,
                    "file_type": "pdf",
                },
            }

        except Exception as e:
            raise IngestError(
                f"PDF extraction failed: {str(e)}",
                error_code="PDF_EXTRACTION_FAILED",
                details={"file_name": file_name, "error": str(e)},
            )

    def _extract_text_file(self, file_content: bytes, file_name: str) -> Dict[str, Any]:
        """
        Extract text from TXT or Markdown file.

        Args:
            file_content: File binary content
            file_name: File name

        Returns:
            Dict with text and metadata
        """
        try:
            text = file_content.decode("utf-8")

            # Estimate page count (roughly 3000 chars per page)
            estimated_pages = max(1, len(text) // 3000)

            if estimated_pages > self.max_pages:
                raise ValidationError(
                    f"File content ~{estimated_pages} pages. Maximum {self.max_pages} allowed.",
                    error_code="FILE_TOO_LARGE",
                    details={"pages": estimated_pages, "max_pages": self.max_pages},
                )

            return {
                "text": text,
                "metadata": {
                    "title": Path(file_name).stem,
                    "page_count": estimated_pages,
                    "file_type": Path(file_name).suffix[1:],
                },
            }

        except UnicodeDecodeError as e:
            raise IngestError(
                f"File is not valid UTF-8 text: {str(e)}",
                error_code="INVALID_TEXT_ENCODING",
                details={"file_name": file_name},
            )

    def _store_vectors(self, book_id: str, chunks: List[Dict[str, Any]]) -> None:
        """
        Store chunk embeddings in Qdrant vector database.

        Args:
            book_id: Book identifier
            chunks: Chunks with embeddings

        Raises:
            IngestError: If storage fails
        """
        try:
            chunk_ids = [chunk["chunk_id"] for chunk in chunks]
            vectors = [chunk["embedding"] for chunk in chunks]
            metadata_list = [
                {
                    "chunk_id": chunk["chunk_id"],
                    "book_id": book_id,
                    "text": chunk["text"],
                    "page_num": chunk.get("page_num"),
                    "section_name": chunk.get("section_name"),
                    "chapter_name": chunk.get("chapter_name"),
                    "token_count": chunk.get("token_count", 0),
                }
                for chunk in chunks
            ]

            self.vector_store.upsert_vectors(
                chunk_ids=chunk_ids,
                vectors=vectors,
                metadata=metadata_list,
            )

        except Exception as e:
            raise IngestError(
                f"Failed to store vectors in Qdrant: {str(e)}",
                error_code="VECTOR_STORAGE_FAILED",
                details={"book_id": book_id, "chunk_count": len(chunks), "error": str(e)},
            )

    def _store_metadata(self, book_id: str, chunks: List[Dict[str, Any]]) -> None:
        """
        Store chunk metadata in PostgreSQL.

        Args:
            book_id: Book identifier
            chunks: Chunks with metadata

        Raises:
            IngestError: If storage fails
        """
        try:
            for chunk in chunks:
                self.metadata_store.store_chunk_metadata(
                    chunk_id=chunk["chunk_id"],
                    book_id=book_id,
                    page_num=chunk.get("page_num"),
                    section_name=chunk.get("section_name"),
                    chapter_name=chunk.get("chapter_name"),
                    text_hash=hashlib.sha256(chunk["text"].encode()).hexdigest(),
                )

        except Exception as e:
            # Log warning but don't fail ingestion if metadata storage fails
            logger.warning(
                "metadata_storage_failed",
                book_id=book_id,
                chunk_count=len(chunks),
                error=str(e),
            )
