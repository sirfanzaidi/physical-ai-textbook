"""
Neon PostgreSQL client wrapper.

Provides interface for storing and retrieving chunk metadata.
"""

from typing import List, Optional, Dict, Any
import psycopg2
from psycopg2.extras import RealDictCursor
from psycopg2.pool import SimpleConnectionPool
import structlog
from datetime import datetime

from utils.errors import NeonError

logger = structlog.get_logger(__name__)


class PostgresMetadataStore:
    """Wrapper for Neon PostgreSQL metadata operations."""

    def __init__(self, database_url: str, pool_size: int = 5):
        """
        Initialize PostgreSQL connection pool.

        Args:
            database_url: PostgreSQL connection string
            pool_size: Connection pool size

        Raises:
            NeonError: If connection fails
        """
        try:
            self.connection_pool = SimpleConnectionPool(
                1,
                pool_size,
                database_url,
            )
            self.database_url = database_url
            logger.info(
                "postgres_pool_initialized",
                pool_size=pool_size,
            )
        except Exception as e:
            raise NeonError(
                f"Failed to initialize PostgreSQL connection pool: {str(e)}",
                error_code="POSTGRES_INIT_FAILED",
                details={"error": str(e)},
            )

    def store_chunk_metadata(
        self,
        chunk_id: str,
        book_id: str,
        page_num: Optional[int] = None,
        section_name: Optional[str] = None,
        chapter_name: Optional[str] = None,
        text_hash: Optional[str] = None,
    ) -> bool:
        """
        Store chunk metadata in PostgreSQL.

        Args:
            chunk_id: Unique chunk identifier
            book_id: Book identifier
            page_num: Page number (optional)
            section_name: Section name (optional)
            chapter_name: Chapter name (optional)
            text_hash: Hash of chunk text for deduplication

        Returns:
            True if successful

        Raises:
            NeonError: If operation fails
        """
        conn = None
        try:
            conn = self.connection_pool.getconn()
            cursor = conn.cursor()

            cursor.execute(
                """
                INSERT INTO chunks_metadata (
                    chunk_id, book_id, page_num, section_name, chapter_name, text_hash
                ) VALUES (%s, %s, %s, %s, %s, %s)
                ON CONFLICT (chunk_id) DO UPDATE SET
                    updated_at = CURRENT_TIMESTAMP
                """,
                (chunk_id, book_id, page_num, section_name, chapter_name, text_hash),
            )
            conn.commit()

            logger.info(
                "chunk_metadata_stored",
                chunk_id=chunk_id,
                book_id=book_id,
            )
            return True

        except Exception as e:
            if conn:
                conn.rollback()
            raise NeonError(
                f"Failed to store chunk metadata: {str(e)}",
                error_code="POSTGRES_STORE_FAILED",
                details={"chunk_id": chunk_id, "book_id": book_id, "error": str(e)},
            )
        finally:
            if conn:
                self.connection_pool.putconn(conn)

    def get_chunk_metadata(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve chunk metadata by ID.

        Args:
            chunk_id: Chunk identifier

        Returns:
            Metadata dict or None if not found

        Raises:
            NeonError: If query fails
        """
        conn = None
        try:
            conn = self.connection_pool.getconn()
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            cursor.execute(
                """
                SELECT * FROM chunks_metadata WHERE chunk_id = %s
                """,
                (chunk_id,),
            )
            result = cursor.fetchone()
            return dict(result) if result else None

        except Exception as e:
            raise NeonError(
                f"Failed to retrieve chunk metadata: {str(e)}",
                error_code="POSTGRES_RETRIEVE_FAILED",
                details={"chunk_id": chunk_id, "error": str(e)},
            )
        finally:
            if conn:
                self.connection_pool.putconn(conn)

    def get_book_chunks(self, book_id: str) -> List[Dict[str, Any]]:
        """
        Retrieve all chunks for a specific book.

        Args:
            book_id: Book identifier

        Returns:
            List of chunk metadata dicts

        Raises:
            NeonError: If query fails
        """
        conn = None
        try:
            conn = self.connection_pool.getconn()
            cursor = conn.cursor(cursor_factory=RealDictCursor)

            cursor.execute(
                """
                SELECT * FROM chunks_metadata WHERE book_id = %s
                ORDER BY page_num ASC, id ASC
                """,
                (book_id,),
            )
            results = cursor.fetchall()
            return [dict(row) for row in results]

        except Exception as e:
            raise NeonError(
                f"Failed to retrieve book chunks: {str(e)}",
                error_code="POSTGRES_RETRIEVE_FAILED",
                details={"book_id": book_id, "error": str(e)},
            )
        finally:
            if conn:
                self.connection_pool.putconn(conn)

    def delete_book_chunks(self, book_id: str) -> int:
        """
        Delete all chunks for a specific book.

        Args:
            book_id: Book identifier

        Returns:
            Number of chunks deleted

        Raises:
            NeonError: If operation fails
        """
        conn = None
        try:
            conn = self.connection_pool.getconn()
            cursor = conn.cursor()

            cursor.execute(
                """
                DELETE FROM chunks_metadata WHERE book_id = %s
                """,
                (book_id,),
            )
            conn.commit()
            deleted_count = cursor.rowcount

            logger.info(
                "book_chunks_deleted",
                book_id=book_id,
                count=deleted_count,
            )
            return deleted_count

        except Exception as e:
            if conn:
                conn.rollback()
            raise NeonError(
                f"Failed to delete book chunks: {str(e)}",
                error_code="POSTGRES_DELETE_FAILED",
                details={"book_id": book_id, "error": str(e)},
            )
        finally:
            if conn:
                self.connection_pool.putconn(conn)

    def get_chunk_count(self, book_id: str) -> int:
        """
        Get count of chunks for a book.

        Args:
            book_id: Book identifier

        Returns:
            Number of chunks

        Raises:
            NeonError: If query fails
        """
        conn = None
        try:
            conn = self.connection_pool.getconn()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT COUNT(*) FROM chunks_metadata WHERE book_id = %s
                """,
                (book_id,),
            )
            count = cursor.fetchone()[0]
            return count

        except Exception as e:
            raise NeonError(
                f"Failed to count book chunks: {str(e)}",
                error_code="POSTGRES_COUNT_FAILED",
                details={"book_id": book_id, "error": str(e)},
            )
        finally:
            if conn:
                self.connection_pool.putconn(conn)

    def health_check(self) -> bool:
        """
        Check if PostgreSQL is accessible.

        Returns:
            True if healthy, False otherwise
        """
        conn = None
        try:
            conn = self.connection_pool.getconn()
            cursor = conn.cursor()
            cursor.execute("SELECT 1")
            cursor.fetchone()
            logger.info("postgres_health_check_passed")
            return True
        except Exception as e:
            logger.warning("postgres_health_check_failed", error=str(e))
            return False
        finally:
            if conn:
                self.connection_pool.putconn(conn)

    def close(self):
        """Close all connections in pool."""
        try:
            self.connection_pool.closeall()
            logger.info("postgres_pool_closed")
        except Exception as e:
            logger.error("error_closing_postgres_pool", error=str(e))


# Singleton instance
_postgres_store: Optional[PostgresMetadataStore] = None


def get_postgres_store(database_url: str = None) -> Optional[PostgresMetadataStore]:
    """
    Get or create PostgreSQL metadata store instance.

    Args:
        database_url: PostgreSQL connection string (only needed for initialization)

    Returns:
        PostgresMetadataStore instance or None if database_url not provided
    """
    global _postgres_store
    if _postgres_store is None and database_url:
        _postgres_store = PostgresMetadataStore(database_url)
    return _postgres_store
