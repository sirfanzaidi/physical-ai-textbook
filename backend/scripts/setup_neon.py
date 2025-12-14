"""
Neon PostgreSQL metadata table initialization script.

Creates the 'chunks_metadata' table for tracking indexed book chunks.
"""

import os
import sys
import psycopg2
from psycopg2.extensions import ISOLATION_LEVEL_AUTOCOMMIT

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from app.config import get_settings


def setup_neon_tables():
    """Initialize metadata tables in Neon PostgreSQL."""
    settings = get_settings()

    if not settings.database_url:
        print("⚠ DATABASE_URL not configured, skipping Neon setup (optional)")
        return True

    try:
        # Connect to database
        conn = psycopg2.connect(settings.database_url)
        conn.set_isolation_level(ISOLATION_LEVEL_AUTOCOMMIT)
        cursor = conn.cursor()

        # Create chunks_metadata table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chunks_metadata (
                id SERIAL PRIMARY KEY,
                chunk_id VARCHAR(255) UNIQUE NOT NULL,
                book_id VARCHAR(255) NOT NULL,
                page_num INTEGER,
                section_name VARCHAR(255),
                chapter_name VARCHAR(255),
                text_hash VARCHAR(255),
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        """)

        # Create index on book_id for faster queries
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_chunks_book_id ON chunks_metadata(book_id);
        """)

        # Create index on chunk_id for faster lookup
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_chunks_chunk_id ON chunks_metadata(chunk_id);
        """)

        print("✓ Neon metadata tables created successfully")
        print("  - Table: chunks_metadata (id, chunk_id, book_id, page_num, section_name, chapter_name, text_hash)")
        print("  - Indexes: book_id, chunk_id")

        cursor.close()
        conn.close()
        return True

    except Exception as e:
        print(f"✗ Error setting up Neon tables: {e}")
        return False


if __name__ == "__main__":
    success = setup_neon_tables()
    sys.exit(0 if success else 1)
