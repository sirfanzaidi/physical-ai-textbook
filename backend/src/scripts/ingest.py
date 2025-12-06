"""Script to ingest and index chapters into ChromaDB.

Usage:
    python -m src.scripts.ingest --chapters 1 2 3 --mode delta
    python -m src.scripts.ingest --all --mode full
"""

import argparse
import json
import logging
import os
from pathlib import Path
from typing import List

from src.services import EmbeddingService, VectorDBService, ChunkingService

# Configure logging
logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO"),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def load_chapter(chapter_number: int, docs_path: str = "website/docs") -> tuple:
    """Load chapter content from markdown file.

    Args:
        chapter_number: Chapter number (1-6)
        docs_path: Path to documentation files

    Returns:
        Tuple of (title, content) or (None, None) if not found
    """
    # Look for chapter markdown file
    chapter_file = Path(docs_path) / f"{chapter_number:02d}-*.md"
    matching_files = list(Path(docs_path).glob(f"{chapter_number:02d}-*"))

    if not matching_files:
        logger.warning(f"Chapter {chapter_number} not found in {docs_path}")
        return None, None

    chapter_path = matching_files[0]
    try:
        with open(chapter_path, "r", encoding="utf-8") as f:
            content = f.read()

        # Extract title from first heading
        title = f"Chapter {chapter_number}"
        for line in content.split("\n"):
            if line.startswith("# "):
                title = line.replace("# ", "").strip()
                break

        return title, content
    except Exception as e:
        logger.error(f"Failed to load chapter {chapter_number}: {e}")
        return None, None


def ingest_chapters(
    chapters: List[int],
    embedding_service: EmbeddingService,
    vector_db_service: VectorDBService,
    chunking_service: ChunkingService,
    mode: str = "delta"
) -> dict:
    """Ingest chapters into ChromaDB.

    Args:
        chapters: List of chapter numbers to ingest
        embedding_service: Embedding service
        vector_db_service: Vector DB service
        chunking_service: Chunking service
        mode: Indexing mode ('delta' or 'full')

    Returns:
        Stats dict with results
    """
    logger.info(f"Starting chapter ingestion (mode={mode}, chapters={chapters})")

    all_chunks = []
    total_tokens = 0

    # Step 1: Load and chunk chapters
    logger.info(f"Loading and chunking {len(chapters)} chapters...")
    for chapter_num in chapters:
        title, content = load_chapter(chapter_num)
        if not content:
            logger.warning(f"Skipping chapter {chapter_num} (not found or error)")
            continue

        chunks = chunking_service.chunk_text(
            text=content,
            chapter_number=chapter_num,
            chapter_title=title or f"Chapter {chapter_num}",
        )
        logger.info(f"Chapter {chapter_num}: {len(chunks)} chunks ({sum(c['token_count'] for c in chunks)} tokens)")
        all_chunks.extend(chunks)
        total_tokens += sum(c["token_count"] for c in chunks)

    if not all_chunks:
        logger.error("No chunks to ingest!")
        return {"status": "failed", "message": "No chapters found to ingest"}

    # Step 2: Embed chunks
    logger.info(f"Embedding {len(all_chunks)} chunks...")
    documents = [chunk["content"] for chunk in all_chunks]
    embeddings = embedding_service.embed(documents)

    # Step 3: Prepare chunks with embeddings for ChromaDB
    prepared_chunks = []
    for chunk, embedding in zip(all_chunks, embeddings):
        prepared_chunks.append({
            "id": chunk["id"],
            "document": chunk["content"],
            "embedding": embedding,
            "metadata": chunk["metadata"],
        })

    # Step 4: Add to ChromaDB
    logger.info(f"Adding {len(prepared_chunks)} chunks to ChromaDB...")
    try:
        count = vector_db_service.add_chunks(prepared_chunks)
        logger.info(f"Successfully indexed {count} chunks")
    except Exception as e:
        logger.error(f"Failed to add chunks to ChromaDB: {e}")
        return {"status": "failed", "message": str(e)}

    # Step 5: Get stats
    db_stats = vector_db_service.get_db_stats()

    return {
        "status": "success",
        "chapters_processed": len(chapters),
        "total_chunks": len(prepared_chunks),
        "total_tokens": total_tokens,
        "db_stats": db_stats,
    }


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Ingest and index textbook chapters into ChromaDB"
    )
    parser.add_argument(
        "--chapters",
        type=int,
        nargs="+",
        help="Specific chapter numbers to ingest (1-6)"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Ingest all chapters (1-6)"
    )
    parser.add_argument(
        "--mode",
        choices=["delta", "full"],
        default="delta",
        help="Indexing mode (default: delta)"
    )

    args = parser.parse_args()

    # Determine which chapters to ingest
    if args.all:
        chapters = list(range(1, 7))
    elif args.chapters:
        chapters = sorted(set(args.chapters))
    else:
        parser.print_help()
        return

    # Initialize services
    embedding_service = EmbeddingService()
    vector_db_service = VectorDBService()
    chunking_service = ChunkingService()

    # Ingest chapters
    results = ingest_chapters(
        chapters=chapters,
        embedding_service=embedding_service,
        vector_db_service=vector_db_service,
        chunking_service=chunking_service,
        mode=args.mode,
    )

    # Print results
    print(json.dumps(results, indent=2, default=str))


if __name__ == "__main__":
    main()
