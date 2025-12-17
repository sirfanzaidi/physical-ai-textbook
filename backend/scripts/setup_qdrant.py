"""
Qdrant collection initialization script.

Creates the 'book_vectors' collection with proper configuration for RAG chatbot.
"""

import os
import sys
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from app.config import get_settings


def setup_qdrant_collection():
    """Initialize Qdrant collection for book embeddings."""
    settings = get_settings()

    try:
        # Initialize Qdrant client
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

        # Check if collection already exists
        try:
            client.get_collection(settings.qdrant_collection)
            print(f"✓ Collection '{settings.qdrant_collection}' already exists")
            return True
        except Exception:
            # Collection doesn't exist, create it
            print(f"Creating collection '{settings.qdrant_collection}'...")

        # Create collection with proper configuration
        client.create_collection(
            collection_name=settings.qdrant_collection,
            vectors_config=VectorParams(
                size=1024,  # embed-v4.0 default dimension
                distance=Distance.COSINE,
            ),
            # Payload schema not required but helpful for documentation
            # payload_schema provides type hints for payload fields
        )

        print(f"✓ Collection '{settings.qdrant_collection}' created successfully")
        print("  - Vector dimension: 1024 (Cohere embed-v4.0)")
        print("  - Distance metric: Cosine")

        return True

    except Exception as e:
        print(f"✗ Error setting up Qdrant collection: {e}")
        return False


if __name__ == "__main__":
    success = setup_qdrant_collection()
    sys.exit(0 if success else 1)
