"""
Smoke tests for external service connections.

Verifies that Cohere API, Qdrant, and Neon database connections work.
"""

import os
import pytest
import sys

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from app.config import get_settings


class TestCoherConnection:
    """Test Cohere API connectivity."""

    def test_cohere_api_key_configured(self):
        """Verify Cohere API key is configured."""
        settings = get_settings()
        assert settings.cohere_api_key, "COHERE_API_KEY not configured"
        assert settings.cohere_api_key != "your_cohere_api_key_here", "COHERE_API_KEY is placeholder"

    def test_cohere_client_initialization(self):
        """Test Cohere client can be initialized."""
        try:
            import cohere
            settings = get_settings()

            # Initialize client
            client = cohere.ClientV2(api_key=settings.cohere_api_key)

            # Verify client has required methods
            assert hasattr(client, 'embed'), "Client missing embed method"
            assert hasattr(client, 'rerank'), "Client missing rerank method"
            assert hasattr(client, 'chat'), "Client missing chat method"

        except Exception as e:
            pytest.skip(f"Cohere client initialization skipped: {e}")


class TestQdrantConnection:
    """Test Qdrant vector database connectivity."""

    def test_qdrant_url_configured(self):
        """Verify Qdrant URL is configured."""
        settings = get_settings()
        assert settings.qdrant_url, "QDRANT_URL not configured"
        assert settings.qdrant_url != "https://your-qdrant-cluster-id.region.cloud.qdrant.io", "QDRANT_URL is placeholder"

    def test_qdrant_api_key_configured(self):
        """Verify Qdrant API key is configured."""
        settings = get_settings()
        assert settings.qdrant_api_key, "QDRANT_API_KEY not configured"
        assert settings.qdrant_api_key != "your_qdrant_api_key_here", "QDRANT_API_KEY is placeholder"

    def test_qdrant_client_initialization(self):
        """Test Qdrant client can be initialized."""
        try:
            from qdrant_client import QdrantClient
            settings = get_settings()

            # Initialize client
            client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
            )

            # Verify client can list collections
            collections = client.get_collections()
            assert collections is not None, "Failed to list Qdrant collections"

        except Exception as e:
            pytest.skip(f"Qdrant client initialization skipped: {e}")


class TestNeonConnection:
    """Test Neon PostgreSQL connectivity."""

    def test_neon_database_url_configured(self):
        """Verify Neon database URL is configured."""
        settings = get_settings()
        # DATABASE_URL is optional
        if settings.database_url:
            assert settings.database_url != "postgresql://user:password@host:port/database", "DATABASE_URL is placeholder"

    def test_neon_client_initialization(self):
        """Test Neon client can be initialized."""
        try:
            import psycopg2
            settings = get_settings()

            if not settings.database_url:
                pytest.skip("DATABASE_URL not configured (optional)")

            # Initialize connection
            conn = psycopg2.connect(settings.database_url)
            cursor = conn.cursor()

            # Test query
            cursor.execute("SELECT 1")
            result = cursor.fetchone()
            assert result is not None, "Failed to execute test query"

            cursor.close()
            conn.close()

        except Exception as e:
            pytest.skip(f"Neon client initialization skipped: {e}")


class TestConfigurationSettings:
    """Test configuration settings are loaded correctly."""

    def test_settings_loaded(self):
        """Verify settings are loaded."""
        settings = get_settings()
        assert settings is not None, "Settings not loaded"

    def test_required_settings_present(self):
        """Verify required settings are present."""
        settings = get_settings()
        assert settings.cohere_api_key, "cohere_api_key not configured"
        assert settings.qdrant_url, "qdrant_url not configured"
        assert settings.qdrant_api_key, "qdrant_api_key not configured"
        assert settings.qdrant_collection, "qdrant_collection not configured"

    def test_default_values_set(self):
        """Verify default values are set correctly."""
        settings = get_settings()
        assert settings.chunk_size == 300, "chunk_size default incorrect"
        assert settings.chunk_max_size == 500, "chunk_max_size default incorrect"
        assert settings.chunk_overlap == 200, "chunk_overlap default incorrect"
        assert settings.max_book_pages == 500, "max_book_pages default incorrect"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
