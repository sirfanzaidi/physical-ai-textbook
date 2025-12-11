"""Tests for the RAG Chatbot API."""

import pytest
from fastapi.testclient import TestClient
from api.app import app

# Create a test client
client = TestClient(app)


class TestHealthEndpoint:
    """Health check endpoint tests."""

    def test_health_endpoint_success(self):
        """Test health endpoint returns valid response."""
        response = client.get("/api/health")
        assert response.status_code == 200

        data = response.json()
        assert "status" in data
        assert "qdrant_connected" in data
        assert "collection_exists" in data
        assert "collection_count" in data
        assert "cohere_available" in data

        # Status should be one of the valid values
        assert data["status"] in ["healthy", "degraded", "unhealthy"]


class TestChatEndpoint:
    """Chat endpoint tests."""

    def test_chat_endpoint_success(self):
        """Test chat endpoint with valid question."""
        response = client.post(
            "/api/chat",
            json={"question": "What is physical AI?"}
        )
        assert response.status_code == 200

        data = response.json()
        assert "answer" in data
        assert "sources" in data
        assert "retrieved_chunks_count" in data

        assert isinstance(data["answer"], str)
        assert len(data["answer"]) > 0
        assert isinstance(data["sources"], list)
        assert isinstance(data["retrieved_chunks_count"], int)

    def test_chat_endpoint_with_options(self):
        """Test chat endpoint with all optional fields."""
        response = client.post(
            "/api/chat",
            json={
                "question": "Explain humanoid robotics",
                "selected_text": "Some selected text",
                "chapter_filter": 2
            }
        )
        assert response.status_code == 200
        data = response.json()
        assert "answer" in data

    def test_chat_endpoint_short_question(self):
        """Test chat endpoint with too short question."""
        response = client.post(
            "/api/chat",
            json={"question": "Hi"}  # Only 2 characters, min is 3
        )
        assert response.status_code == 422  # Validation error

    def test_chat_endpoint_long_question(self):
        """Test chat endpoint with too long question."""
        response = client.post(
            "/api/chat",
            json={"question": "x" * 501}  # Max is 500
        )
        assert response.status_code == 422  # Validation error

    def test_chat_endpoint_empty_question(self):
        """Test chat endpoint with empty question."""
        response = client.post(
            "/api/chat",
            json={"question": ""}
        )
        assert response.status_code == 422  # Validation error

    def test_chat_endpoint_invalid_chapter_filter(self):
        """Test chat endpoint with invalid chapter filter."""
        response = client.post(
            "/api/chat",
            json={
                "question": "What is a robot?",
                "chapter_filter": 10  # Only 1-6 are valid
            }
        )
        assert response.status_code == 422  # Validation error


class TestRootEndpoint:
    """Root endpoint tests."""

    def test_root_endpoint(self):
        """Test root endpoint returns info."""
        response = client.get("/")
        assert response.status_code == 200

        data = response.json()
        assert "message" in data
        assert "docs" in data
        assert "health" in data


class TestAPIDocs:
    """API documentation tests."""

    def test_swagger_docs_available(self):
        """Test Swagger documentation is available."""
        response = client.get("/api/docs")
        assert response.status_code == 200
        assert "swagger" in response.text.lower() or "openapi" in response.text.lower()

    def test_redoc_docs_available(self):
        """Test ReDoc documentation is available."""
        response = client.get("/api/redoc")
        assert response.status_code == 200
        assert "redoc" in response.text.lower() or "openapi" in response.text.lower()
