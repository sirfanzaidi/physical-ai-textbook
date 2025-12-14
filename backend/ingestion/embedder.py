"""
Cohere embedding integration for RAG Chatbot.

Generates embeddings using Cohere embed-v4.0 model.
"""

from typing import List, Dict, Any
import cohere
import structlog

from utils.errors import EmbeddingError

logger = structlog.get_logger(__name__)


class CohereEmbedder:
    """Generates embeddings using Cohere API."""

    def __init__(self, api_key: str):
        """
        Initialize Cohere embedder.

        Args:
            api_key: Cohere API key

        Raises:
            EmbeddingError: If initialization fails
        """
        try:
            self.client = cohere.ClientV2(api_key=api_key)
            self.model = "embed-v4.0"
            logger.info("cohere_embedder_initialized", model=self.model)
        except Exception as e:
            raise EmbeddingError(
                f"Failed to initialize Cohere client: {str(e)}",
                error_code="COHERE_INIT_FAILED",
                details={"error": str(e)},
            )

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors (1024-dimensional)

        Raises:
            EmbeddingError: If embedding fails
        """
        if not texts:
            return []

        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document",
            )

            embeddings = response.embeddings

            logger.info(
                "texts_embedded",
                model=self.model,
                count=len(texts),
                embedding_dim=len(embeddings[0]) if embeddings else 0,
            )
            return embeddings

        except Exception as e:
            raise EmbeddingError(
                f"Failed to generate embeddings: {str(e)}",
                error_code="EMBEDDING_FAILED",
                details={"text_count": len(texts), "error": str(e)},
            )

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a query.

        Args:
            query: Query text

        Returns:
            Embedding vector (1024-dimensional)

        Raises:
            EmbeddingError: If embedding fails
        """
        try:
            response = self.client.embed(
                texts=[query],
                model=self.model,
                input_type="search_query",
            )

            embedding = response.embeddings[0]

            logger.info(
                "query_embedded",
                model=self.model,
                embedding_dim=len(embedding),
            )
            return embedding

        except Exception as e:
            raise EmbeddingError(
                f"Failed to embed query: {str(e)}",
                error_code="QUERY_EMBEDDING_FAILED",
                details={"query_length": len(query), "error": str(e)},
            )

    def embed_chunks(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for a list of chunks.

        Updates chunks in-place with 'embedding' field.

        Args:
            chunks: List of chunk dicts with 'text' field

        Returns:
            Updated chunks with embeddings

        Raises:
            EmbeddingError: If embedding fails
        """
        if not chunks:
            return []

        # Extract texts in order
        texts = [chunk["text"] for chunk in chunks]

        try:
            embeddings = self.embed_texts(texts)

            # Attach embeddings to chunks
            for chunk, embedding in zip(chunks, embeddings):
                chunk["embedding"] = embedding

            logger.info(
                "chunks_embedded",
                count=len(chunks),
                avg_text_length=int(sum(len(t) for t in texts) / len(texts)) if texts else 0,
            )
            return chunks

        except EmbeddingError:
            raise
        except Exception as e:
            raise EmbeddingError(
                f"Failed to embed chunks: {str(e)}",
                error_code="CHUNK_EMBEDDING_FAILED",
                details={"chunk_count": len(chunks), "error": str(e)},
            )
