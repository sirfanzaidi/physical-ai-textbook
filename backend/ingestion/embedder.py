"""
Cohere embedding integration for RAG Chatbot.

Generates embeddings using Cohere embed-v4.0 model.
"""

from typing import List, Dict, Any
import cohere
import structlog

from backend.utils.errors import EmbeddingError

logger = structlog.get_logger(__name__)


class CohereEmbedder:
    """Generates embeddings using Cohere API.

    Uses embed-v4.0 model which produces 1536-dimensional embeddings.
    """

    def __init__(self, api_key: str):
        """
        Initialize Cohere embedder with embed-v4.0 model (1536-dim).
        """
        try:
            self.client = cohere.ClientV2(api_key=api_key)
            self.model = "embed-v4.0"
            logger.info("cohere_embedder_initialized", model=self.model, dimension=1536)
        except Exception as e:
            raise EmbeddingError(
                f"Failed to initialize Cohere client: {str(e)}",
                error_code="COHERE_INIT_FAILED",
                details={"error": str(e)},
            )

    # ---------------------------
    # INTERNAL HELPER (NEW)
    # ---------------------------
    def _extract_embeddings(self, response) -> List[List[float]]:
      """
      Safely extract embeddings from Cohere API response.
      Handles EmbedByTypeResponseEmbeddings which returns tuples when iterating.
      Format: When iterating over embeddings object, yields tuples like ('float_', [[vectors...]])
      """
      if not hasattr(response, "embeddings"):
          return []

      emb = response.embeddings

      # Case 1: Direct list of vectors
      if isinstance(emb, list):
          return emb if emb else []

      # Case 2: EmbedByTypeResponseEmbeddings object - must iterate to extract data
      # This returns tuples when iterated: ('float_', [[embedding vectors...]])
      if hasattr(emb, "__iter__") and not isinstance(emb, str):
          result = []
          try:
              for item in emb:
                  # Extract from tuple format
                  if isinstance(item, tuple) and len(item) >= 2:
                      data = item[1]  # Get second element: the embedding vectors
                      if isinstance(data, list):
                          result.extend(data)  # Add all vectors to result
                  # Handle direct list (fallback)
                  elif isinstance(item, list):
                      result.append(item)
              return result if result else []
          except Exception:
              pass

      # Case 3: Object with .data attribute
      if hasattr(emb, "data"):
          try:
              data = emb.data
              if isinstance(data, list):
                  return data
              return list(data) if hasattr(data, "__iter__") else []
          except Exception:
              pass

      return []

    # ---------------------------
    # FIX 1: embed_texts
    # ---------------------------
    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.
        """
        if not texts:
            return []

        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document",
            )

            embeddings = self._extract_embeddings(response)

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

    # ---------------------------
    # FIX 2: embed_query
    # ---------------------------
    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a query.
        """
        try:
            response = self.client.embed(
                texts=[query],
                model=self.model,
                input_type="search_query",
            )

            embeddings = self._extract_embeddings(response)
            embedding = embeddings[0] if embeddings else []

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

    # ---------------------------
    # embed_chunks (UNCHANGED)
    # ---------------------------
    def embed_chunks(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for a list of chunks.
        """
        if not chunks:
            return []

        texts = [chunk["text"] for chunk in chunks]

        try:
            embeddings = self.embed_texts(texts)

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
