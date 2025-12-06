"""Embedding service for converting text to vectors using sentence-transformers."""

import os
import logging
from typing import List, Optional
import time

logger = logging.getLogger(__name__)


class EmbeddingService:
    """Service for generating embeddings using sentence-transformers."""

    def __init__(
        self,
        model_name: str = "sentence-transformers/all-MiniLM-L6-v2",
        batch_size: int = 32,
        device: str = "cpu"
    ):
        """Initialize embedding service.

        Args:
            model_name: HuggingFace model identifier (default: all-MiniLM-L6-v2, 384-dims)
            batch_size: Number of texts to embed at once (default: 32)
            device: Device to run on ('cpu', 'cuda', 'mps')
        """
        self.model_name = model_name or os.getenv("EMBEDDING_MODEL", "sentence-transformers/all-MiniLM-L6-v2")
        self.batch_size = int(os.getenv("EMBEDDING_BATCH_SIZE", batch_size))
        self.device = device

        self._model = None
        self._is_loaded = False
        logger.info(f"EmbeddingService initialized (model={self.model_name}, batch_size={self.batch_size})")

    def _ensure_loaded(self) -> None:
        """Lazy load the embedding model on first use."""
        if self._is_loaded:
            return

        try:
            from sentence_transformers import SentenceTransformer
            logger.info(f"Loading embedding model: {self.model_name}")
            start_time = time.time()
            self._model = SentenceTransformer(self.model_name, device=self.device)
            load_time = time.time() - start_time
            logger.info(f"Embedding model loaded in {load_time:.2f}s")
            self._is_loaded = True
        except ImportError:
            raise ImportError(
                "sentence-transformers not installed. Install with: pip install sentence-transformers"
            )
        except Exception as e:
            logger.error(f"Failed to load embedding model: {e}")
            raise

    def embed(self, texts: List[str]) -> List[List[float]]:
        """Embed a batch of texts.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors (List[float])

        Raises:
            ValueError: If texts is empty
            RuntimeError: If embedding fails
        """
        if not texts:
            raise ValueError("texts cannot be empty")

        self._ensure_loaded()

        try:
            start_time = time.time()
            # Encode returns numpy array, convert to list for JSON serialization
            embeddings = self._model.encode(texts, batch_size=self.batch_size)
            embeddings_list = [embedding.tolist() for embedding in embeddings]
            elapsed = time.time() - start_time
            logger.debug(f"Embedded {len(texts)} texts in {elapsed:.2f}s ({elapsed/len(texts)*1000:.1f}ms per text)")
            return embeddings_list
        except Exception as e:
            logger.error(f"Embedding failed: {e}")
            raise RuntimeError(f"Failed to embed texts: {e}")

    def embed_single(self, text: str) -> List[float]:
        """Embed a single text string.

        Args:
            text: Text string to embed

        Returns:
            Single embedding vector (List[float])
        """
        return self.embed([text])[0]

    def get_embedding_dimension(self) -> int:
        """Get the dimensionality of embeddings.

        Returns:
            Number of dimensions (e.g., 384 for all-MiniLM-L6-v2)
        """
        self._ensure_loaded()
        return self._model.get_sentence_embedding_dimension()

    def get_model_info(self) -> dict:
        """Get information about the loaded model.

        Returns:
            Dict with model name and dimensions
        """
        self._ensure_loaded()
        return {
            "model_name": self.model_name,
            "dimensions": self.get_embedding_dimension(),
            "batch_size": self.batch_size,
            "device": self.device,
        }
