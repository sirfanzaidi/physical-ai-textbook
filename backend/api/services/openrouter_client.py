"""OpenRouter API client for embeddings and completions."""

import asyncio
from typing import List, Optional, AsyncGenerator
from openai import AsyncOpenAI
from ..config import Settings
from ..utils import get_logger, OpenRouterError

logger = get_logger(__name__)


class OpenRouterClient:
    """Client for OpenRouter API (embeddings + completions)."""

    def __init__(self, settings: Settings):
        """Initialize OpenRouter client.

        Args:
            settings: Application settings with API key and model names

        Raises:
            OpenRouterError: If API key is not configured
        """
        if not settings.openrouter_api_key:
            raise OpenRouterError("OPENROUTER_API_KEY not configured in environment")

        self.settings = settings
        self.client = AsyncOpenAI(
            api_key=settings.openrouter_api_key,
            base_url=settings.openrouter_base_url,
        )
        logger.info(f"OpenRouter client initialized with model: {settings.generation_model}")

    async def embed_text(self, text: str) -> List[float]:
        """Generate embedding for text using OpenRouter.

        Args:
            text: Text to embed

        Returns:
            Embedding vector

        Raises:
            OpenRouterError: If embedding fails
        """
        try:
            response = await self.client.embeddings.create(
                model=self.settings.embedding_model,
                input=text,
            )
            embedding = response.data[0].embedding
            logger.debug(f"Generated embedding with {len(embedding)} dimensions")
            return embedding
        except Exception as e:
            logger.error(f"Embedding generation failed: {e}")
            raise OpenRouterError(f"Failed to generate embedding: {str(e)}")

    async def embed_texts_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple texts.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors

        Raises:
            OpenRouterError: If embedding fails
        """
        try:
            response = await self.client.embeddings.create(
                model=self.settings.embedding_model,
                input=texts,
            )
            embeddings = [item.embedding for item in response.data]
            logger.debug(f"Generated {len(embeddings)} embeddings")
            return embeddings
        except Exception as e:
            logger.error(f"Batch embedding generation failed: {e}")
            raise OpenRouterError(f"Failed to generate batch embeddings: {str(e)}")

    async def generate_completion(
        self,
        messages: List[dict],
        temperature: float = 0.7,
        max_tokens: int = 1000,
    ) -> str:
        """Generate text completion using OpenRouter.

        Args:
            messages: List of message dicts with 'role' and 'content'
            temperature: Sampling temperature (0-2)
            max_tokens: Maximum tokens in response

        Returns:
            Generated completion text

        Raises:
            OpenRouterError: If generation fails
        """
        try:
            response = await self.client.chat.completions.create(
                model=self.settings.generation_model,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens,
            )
            completion = response.choices[0].message.content
            logger.debug(f"Generated completion: {len(completion)} chars")
            return completion
        except Exception as e:
            logger.error(f"Completion generation failed: {e}")
            raise OpenRouterError(f"Failed to generate completion: {str(e)}")

    async def generate_completion_stream(
        self,
        messages: List[dict],
        temperature: float = 0.7,
        max_tokens: int = 1000,
    ) -> AsyncGenerator[str, None]:
        """Generate text completion with streaming using OpenRouter.

        Args:
            messages: List of message dicts with 'role' and 'content'
            temperature: Sampling temperature (0-2)
            max_tokens: Maximum tokens in response

        Yields:
            Text chunks as they are generated

        Raises:
            OpenRouterError: If generation fails
        """
        try:
            stream = await self.client.chat.completions.create(
                model=self.settings.generation_model,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens,
                stream=True,
            )

            async for chunk in stream:
                if chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content
        except Exception as e:
            logger.error(f"Streaming completion generation failed: {e}")
            raise OpenRouterError(f"Failed to generate streaming completion: {str(e)}")
