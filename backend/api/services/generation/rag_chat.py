"""RAG chatbot implementation for orchestrating retrieval and generation."""

import time
from typing import List, Dict, Any, Optional, AsyncGenerator
from ...config import Settings
from ...utils import get_logger, GenerationError
from ..openrouter_client import OpenRouterClient
from ..retrieval.qdrant_store import QdrantStore

logger = get_logger(__name__)


class RAGChatbot:
    """RAG chatbot combining retrieval and generation."""

    def __init__(
        self,
        settings: Settings,
        openrouter_client: OpenRouterClient,
        qdrant_store: QdrantStore,
    ):
        """Initialize RAG chatbot.

        Args:
            settings: Application settings
            openrouter_client: OpenRouter API client
            qdrant_store: Qdrant vector store
        """
        self.settings = settings
        self.openrouter = openrouter_client
        self.qdrant = qdrant_store
        logger.info("RAGChatbot initialized")

    async def retrieve_context(
        self,
        query: str,
        top_k: int = 5,
        mode: str = "full",
        selected_text: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """Retrieve relevant context from vector store.

        Args:
            query: User query
            top_k: Number of chunks to retrieve
            mode: Query mode ('full' or 'selected')
            selected_text: Selected text for constrained retrieval

        Returns:
            List of relevant chunks with metadata

        Raises:
            GenerationError: If retrieval fails
        """
        try:
            # Generate embedding for query
            query_embedding = await self.openrouter.embed_text(query)

            # Search vector store
            results = await self.qdrant.search_with_mode(
                query_embedding=query_embedding,
                mode=mode,
                selected_text=selected_text,
                top_k=top_k,
            )

            logger.debug(f"Retrieved {len(results)} context chunks")

            # Validate results if in selected mode
            if mode == "selected" and not results:
                logger.warning("No chunks found for selected text mode")

            return results

        except Exception as e:
            logger.error(f"Context retrieval failed: {e}")
            raise GenerationError(f"Failed to retrieve context: {str(e)}")

    def build_prompt(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        mode: str = "full",
        selected_text: Optional[str] = None,
    ) -> str:
        """Build augmented prompt from query and context.

        Args:
            query: User query
            context_chunks: Retrieved context chunks
            mode: Query mode
            selected_text: Selected text if applicable

        Returns:
            Augmented prompt for LLM
        """
        # Build context section
        context_text = "\n\n".join(
            [
                f"Source: {chunk['payload'].get('chapter_name', 'Unknown')}\n"
                f"Content:\n{chunk['payload'].get('content', '')}"
                for chunk in context_chunks
            ]
        )

        # Build mode-specific instruction
        mode_instruction = ""
        if mode == "selected":
            mode_instruction = (
                f"\n\nIMPORTANT: The user highlighted this specific text:\n"
                f'"{selected_text}"\n\n'
                f"Please answer the question based ONLY on this highlighted text. "
                f"If the answer cannot be found in this text, say so explicitly."
            )

        # Construct full prompt
        prompt = (
            f"You are an educational assistant for the 'Physical AI & Humanoid Robotics' textbook. "
            f"Answer the user's question based ONLY on the provided context from the textbook. "
            f"If the answer is not in the context, say 'This information is not covered in the textbook.' "
            f"Always cite which chapter or section your answer comes from."
            f"{mode_instruction}\n\n"
            f"CONTEXT:\n{context_text}\n\n"
            f"USER QUESTION:\n{query}\n\n"
            f"ANSWER:"
        )

        return prompt

    async def generate_response_stream(
        self,
        prompt: str,
        temperature: float = 0.7,
        max_tokens: int = 1000,
    ) -> AsyncGenerator[str, None]:
        """Generate response with streaming.

        Args:
            prompt: Augmented prompt
            temperature: Sampling temperature
            max_tokens: Maximum response tokens

        Yields:
            Response chunks as generated

        Raises:
            GenerationError: If generation fails
        """
        try:
            messages = [
                {
                    "role": "system",
                    "content": "You are a helpful educational assistant.",
                },
                {
                    "role": "user",
                    "content": prompt,
                },
            ]

            async for chunk in self.openrouter.generate_completion_stream(
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens,
            ):
                yield chunk

        except Exception as e:
            logger.error(f"Response generation failed: {e}")
            raise GenerationError(f"Failed to generate response: {str(e)}")

    async def generate_response(
        self,
        prompt: str,
        temperature: float = 0.7,
        max_tokens: int = 1000,
    ) -> str:
        """Generate complete response (non-streaming).

        Args:
            prompt: Augmented prompt
            temperature: Sampling temperature
            max_tokens: Maximum response tokens

        Returns:
            Generated response text

        Raises:
            GenerationError: If generation fails
        """
        try:
            messages = [
                {
                    "role": "system",
                    "content": "You are a helpful educational assistant.",
                },
                {
                    "role": "user",
                    "content": prompt,
                },
            ]

            response = await self.openrouter.generate_completion(
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens,
            )

            return response

        except Exception as e:
            logger.error(f"Response generation failed: {e}")
            raise GenerationError(f"Failed to generate response: {str(e)}")

    async def answer_query(
        self,
        query: str,
        mode: str = "full",
        selected_text: Optional[str] = None,
        top_k: int = 5,
    ) -> Dict[str, Any]:
        """Answer query using RAG pipeline.

        Args:
            query: User query
            mode: Query mode
            selected_text: Selected text if applicable
            top_k: Number of context chunks

        Returns:
            Response with answer, citations, and metadata

        Raises:
            GenerationError: If any pipeline step fails
        """
        start_time = time.time()

        try:
            # Step 1: Retrieve context
            context_chunks = await self.retrieve_context(
                query=query,
                top_k=top_k,
                mode=mode,
                selected_text=selected_text,
            )

            # Step 2: Build prompt
            prompt = self.build_prompt(query, context_chunks, mode, selected_text)

            # Step 3: Generate response
            response_text = await self.generate_response(prompt)

            # Step 4: Extract citations
            citations = [
                {
                    "chunk_id": chunk["id"],
                    "chapter_name": chunk["payload"].get("chapter_name", "Unknown"),
                    "section_name": chunk["payload"].get("section_name"),
                    "source_url": chunk["payload"].get("source_url", ""),
                    "page_num": chunk["payload"].get("page_num"),
                    "score": chunk["score"],
                }
                for chunk in context_chunks
            ]

            latency_ms = (time.time() - start_time) * 1000

            result = {
                "response_text": response_text,
                "citations": citations,
                "sources": list(set(c["source_url"] for c in citations if c["source_url"])),
                "latency_ms": latency_ms,
                "model": self.settings.generation_model,
                "mode": mode,
            }

            logger.info(f"Query answered in {latency_ms:.0f}ms")
            return result

        except Exception as e:
            logger.error(f"Query answering failed: {e}")
            raise

    async def answer_query_stream(
        self,
        query: str,
        mode: str = "full",
        selected_text: Optional[str] = None,
        top_k: int = 5,
    ) -> AsyncGenerator[Dict[str, Any], None]:
        """Answer query with streaming response.

        Args:
            query: User query
            mode: Query mode
            selected_text: Selected text if applicable
            top_k: Number of context chunks

        Yields:
            Response chunks as Dict with type and content

        Raises:
            GenerationError: If any pipeline step fails
        """
        start_time = time.time()

        try:
            # Step 1: Retrieve context
            context_chunks = await self.retrieve_context(
                query=query,
                top_k=top_k,
                mode=mode,
                selected_text=selected_text,
            )

            # Yield citations first
            citations = [
                {
                    "chunk_id": chunk["id"],
                    "chapter_name": chunk["payload"].get("chapter_name", "Unknown"),
                    "section_name": chunk["payload"].get("section_name"),
                    "source_url": chunk["payload"].get("source_url", ""),
                    "page_num": chunk["payload"].get("page_num"),
                    "score": chunk["score"],
                }
                for chunk in context_chunks
            ]

            yield {
                "type": "citations",
                "citations": citations,
            }

            # Step 2: Build prompt
            prompt = self.build_prompt(query, context_chunks, mode, selected_text)

            # Step 3: Stream response
            async for chunk in self.generate_response_stream(prompt):
                yield {
                    "type": "text",
                    "content": chunk,
                }

            latency_ms = (time.time() - start_time) * 1000

            # Yield metadata
            yield {
                "type": "metadata",
                "latency_ms": latency_ms,
                "model": self.settings.generation_model,
                "mode": mode,
            }

        except Exception as e:
            logger.error(f"Streaming query failed: {e}")
            raise
