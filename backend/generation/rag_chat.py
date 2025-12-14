"""
RAG-powered chat interface using Cohere API.

Orchestrates the complete RAG pipeline: query embedding, retrieval, reranking,
context assembly, and response generation.
"""

from typing import Dict, Any, Optional, List
import cohere
import time
import structlog

from generation.prompts import get_system_prompt, get_user_prompt
from ingestion.embedder import CohereEmbedder
from retrieval.retriever import SemanticRetriever
from retrieval.reranker import CohereReranker
from retrieval.augmenter import DocumentAugmenter
from utils.errors import GenerationError, PromptError
from utils.logging import get_logger

logger = structlog.get_logger(__name__)


class RAGChatbot:
    """RAG-powered chatbot using Cohere API."""

    def __init__(
        self,
        api_key: str,
        embedder: CohereEmbedder,
        retriever: SemanticRetriever,
        reranker: CohereReranker,
        augmenter: DocumentAugmenter,
        generation_model: str = "command-a-03-2025",
    ):
        """
        Initialize RAG chatbot.

        Args:
            api_key: Cohere API key
            embedder: Initialized CohereEmbedder
            retriever: Initialized SemanticRetriever
            reranker: Initialized CohereReranker
            augmenter: Initialized DocumentAugmenter
            generation_model: Cohere generation model (default: command-a-03-2025)

        Raises:
            GenerationError: If initialization fails
        """
        try:
            self.client = cohere.ClientV2(api_key=api_key)
            self.model = generation_model
            self.embedder = embedder
            self.retriever = retriever
            self.reranker = reranker
            self.augmenter = augmenter
            logger.info(
                "rag_chatbot_initialized",
                model=self.model,
            )
        except Exception as e:
            raise GenerationError(
                f"Failed to initialize RAG chatbot: {str(e)}",
                error_code="CHATBOT_INIT_FAILED",
                details={"error": str(e)},
            )

    def chat(
        self,
        query: str,
        book_id: str,
        mode: str = "full",
        selected_text: Optional[str] = None,
        top_k_retrieve: int = 20,
        top_k_rerank: int = 8,
        confidence_threshold: float = 0.5,
    ) -> Dict[str, Any]:
        """
        Process a RAG query and generate response.

        Args:
            query: User query
            book_id: Book to query
            mode: Query mode ("full" or "selected")
            selected_text: Selected text (for "selected" mode)
            top_k_retrieve: Number of chunks to retrieve
            top_k_rerank: Number of chunks after reranking
            confidence_threshold: Minimum confidence to return response

        Returns:
            Dict with response, citations, confidence, and latency

        Raises:
            GenerationError: If generation fails
        """
        start_time = time.time()
        retrieval_start = time.time()

        try:
            # Step 1: Embed query
            query_embedding = self.embedder.embed_query(query)
            embedding_time = (time.time() - retrieval_start) * 1000

            # Step 2: Retrieve similar chunks
            chunks = self.retriever.retrieve(
                query_embedding=query_embedding,
                book_id=book_id,
                top_k=top_k_retrieve,
            )

            if not chunks:
                return {
                    "response": "No relevant information found in the selected book for your query.",
                    "citations": [],
                    "confidence": 0.0,
                    "latency_ms": (time.time() - start_time) * 1000,
                    "retrieval_latency_ms": embedding_time,
                    "generation_latency_ms": 0,
                }

            # Step 3: Rerank retrieved chunks
            reranked_chunks = self.reranker.rerank(
                query=query,
                documents=chunks,
                top_k=top_k_rerank,
            )
            retrieval_latency = (time.time() - retrieval_start) * 1000

            # Step 4: Assemble context
            if mode == "selected":
                if not selected_text:
                    raise PromptError(
                        "Selected text required for 'selected' mode",
                        error_code="MISSING_SELECTED_TEXT",
                    )
                context_data = self.augmenter.assemble_select_text_context(
                    selected_text=selected_text,
                    chunks=reranked_chunks,
                )
            else:  # "full"
                context_data = self.augmenter.augment(chunks=reranked_chunks)

            context = context_data["context"]
            citations = context_data["citations"]

            # Step 5: Generate response
            generation_start = time.time()

            system_prompt = get_system_prompt(mode=("select_text" if mode == "selected" else "rag"))
            user_prompt = get_user_prompt(
                mode=("select_text" if mode == "selected" else "rag"),
                context=context,
                selected_text=selected_text or "",
                query=query,
            )

            response_obj = self.client.chat(
                model=self.model,
                system=[{"type": "text", "text": system_prompt}],
                messages=[{"role": "user", "content": user_prompt}],
                temperature=0.3,  # Low temperature for consistency
                max_tokens=500,  # Reasonable response length
            )

            response_text = response_obj.message.content[0].text
            generation_latency = (time.time() - generation_start) * 1000

            # Step 6: Extract confidence (heuristic for now)
            # In production, could use Cohere's reranker scores or separate model
            confidence = self._estimate_confidence(
                response=response_text,
                rerank_scores=[c.get("rerank_score", 0.5) for c in reranked_chunks],
                chunk_count=len(reranked_chunks),
            )

            total_latency = (time.time() - start_time) * 1000

            logger.info(
                "query_processed",
                book_id=book_id,
                mode=mode,
                chunks_retrieved=len(chunks),
                chunks_reranked=len(reranked_chunks),
                confidence=round(confidence, 3),
                total_latency_ms=round(total_latency, 2),
            )

            return {
                "response": response_text,
                "citations": citations,
                "confidence": round(confidence, 3),
                "latency_ms": round(total_latency, 2),
                "retrieval_latency_ms": round(retrieval_latency, 2),
                "generation_latency_ms": round(generation_latency, 2),
            }

        except (PromptError, GenerationError):
            raise
        except Exception as e:
            raise GenerationError(
                f"Failed to generate response: {str(e)}",
                error_code="GENERATION_FAILED",
                details={
                    "query_length": len(query),
                    "book_id": book_id,
                    "error": str(e),
                },
            )

    def _estimate_confidence(
        self,
        response: str,
        rerank_scores: List[float],
        chunk_count: int,
    ) -> float:
        """
        Estimate confidence of generated response.

        Heuristic-based confidence combining:
        - Average rerank score of retrieved chunks
        - Number of chunks available
        - Response characteristics (length, completeness)

        Args:
            response: Generated response text
            rerank_scores: Reranking scores of source chunks
            chunk_count: Number of chunks used

        Returns:
            Confidence score (0-1)
        """
        if not rerank_scores or chunk_count == 0:
            return 0.3

        # Average rerank score provides signal
        avg_rerank = sum(rerank_scores) / len(rerank_scores)

        # More chunks generally better
        chunk_factor = min(chunk_count / 5, 1.0)  # Max benefit at 5+ chunks

        # Response length (some correlation with quality)
        response_factor = min(len(response) / 200, 1.0)  # Max benefit at 200+ chars

        # Combine factors
        confidence = (avg_rerank * 0.6 + chunk_factor * 0.2 + response_factor * 0.2)
        confidence = max(0.0, min(1.0, confidence))  # Clamp to [0, 1]

        return confidence
