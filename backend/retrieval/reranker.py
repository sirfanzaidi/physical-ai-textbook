"""
Cohere-powered reranking for retrieval results.

Re-ranks retrieved chunks using Cohere rerank-v4.0-pro model.
"""

from typing import List, Dict, Any
import cohere
import structlog

from backend.utils.errors import RerankError

logger = structlog.get_logger(__name__)


class CohereReranker:
    """Re-ranks search results using Cohere API."""

    def __init__(self, api_key: str):
        """
        Initialize Cohere reranker.

        Args:
            api_key: Cohere API key

        Raises:
            RerankError: If initialization fails
        """
        try:
            self.client = cohere.ClientV2(api_key=api_key)
            self.model = "rerank-v4.0-pro"
            logger.info("cohere_reranker_initialized", model=self.model)
        except Exception as e:
            raise RerankError(
                f"Failed to initialize Cohere reranker: {str(e)}",
                error_code="RERANK_INIT_FAILED",
                details={"error": str(e)},
            )

    def rerank(
        self,
        query: str,
        documents: List[Dict[str, Any]],
        top_k: int = 8,
    ) -> List[Dict[str, Any]]:
        """
        Rerank retrieved documents by relevance to query.

        Args:
            query: User query text
            documents: List of retrieved document chunks
            top_k: Number of top results to return (default: 8)

        Returns:
            List of reranked documents (top_k only) with relevance scores

        Raises:
            RerankError: If reranking fails
        """
        if not documents:
            return []

        try:
            # Prepare documents for reranking API
            doc_texts = [doc.get("metadata", {}).get("text") or doc.get("text", "") for doc in documents]

            # Call Cohere rerank API
            response = self.client.rerank(
                model=self.model,
                query=query,
                documents=doc_texts,
                top_n=top_k,
            )

            # Map reranked results back to original documents
            reranked = []
            for result in response.results:
                original_doc = documents[result.index]
                reranked.append(
                    {
                        **original_doc,
                        "rerank_score": result.relevance_score,
                    }
                )

            logger.info(
                "documents_reranked",
                model=self.model,
                input_count=len(documents),
                output_count=len(reranked),
                top_k=top_k,
            )
            return reranked

        except Exception as e:
            raise RerankError(
                f"Failed to rerank documents: {str(e)}",
                error_code="RERANK_FAILED",
                details={
                    "document_count": len(documents),
                    "top_k": top_k,
                    "error": str(e),
                },
            )

    def rerank_with_threshold(
        self,
        query: str,
        documents: List[Dict[str, Any]],
        threshold: float = 0.5,
    ) -> List[Dict[str, Any]]:
        """
        Rerank documents and filter by relevance threshold.

        Args:
            query: User query text
            documents: List of retrieved documents
            threshold: Minimum relevance score (0-1)

        Returns:
            List of documents above threshold with rerank scores

        Raises:
            RerankError: If reranking fails
        """
        if not documents:
            return []

        try:
            doc_texts = [doc.get("metadata", {}).get("text") or doc.get("text", "") for doc in documents]

            response = self.client.rerank(
                model=self.model,
                query=query,
                documents=doc_texts,
                top_n=len(documents),  # Get all results for filtering
            )

            # Filter by threshold and map back
            reranked = []
            for result in response.results:
                if result.relevance_score >= threshold:
                    original_doc = documents[result.index]
                    reranked.append(
                        {
                            **original_doc,
                            "rerank_score": result.relevance_score,
                        }
                    )

            logger.info(
                "documents_reranked_with_threshold",
                model=self.model,
                input_count=len(documents),
                output_count=len(reranked),
                threshold=threshold,
            )
            return reranked

        except Exception as e:
            raise RerankError(
                f"Failed to rerank documents with threshold: {str(e)}",
                error_code="RERANK_FAILED",
                details={
                    "document_count": len(documents),
                    "threshold": threshold,
                    "error": str(e),
                },
            )
