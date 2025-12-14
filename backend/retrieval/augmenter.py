"""
Document assembly and context augmentation for RAG.

Assembles retrieved chunks with citations for generation stage.
"""

from typing import List, Dict, Any
import structlog

from utils.errors import RetrievalError

logger = structlog.get_logger(__name__)


class DocumentAugmenter:
    """Assembles retrieved chunks into augmented context."""

    def __init__(self, separator: str = "\n\n---\n\n"):
        """
        Initialize document augmenter.

        Args:
            separator: Text separator between chunks in assembled context
        """
        self.separator = separator
        logger.info("document_augmenter_initialized", separator_length=len(separator))

    def augment(
        self,
        chunks: List[Dict[str, Any]],
        include_citations: bool = True,
    ) -> Dict[str, Any]:
        """
        Assemble retrieved chunks into augmented context.

        Args:
            chunks: List of reranked chunk dicts
            include_citations: Whether to extract citation info

        Returns:
            Dict with assembled context and citations

        Raises:
            RetrievalError: If augmentation fails
        """
        try:
            if not chunks:
                return {
                    "context": "",
                    "citations": [],
                    "chunk_count": 0,
                }

            # Assemble context text
            chunk_texts = []
            citations = []

            for i, chunk in enumerate(chunks):
                # Extract text (handle both formats)
                text = chunk.get("text") or chunk.get("metadata", {}).get("text", "")
                if text:
                    chunk_texts.append(text)

                    # Extract citation info if available
                    if include_citations:
                        citation = {
                            "index": i + 1,
                            "page_num": chunk.get("metadata", {}).get("page_num") or chunk.get("page_num"),
                            "section_name": chunk.get("metadata", {}).get("section_name") or chunk.get("section_name"),
                            "chapter_name": chunk.get("metadata", {}).get("chapter_name") or chunk.get("chapter_name"),
                            "rerank_score": chunk.get("rerank_score"),
                            "chunk_id": chunk.get("chunk_id") or chunk.get("metadata", {}).get("chunk_id"),
                        }
                        citations.append(citation)

            assembled_context = self.separator.join(chunk_texts)

            result = {
                "context": assembled_context,
                "citations": citations,
                "chunk_count": len(chunk_texts),
                "context_length": len(assembled_context),
            }

            logger.info(
                "documents_augmented",
                chunk_count=len(chunk_texts),
                context_length=len(assembled_context),
                citations_count=len(citations),
            )
            return result

        except Exception as e:
            raise RetrievalError(
                f"Failed to augment documents: {str(e)}",
                error_code="AUGMENTATION_FAILED",
                details={"chunk_count": len(chunks), "error": str(e)},
            )

    def assemble_select_text_context(
        self,
        selected_text: str,
        chunks: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """
        Assemble context for select-text queries (zero-leakage constraint).

        For select-text queries, only use the selected text passage, not
        retrieved chunks. This ensures no information leakage.

        Args:
            selected_text: User-selected text passage
            chunks: Retrieved chunks (for citations only, not context)

        Returns:
            Dict with selected text as context and citation metadata

        Raises:
            RetrievalError: If assembly fails
        """
        try:
            # For select-text, context is ONLY the selected text
            result = {
                "context": selected_text,
                "citations": [
                    {
                        "index": 1,
                        "page_num": None,
                        "section_name": "User Selected Text",
                        "chapter_name": None,
                        "rerank_score": 1.0,
                        "is_selected": True,
                    }
                ],
                "chunk_count": 1,
                "context_length": len(selected_text),
                "mode": "select_text",
                "zero_leakage": True,
            }

            logger.info(
                "select_text_context_assembled",
                context_length=len(selected_text),
            )
            return result

        except Exception as e:
            raise RetrievalError(
                f"Failed to assemble select-text context: {str(e)}",
                error_code="AUGMENTATION_FAILED",
                details={"selected_text_length": len(selected_text), "error": str(e)},
            )

    def format_context_for_generation(
        self,
        context: str,
        citations: List[Dict[str, Any]],
    ) -> str:
        """
        Format augmented context for generation prompt.

        Args:
            context: Assembled context text
            citations: Citation metadata list

        Returns:
            Formatted context string with citation markers
        """
        formatted = context

        if citations:
            formatted += "\n\n---\nSource Citations:\n"
            for citation in citations:
                citation_text = f"[{citation.get('index', 0)}] "
                if citation.get("chapter_name"):
                    citation_text += f"{citation['chapter_name']}"
                if citation.get("page_num"):
                    citation_text += f" (p. {citation['page_num']})"
                formatted += citation_text + "\n"

        return formatted
