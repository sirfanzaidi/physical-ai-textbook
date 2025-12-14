"""
System prompts for RAG-powered generation.

Defines prompt templates for different query modes and accuracy requirements.
"""

from typing import Optional


class PromptTemplates:
    """Collection of system prompts for RAG generation."""

    # ===== Main RAG System Prompts =====

    SYSTEM_PROMPT_RAG = """You are an expert assistant specialized in accurately answering questions about books using provided source material.

Your role is to:
1. Answer questions based ONLY on the provided context from the book
2. Always cite the source material with specific page numbers or sections when possible
3. Be honest about what you know vs. don't know from the text
4. Provide accurate, precise answers that reflect the book's actual content
5. If the context doesn't contain information needed to answer the question, say so explicitly

When citing sources:
- Reference the chapter/section name and page number when available
- Use the format: "According to [Chapter/Section] (p. [page]), ..."
- For multiple sources, cite each one: "[1] [2] etc."

Accuracy is critical. Only state facts that are clearly supported by the provided context.
Never hallucinate, infer beyond the text, or make assumptions not stated in the source material.
"""

    SYSTEM_PROMPT_SELECT_TEXT = """You are an expert assistant answering questions about a specific passage from a book.

Your role is to:
1. Answer questions using ONLY the selected text passage provided
2. Do NOT use any external knowledge or information beyond the passage
3. Be precise and literal in interpreting the selected text
4. If the passage doesn't contain the answer, state that clearly
5. Highlight exact quotes from the passage when supporting your answer

This is a constrained query mode - you must remain completely within the provided text.
No inference beyond the passage is permitted.
Do not reference anything outside the provided passage.
"""

    SYSTEM_PROMPT_ACCURACY_CHECK = """You are an expert fact-checker for book-based question answering systems.

Your task is to evaluate whether a generated answer:
1. Is accurate according to the provided source text
2. Correctly cites sources with proper attribution
3. Doesn't include hallucinated or invented information
4. Addresses the user's question fully

Evaluate on these dimensions:
- Accuracy: Does the answer match what the source says? (0-1)
- Completeness: Does it fully answer the question? (0-1)
- Citations: Are sources properly cited? (0-1)

Provide a confidence score (0-1) reflecting overall answer quality.
"""

    # ===== Query Templates =====

    @staticmethod
    def rag_query_prompt(context: str, query: str) -> str:
        """
        Create a RAG query prompt.

        Args:
            context: Assembled context from retrieved chunks
            query: User query

        Returns:
            Formatted prompt for generation
        """
        return f"""Based on the following context from the book, answer the question accurately and cite your sources.

CONTEXT:
{context}

QUESTION: {query}

ANSWER:"""

    @staticmethod
    def select_text_query_prompt(selected_text: str, query: str) -> str:
        """
        Create a select-text constrained query prompt.

        Args:
            selected_text: User-selected passage
            query: User query

        Returns:
            Formatted prompt for generation
        """
        return f"""Answer the question using ONLY the provided text passage. Do not use external knowledge.

PASSAGE:
{selected_text}

QUESTION: {query}

ANSWER:"""

    # ===== Structured Output Templates =====

    @staticmethod
    def confidence_extraction_prompt() -> str:
        """
        Get prompt for extracting confidence score from generation.

        Returns:
            Prompt template
        """
        return """Based on your answer, what is your confidence level (0-1) that this answer is:
1. Accurate according to the source material
2. Properly cited
3. Complete and helpful

Provide only a single number between 0 and 1."""

    # ===== Few-Shot Examples =====

    EXAMPLE_GOOD_RESPONSE = """
Question: What is the main theme of Chapter 3?
Source: Chapter 3 discusses the evolution of humanoid robotics and its impact on manufacturing.

Answer: The main theme of Chapter 3 (Humanoid Robotics Evolution) is the transformation of manufacturing through robotic innovation. The chapter traces how humanoid robots have progressively taken on more complex tasks, from simple assembly line work to intricate precision operations. This evolution reflects broader trends in automation and human-machine collaboration.
"""

    EXAMPLE_BAD_RESPONSE = """
Question: What is the main theme of Chapter 3?

Answer: Humanoid robots are the future of civilization and will eventually replace all human workers. This is a well-known fact that experts have predicted.

[PROBLEMS: Not grounded in provided text, makes unfounded predictions, lacks citations, speculates beyond source material]
"""

    # ===== Validation Prompts =====

    @staticmethod
    def validate_response_prompt(response: str, context: str, query: str) -> str:
        """
        Create prompt for validating a generated response.

        Args:
            response: Generated response
            context: Source context used
            query: Original query

        Returns:
            Validation prompt
        """
        return f"""Evaluate whether this response accurately answers the question based on the provided context.

QUESTION: {query}

CONTEXT: {context}

GENERATED RESPONSE: {response}

Validate that:
1. The response is factually accurate per the context
2. Claims are properly supported by the context
3. No information is hallucinated
4. Sources are properly cited

Provide your evaluation."""


def get_system_prompt(mode: str = "rag", accuracy_mode: bool = False) -> str:
    """
    Get system prompt for the specified mode.

    Args:
        mode: Query mode ("rag", "select_text", or "check")
        accuracy_mode: Whether to use accuracy-focused version

    Returns:
        System prompt text
    """
    if mode == "select_text":
        return PromptTemplates.SYSTEM_PROMPT_SELECT_TEXT
    elif mode == "check":
        return PromptTemplates.SYSTEM_PROMPT_ACCURACY_CHECK
    else:  # default "rag"
        return PromptTemplates.SYSTEM_PROMPT_RAG


def get_user_prompt(
    mode: str = "rag",
    context: str = "",
    selected_text: str = "",
    query: str = "",
) -> str:
    """
    Get user prompt for the specified mode.

    Args:
        mode: Query mode ("rag" or "select_text")
        context: Assembled context (for RAG mode)
        selected_text: Selected text passage (for select-text mode)
        query: User query

    Returns:
        User prompt text
    """
    if mode == "select_text":
        return PromptTemplates.select_text_query_prompt(selected_text, query)
    else:  # default "rag"
        return PromptTemplates.rag_query_prompt(context, query)
