"""
LLM Service for RAG Response Generation

Generates natural language responses from retrieved textbook chunks.
"""

import logging
from typing import List, Dict, Optional

logger = logging.getLogger(__name__)


class LLMService:
    """Service for generating responses from retrieved chunks."""

    def __init__(self, model_name: str = "template"):
        """
        Initialize LLM service.

        Args:
            model_name: Model to use ('template', 'openai', 'local')
        """
        self.model_name = model_name
        logger.info(f"LLMService initialized with model: {model_name}")

    def generate_response(
        self,
        query: str,
        chunks: List[Dict],
        max_length: int = 500
    ) -> str:
        """
        Generate a response to the query using retrieved chunks.

        Args:
            query: User's question
            chunks: Retrieved chunks with metadata
            max_length: Maximum response length in characters

        Returns:
            Generated response text
        """
        if not chunks:
            return "I don't have information about this in the Physical AI textbook. Please try asking about topics covered in the chapters: Physical AI fundamentals, humanoid robotics, ROS2, digital twins, vision-language-action models, or the capstone project."

        # Use template-based generation for now
        return self._template_generation(query, chunks, max_length)

    def _template_generation(
        self,
        query: str,
        chunks: List[Dict],
        max_length: int
    ) -> str:
        """
        Generate response using template-based approach.

        This is a simple but effective approach that:
        1. Identifies the most relevant chunk
        2. Extracts key information
        3. Formats it as a natural response
        4. Adds context from additional chunks if needed
        """
        # Get top chunk
        top_chunk = chunks[0] if chunks else {}
        content = top_chunk.get("content", "")
        metadata = top_chunk.get("metadata", {})

        chapter_title = metadata.get("chapter_title", "the textbook")
        chapter_number = metadata.get("chapter_number", "")

        # Build response based on query type
        response_parts = []

        # Add intro based on chapter context
        if chapter_number:
            response_parts.append(f"According to Chapter {chapter_number}: {chapter_title},")
        else:
            response_parts.append("Based on the textbook,")

        # Add main content from top chunk
        # Clean up the content to make it more conversational
        main_content = self._extract_relevant_sentence(query, content)
        if main_content:
            response_parts.append(main_content)

        # Add supporting details from additional chunks if available
        if len(chunks) > 1:
            for chunk in chunks[1:3]:  # Use up to 2 more chunks
                supporting_content = self._extract_relevant_sentence(
                    query, chunk.get("content", "")
                )
                if supporting_content and supporting_content != main_content:
                    response_parts.append(f"Additionally, {supporting_content}")

        response = " ".join(response_parts)

        # Truncate if too long
        if len(response) > max_length:
            response = response[:max_length].rsplit('. ', 1)[0] + '.'

        return response

    def _extract_relevant_sentence(self, query: str, content: str) -> str:
        """
        Extract the most relevant sentences from content for the query.

        Simple heuristic: find sentences containing query keywords.
        """
        if not content:
            return ""

        # Split into sentences
        sentences = [s.strip() for s in content.split('. ') if s.strip()]
        if not sentences:
            return content[:200] + "..."

        # Extract keywords from query (simple tokenization)
        query_words = set(query.lower().split())
        # Remove common stop words
        stop_words = {'what', 'is', 'are', 'the', 'a', 'an', 'how', 'why', 'when', 'where', 'who'}
        query_keywords = query_words - stop_words

        # Score sentences by keyword overlap
        scored_sentences = []
        for sentence in sentences:
            sentence_words = set(sentence.lower().split())
            overlap = len(query_keywords & sentence_words)
            if overlap > 0:
                scored_sentences.append((overlap, sentence))

        if scored_sentences:
            # Return sentence with highest overlap
            scored_sentences.sort(reverse=True, key=lambda x: x[0])
            best_sentence = scored_sentences[0][1]

            # Also include next sentence for context if available
            idx = sentences.index(best_sentence)
            if idx + 1 < len(sentences):
                return f"{best_sentence}. {sentences[idx + 1]}"
            return best_sentence

        # Fallback: return first few sentences
        return '. '.join(sentences[:2]) + '.'

    def _openai_generation(self, query: str, chunks: List[Dict], max_length: int) -> str:
        """
        Generate response using OpenAI API.

        TODO: Implement when OpenAI API key is available.
        """
        # This would use the OpenAI API to generate a response
        # For now, fall back to template generation
        logger.warning("OpenAI generation not yet implemented, using template")
        return self._template_generation(query, chunks, max_length)

    def _local_model_generation(self, query: str, chunks: List[Dict], max_length: int) -> str:
        """
        Generate response using local LLM (e.g., Llama, Mistral).

        TODO: Implement when local model is set up.
        """
        # This would use a local LLM to generate a response
        # For now, fall back to template generation
        logger.warning("Local model generation not yet implemented, using template")
        return self._template_generation(query, chunks, max_length)
