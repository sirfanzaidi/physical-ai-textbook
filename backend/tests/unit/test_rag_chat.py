"""
Unit tests for RAG chat generation.

Tests system prompt enforcement and response generation logic.
"""

import pytest
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from generation.prompts import get_system_prompt, get_user_prompt, PromptTemplates
from utils.errors import GenerationError


class TestPromptTemplates:
    """Test prompt template functionality."""

    def test_rag_system_prompt_exists(self):
        """Test RAG system prompt is defined."""
        prompt = PromptTemplates.SYSTEM_PROMPT_RAG
        assert prompt is not None
        assert len(prompt) > 0
        assert "ground" in prompt.lower() or "context" in prompt.lower()

    def test_select_text_system_prompt_exists(self):
        """Test select-text system prompt is defined."""
        prompt = PromptTemplates.SYSTEM_PROMPT_SELECT_TEXT
        assert prompt is not None
        assert len(prompt) > 0
        assert "passage" in prompt.lower() or "selected" in prompt.lower()

    def test_rag_query_prompt_formatting(self):
        """Test RAG query prompt is properly formatted."""
        prompt = PromptTemplates.rag_query_prompt(
            context="Sample context text.",
            query="What is the answer?",
        )

        assert "CONTEXT:" in prompt
        assert "QUESTION:" in prompt
        assert "Sample context text." in prompt
        assert "What is the answer?" in prompt

    def test_select_text_query_prompt_formatting(self):
        """Test select-text query prompt is properly formatted."""
        prompt = PromptTemplates.select_text_query_prompt(
            selected_text="Selected passage text.",
            query="What about this?",
        )

        assert "PASSAGE:" in prompt
        assert "QUESTION:" in prompt
        assert "Selected passage text." in prompt
        assert "What about this?" in prompt

    def test_get_system_prompt_rag_mode(self):
        """Test get_system_prompt returns RAG prompt for rag mode."""
        prompt = get_system_prompt(mode="rag")
        assert prompt is not None
        assert len(prompt) > 0

    def test_get_system_prompt_select_text_mode(self):
        """Test get_system_prompt returns select-text prompt."""
        prompt = get_system_prompt(mode="select_text")
        assert prompt is not None
        assert "passage" in prompt.lower() or "selected" in prompt.lower()

    def test_get_user_prompt_rag_mode(self):
        """Test get_user_prompt returns RAG format."""
        prompt = get_user_prompt(
            mode="rag",
            context="Test context",
            query="Test query",
        )

        assert "CONTEXT:" in prompt
        assert "QUESTION:" in prompt

    def test_get_user_prompt_select_text_mode(self):
        """Test get_user_prompt returns select-text format."""
        prompt = get_user_prompt(
            mode="select_text",
            selected_text="Selected text",
            query="Test query",
        )

        assert "PASSAGE:" in prompt
        assert "QUESTION:" in prompt


class TestResponseGeneration:
    """Test response generation behavior."""

    def test_confidence_extraction_prompt(self):
        """Test confidence extraction prompt is defined."""
        prompt = PromptTemplates.confidence_extraction_prompt()
        assert prompt is not None
        assert "confidence" in prompt.lower()

    def test_validation_prompt_structure(self):
        """Test validation prompt includes necessary components."""
        prompt = PromptTemplates.validate_response_prompt(
            response="Test response",
            context="Test context",
            query="Test query",
        )

        assert "Test response" in prompt
        assert "Test context" in prompt
        assert "Test query" in prompt

    def test_example_good_response_defined(self):
        """Test good response example is defined."""
        example = PromptTemplates.EXAMPLE_GOOD_RESPONSE
        assert example is not None
        assert len(example) > 0

    def test_example_bad_response_defined(self):
        """Test bad response example is defined."""
        example = PromptTemplates.EXAMPLE_BAD_RESPONSE
        assert example is not None
        assert "PROBLEMS" in example


class TestPromptSafety:
    """Test prompt templates don't contain unsafe patterns."""

    def test_no_injection_keywords_in_templates(self):
        """Test prompts don't contain obvious injection patterns."""
        templates = [
            PromptTemplates.SYSTEM_PROMPT_RAG,
            PromptTemplates.SYSTEM_PROMPT_SELECT_TEXT,
        ]

        # Simple check for obvious injection attempts
        dangerous_patterns = ["system:", "user:", "ignore:", "override:"]
        for template in templates:
            for pattern in dangerous_patterns:
                # Allow pattern if it appears in context (e.g., "System Prompt:")
                if pattern.capitalize() not in template:
                    assert pattern not in template.lower()

    def test_prompts_enforce_grounding(self):
        """Test RAG prompt enforces grounding."""
        prompt = PromptTemplates.SYSTEM_PROMPT_RAG
        grounding_words = ["ground", "context", "source", "provided"]
        assert any(word in prompt.lower() for word in grounding_words)


class TestPromptParameterization:
    """Test prompt parameter substitution."""

    def test_context_substitution(self):
        """Test context parameter is properly substituted."""
        context = "UNIQUE_TEST_CONTEXT_123"
        prompt = PromptTemplates.rag_query_prompt(
            context=context,
            query="test",
        )
        assert context in prompt

    def test_query_substitution(self):
        """Test query parameter is properly substituted."""
        query = "UNIQUE_TEST_QUERY_456"
        prompt = PromptTemplates.rag_query_prompt(
            context="test",
            query=query,
        )
        assert query in prompt

    def test_selected_text_substitution(self):
        """Test selected_text parameter is properly substituted."""
        text = "UNIQUE_SELECTED_TEXT_789"
        prompt = PromptTemplates.select_text_query_prompt(
            selected_text=text,
            query="test",
        )
        assert text in prompt


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
