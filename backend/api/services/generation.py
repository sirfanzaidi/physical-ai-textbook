"""Answer generation service using OpenAI API."""

import logging
from api.models import RetrievedChunk
import openai

logger = logging.getLogger(__name__)


async def generate_answer(
    question: str,
    chunks: list[RetrievedChunk],
    openai_client: openai.OpenAI,
    model: str = "gpt-4o-mini",
    max_tokens: int = 500,
    temperature: float = 0.0,
) -> str:
    """
    Generate an answer using OpenAI API.

    Args:
        question: User's question
        chunks: Retrieved context chunks
        openai_client: OpenAI client instance
        model: OpenAI model to use (gpt-4o-mini for free trial)
        max_tokens: Maximum tokens in response
        temperature: Temperature for generation (0.0 = deterministic)

    Returns:
        Generated answer string
    """
    # Assemble context from retrieved chunks
    context_parts = []
    for chunk in chunks:
        context_parts.append(f"[Chapter {chunk.chapter_num}: {chunk.chapter_title}]\n{chunk.text}")

    context = "\n\n".join(context_parts)

    # System prompt for textbook Q&A
    system_prompt = """You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
Your task is to answer questions using ONLY the provided context from the textbook.

Rules:
1. Base your answer ONLY on the context provided
2. If the answer is not in the context, say "I don't have information about this in the textbook"
3. Be concise but comprehensive (aim for 2-3 paragraphs)
4. Reference specific chapters when relevant
5. Use technical terms correctly
6. If asked about something not in the textbook, politely decline to answer"""

    # User prompt with context and question
    user_message = f"""Context from the textbook:
{context}

Question: {question}

Answer:"""

    logger.info(f"Generating answer for question: {question[:50]}...")

    try:
        # Call OpenAI API
        response = openai_client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message},
            ],
            temperature=temperature,
            max_tokens=max_tokens,
        )

        answer = response.choices[0].message.content
        logger.info(f"Generated answer ({len(answer)} chars)")
        return answer

    except Exception as e:
        logger.error(f"Error generating answer: {e}")
        raise
