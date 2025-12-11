"""Answer generation service using LangChain + Hugging Face (FREE!)."""

import logging
from api.models import RetrievedChunk
from langchain_huggingface import HuggingFaceEndpoint
from langchain.chains import LLMChain
from langchain.prompts import PromptTemplate
from langchain.memory import ConversationBufferMemory

logger = logging.getLogger(__name__)


def get_llm(huggingface_api_key: str = ""):
    """
    Initialize Hugging Face LLM endpoint.

    Uses free Hugging Face Inference API.
    No key required for public models, but providing one gives higher rate limits.

    Free tier: 30,000 requests/month
    """
    # Using free Hugging Face model (mistral-7b-instruct)
    llm = HuggingFaceEndpoint(
        repo_id="mistralai/Mistral-7B-Instruct-v0.1",
        temperature=0.7,
        model_kwargs={
            "max_new_tokens": 500,
            "top_p": 0.95,
        },
        huggingfacehub_api_token=huggingface_api_key if huggingface_api_key else None,
    )
    return llm


async def generate_answer(
    question: str,
    chunks: list[RetrievedChunk],
    huggingface_api_key: str = "",
    max_tokens: int = 500,
    temperature: float = 0.0,
) -> str:
    """
    Generate an answer using LangChain + Hugging Face (completely FREE!).

    Args:
        question: User's question
        chunks: Retrieved context chunks
        huggingface_api_key: Optional HF API key for higher rate limits
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

    # Prompt template for textbook Q&A
    prompt_template = PromptTemplate(
        input_variables=["context", "question"],
        template="""You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
Your task is to answer questions using ONLY the provided context from the textbook.

Rules:
1. Base your answer ONLY on the context provided
2. If the answer is not in the context, say "I don't have information about this in the textbook"
3. Be concise but comprehensive (aim for 2-3 paragraphs)
4. Reference specific chapters when relevant
5. Use technical terms correctly

Context from the textbook:
{context}

Question: {question}

Answer:"""
    )

    logger.info(f"Generating answer for question: {question[:50]}... using Hugging Face")

    try:
        # Initialize LLM
        llm = get_llm(huggingface_api_key)

        # Create LangChain chain with memory
        memory = ConversationBufferMemory(
            memory_key="chat_history",
            return_messages=True
        )

        chain = LLMChain(
            llm=llm,
            prompt=prompt_template,
            memory=memory,
            verbose=False
        )

        # Generate answer
        answer = chain.run(context=context, question=question)

        logger.info(f"Generated answer ({len(answer)} chars) using Hugging Face")
        return answer.strip()

    except Exception as e:
        logger.error(f"Error generating answer: {e}")
        # Fallback response if HF API is unavailable
        return f"I'm having trouble generating a response right now. Please try again. (Error: {str(e)[:50]}...)"
