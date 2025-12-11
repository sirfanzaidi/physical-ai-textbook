"""Chat endpoint for RAG Q&A."""

import logging
import time
from fastapi import APIRouter, Depends, HTTPException
from api.models import ChatQuery, ChatResponse, SourceCitation, RetrievedChunk
from api.dependencies import (
    get_embedding_client,
    get_qdrant_manager,
    get_openai_client,
    get_settings,
)
from api.services.generation import generate_answer

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["chat"])


@router.post("/chat", response_model=ChatResponse)
async def chat(
    query: ChatQuery,
    embedding_client=Depends(get_embedding_client),
    qdrant_manager=Depends(get_qdrant_manager),
    openai_client=Depends(get_openai_client),
    settings=Depends(get_settings),
):
    """
    RAG Chatbot endpoint.

    Process flow:
    1. Embed user question using Cohere
    2. Retrieve relevant chunks from Qdrant
    3. Generate answer using OpenAI GPT-4o-mini
    4. Return answer with source citations

    Args:
        query: User's chat query with question
        embedding_client: Dependency for embedding
        qdrant_manager: Dependency for retrieval
        openai_client: Dependency for generation
        settings: Dependency for configuration

    Returns:
        ChatResponse with answer and sources

    Raises:
        HTTPException: On retrieval or generation errors
    """
    start_time = time.time()

    try:
        logger.info(f"Processing chat query: {query.question[:50]}...")

        # Step 1: Embed user question
        logger.debug("Embedding user question...")
        query_embeddings = await embedding_client.embed_batch([query.question])

        if not query_embeddings or not query_embeddings[0]:
            logger.error("Failed to generate embedding for query")
            raise HTTPException(status_code=500, detail="Failed to embed query")

        query_embedding = query_embeddings[0]
        logger.debug(f"Query embedding generated (dimension: {len(query_embedding)})")

        # Step 2: Retrieve relevant chunks
        logger.debug(f"Retrieving top-{settings.retrieval_limit} chunks...")
        results = qdrant_manager.search(
            query_embedding=query_embedding, limit=settings.retrieval_limit
        )

        logger.info(f"Retrieved {len(results)} chunks from Qdrant")

        if not results:
            logger.info("No relevant chunks found for query")
            return ChatResponse(
                answer="I couldn't find relevant information in the textbook to answer this question. "
                "Please try rephrasing your question.",
                sources=[],
                retrieved_chunks_count=0,
            )

        # Convert to RetrievedChunk format with full text
        chunks = []
        for result in results:
            chunk = RetrievedChunk(
                chunk_id=result.chunk_id,
                chapter_num=result.chapter_num,
                chapter_title=result.chapter_title,
                chapter_url=result.chapter_url,
                text=result.text if hasattr(result, "text") and result.text else result.text_snippet,
                similarity_score=result.similarity_score,
            )
            chunks.append(chunk)
            logger.debug(
                f"  - {result.chapter_title} (score: {result.similarity_score:.3f})"
            )

        # Step 3: Generate answer
        logger.debug("Generating answer using OpenAI...")
        answer = await generate_answer(
            question=query.question,
            chunks=chunks,
            openai_client=openai_client,
            model=settings.openai_model,
            max_tokens=settings.max_tokens,
            temperature=settings.temperature,
        )

        # Step 4: Create source citations
        sources = [
            SourceCitation(
                chapter_num=chunk.chapter_num,
                chapter_title=chunk.chapter_title,
                chapter_url=chunk.chapter_url,
                snippet=chunk.text[:300] + "..." if len(chunk.text) > 300 else chunk.text,
            )
            for chunk in chunks
        ]

        elapsed = time.time() - start_time
        logger.info(f"Chat request completed in {elapsed:.2f}s")

        return ChatResponse(
            answer=answer, sources=sources, retrieved_chunks_count=len(chunks)
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Chat endpoint error: {type(e).__name__}: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")
