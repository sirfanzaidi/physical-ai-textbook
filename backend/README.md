# Embedding Pipeline Backend

RAG chatbot embedding pipeline for the Physical AI & Humanoid Robotics textbook.

## Overview

This backend implements a complete embedding pipeline that:
1. Fetches all 6 chapters from the deployed Docusaurus textbook
2. Extracts clean text from HTML
3. Chunks text semantically (800-1200 characters, 150-200 character overlap)
4. Generates embeddings via Cohere API (1024-dimensional vectors)
5. Stores embeddings in Qdrant vector database
6. Validates semantic search retrieval accuracy
7. Generates a summary report

## Quick Start

### Prerequisites
- Python 3.11+
- UV package manager (https://astral.sh/uv)
- Cohere API key (free tier available)
- Qdrant instance (cloud free tier or local Docker)

### Setup

1. **Create `.env` from template**:
   ```bash
   cp .env.example .env
   ```

2. **Fill in environment variables**:
   ```
   COHERE_API_KEY=your-cohere-api-key
   QDRANT_URL=https://your-qdrant-instance.com
   QDRANT_API_KEY=your-qdrant-key
   TEXTBOOK_BASE_URL=https://physical-ai-textbook-two.vercel.app
   ```

3. **Install dependencies**:
   ```bash
   uv sync
   ```

### Run Pipeline

```bash
python main.py
```

## Tech Stack

- **Language**: Python 3.11+
- **Embeddings**: Cohere API (embed-english-v3.0, 1024-dim)
- **Vector DB**: Qdrant
- **HTTP Client**: httpx with async/await
- **Type Safety**: Pydantic
- **Package Manager**: UV

## Pipeline Phases

### Phase 1: Initialization
- Loads environment variables
- Initializes Cohere and Qdrant clients
- Creates Qdrant collection with schema

### Phase 2: Fetch Content
- Async HTTP fetching from 6 Docusaurus chapter URLs
- Fallback to hardcoded URLs if dynamic parsing fails

### Phase 3: Extract Text
- BeautifulSoup HTML parsing
- Removes navigation, sidebars, footers, scripts
- Preserves headings and content

### Phase 4: Chunk Text
- Recursive character splitting (800-1200 chars)
- 150-200 character overlap
- Sentence boundary detection

### Phase 5: Generate Embeddings
- Batch Cohere API calls (50 chunks/batch)
- Automatic retry on rate limiting
- Validation of 1024-dimensional vectors

### Phase 6: Upsert to Qdrant
- Batch upsertion (100 points/batch)
- Rich metadata storage
- Deduplication checks

### Phase 7: Verify Search
- Test queries from spec acceptance scenarios
- Log top-5 results with similarity scores
- Measure retrieval latency

### Phase 8: Report
- Summary statistics
- Error logging and recovery
- Elapsed time tracking

## Output

Final report includes:
- Chapters processed (target: 6)
- Total chunks created (target: ~200-300)
- Chunks successfully embedded
- Chunks upserted to Qdrant
- Test query results
- Elapsed time

## Error Handling

- Network timeouts: 30-second timeout with graceful fallback
- Missing chapters: Logs warning, continues with others
- Embedding failures: Retries with exponential backoff
- Qdrant errors: Logs error, continues with next batch

## Performance Targets

- Cohere embedding: <200ms per chunk
- Qdrant upsert: <500ms per batch
- Total pipeline: <5 minutes for all 6 chapters
- Semantic search latency: <2 seconds (p95)

## Testing

```bash
pytest tests/ -v
```

## Troubleshooting

**Error: COHERE_API_KEY not set**
- Verify `.env` file exists and has valid API key

**Error: Failed to connect to Qdrant**
- Check QDRANT_URL is correct and instance is running
- For local: `docker run -p 6333:6333 qdrant/qdrant`

**Error: Rate limit exceeded**
- Pipeline automatically retries with backoff
- Wait 1 hour or upgrade to paid Cohere tier

**Error: Timeout fetching chapter**
- Check network connectivity to deployed textbook
- Fallback hardcoded URLs will be used

## Next Steps

After embedding pipeline completes:
1. Build FastAPI chatbot API (Phase 2)
2. Implement semantic search endpoint
3. Integrate with OpenAI for answer generation
4. Connect to Docusaurus frontend

## Files

- `main.py` - Core pipeline implementation (1159 lines)
- `.env.example` - Environment variable template
- `.gitignore` - Git ignore rules
- `pyproject.toml` - UV project configuration
- `tests/` - Unit and integration tests

## Support

For issues or questions, check:
1. `.env` file configuration
2. API key validity
3. Network connectivity
4. Logs in pipeline output
