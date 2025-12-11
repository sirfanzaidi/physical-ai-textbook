# Quick Start Guide

Get the embedding pipeline running in 5 minutes.

## 1. Install UV (if not already installed)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## 2. Clone and navigate

```bash
cd backend
```

## 3. Create .env file

```bash
cp .env.example .env
```

## 4. Edit .env with your API keys

```bash
export COHERE_API_KEY="your-cohere-api-key"
export QDRANT_URL="http://localhost:6333"  # or Qdrant Cloud URL
export QDRANT_API_KEY="optional"
export TEXTBOOK_BASE_URL="https://physical-ai-textbook-two.vercel.app"
```

## 5. Install dependencies

```bash
uv sync
```

## 6. Run pipeline

```bash
python main.py
```

## Expected Output

```
[2025-12-11 14:30:00] [INFO] ================================================
[2025-12-11 14:30:00] [INFO] EMBEDDING PIPELINE - Physical AI Textbook RAG
[2025-12-11 14:30:00] [INFO] ================================================
[2025-12-11 14:30:00] [INFO] Initializing Cohere client...
[2025-12-11 14:30:01] [INFO] ✓ Cohere client initialized
[2025-12-11 14:30:01] [INFO] Initializing Qdrant client...
[2025-12-11 14:30:02] [INFO] ✓ Qdrant client initialized
[2025-12-11 14:30:02] [INFO] Setting up Qdrant collection...
[2025-12-11 14:30:03] [INFO] ✓ Collection 'book_chunks' already exists
[2025-12-11 14:30:03] [INFO] Found 6 chapters to index

[2025-12-11 14:30:03] [INFO] [1/6] Processing: Introduction to Physical AI
[2025-12-11 14:30:05] [INFO]   Fetching: https://physical-ai-textbook-two.vercel.app/docs/intro
[2025-12-11 14:30:06] [INFO]   Extracted 12450 characters
[2025-12-11 14:30:06] [INFO]   Chunking Chapter 1...
[2025-12-11 14:30:06] [INFO]     Created 48 chunks
[2025-12-11 14:30:08] [INFO]   Generating embeddings for 48 chunks...
[2025-12-11 14:30:12] [INFO]   ✓ Embeddings complete
[2025-12-11 14:30:12] [INFO]   Upserting 48 chunks to Qdrant...
[2025-12-11 14:30:13] [INFO]   ✓ Upserted 48 chunks
[2025-12-11 14:30:13] [INFO]   ✓ Chapter 1 complete

... (chapters 2-6 processed similarly) ...

[2025-12-11 14:35:22] [INFO]
[2025-12-11 14:35:22] [INFO] ================================================
[2025-12-11 14:35:22] [INFO] PIPELINE SUMMARY
[2025-12-11 14:35:22] [INFO] ================================================
[2025-12-11 14:35:22] [INFO] ✓ Chapters processed: 6
[2025-12-11 14:35:22] [INFO] ✓ Total chunks created: 287
[2025-12-11 14:35:22] [INFO] ✓ Total chunks embedded & upserted: 287
[2025-12-11 14:35:22] [INFO] ✓ Queries tested: 5
[2025-12-11 14:35:22] [INFO] ✓ Total time: 321.45 seconds (5.36 minutes)
[2025-12-11 14:35:22] [INFO] ================================================
```

## Troubleshooting

### "COHERE_API_KEY not set"
- Check that `.env` file exists in backend/ directory
- Verify API key is valid
- Restart terminal/shell if environment was updated

### "Failed to connect to Qdrant"
- Check QDRANT_URL is correct
- For local: Run `docker run -p 6333:6333 qdrant/qdrant`
- For cloud: Verify credentials and network access

### "Rate limit exceeded from Cohere"
- Wait 1 hour (free tier resets hourly)
- Or upgrade to paid Cohere plan
- Pipeline automatically retries with backoff

### "Timeout fetching chapter"
- Check internet connectivity
- Deployed textbook may be temporarily unavailable
- Pipeline will use fallback hardcoded URLs

## Next Steps

1. **Verify Semantic Search**
   ```bash
   python -c "import main; main.verify_search(...)"
   ```

2. **Build Chatbot API** (Phase 2)
   - Create FastAPI endpoints for `/query` and `/search`
   - Integrate with OpenAI for answer generation
   - Deploy to Vercel or self-hosted

3. **Connect Frontend** (Phase 3)
   - Add Docusaurus plugin for "Ask AI" button
   - Implement select-text feature
   - Add real-time search

## Support

- Check README.md for detailed documentation
- Review main.py comments for implementation details
- Check logs output for error messages and debugging info
