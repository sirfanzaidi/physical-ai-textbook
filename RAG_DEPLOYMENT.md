# RAG Chatbot Deployment Guide

## Overview

This document covers deployment of the Physical AI & Humanoid Robotics Textbook RAG (Retrieval-Augmented Generation) chatbot backend using:
- **Qdrant** - Vector database for semantic search
- **Cohere** - LLM for embeddings and response generation
- **FastAPI** - REST API framework
- **Sitemap XML** - Automated content discovery

## Architecture

```
Docusaurus Frontend (Vercel)
    ↓
FastAPI Backend (Railway)
    ├── Cohere Client (for embeddings & generation)
    ├── Qdrant Client (for vector search)
    └── Sitemap Parser (discovers pages from frontend)
```

## Prerequisites

### Required API Keys

1. **Cohere API Key** - Get from https://dashboard.cohere.ai
   - Model: `embed-english-v3.0` (for embeddings)
   - Model: `command-r-plus` (for response generation)

2. **Qdrant Cloud** - Get from https://cloud.qdrant.io
   - API Key: Your cluster API key
   - URL: Your cluster URL (e.g., `https://xxxx-xxxxx.qdrant.io`)

3. **Frontend URL** - Deployed Docusaurus site
   - Default: `https://physical-ai-textbook-two.vercel.app`

## Environment Variables

Create a `.env` file in the `backend/` directory with:

```env
# Cohere API
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Sitemap Configuration
SITEMAP_URL=https://physical-ai-textbook-two.vercel.app/sitemap.xml

# CORS Configuration (comma-separated origins)
CORS_ORIGINS=http://localhost:3000,https://physical-ai-textbook-two.vercel.app

# Logging
LOG_LEVEL=INFO
```

## Local Testing

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements-prod.txt
```

### 2. Set Environment Variables

Create `backend/.env` with values from above. On Windows PowerShell:

```powershell
$env:COHERE_API_KEY = "your_key"
$env:QDRANT_URL = "your_url"
$env:QDRANT_API_KEY = "your_key"
$env:SITEMAP_URL = "https://physical-ai-textbook-two.vercel.app/sitemap.xml"
$env:CORS_ORIGINS = "http://localhost:3000,https://physical-ai-textbook-two.vercel.app"
```

### 3. Run Backend

```bash
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 4. Test Health Endpoint

```bash
curl http://localhost:8000/health
```

Expected response:
```json
{
  "status": "ok",
  "version": "2.0.0",
  "timestamp": "2024-12-08T15:30:00.000000",
  "qdrant_collection": "physical_ai_textbook",
  "total_documents": 250
}
```

### 5. Test Chat Endpoint

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is a humanoid robot?",
    "include_sources": true
  }'
```

Expected response:
```json
{
  "answer": "A humanoid robot is...",
  "citations": [
    {
      "text": "Relevant excerpt from textbook...",
      "source": "docs/01-introduction/",
      "url": "https://physical-ai-textbook-two.vercel.app/docs/01-introduction/"
    }
  ],
  "model": "cohere",
  "timestamp": "2024-12-08T15:30:00.000000"
}
```

## Deployment to Railway

### 1. Prepare Railway Configuration

The repository includes:
- `Dockerfile` - Multi-stage build for minimal image size (~2-3GB)
- `railway.toml` - Railway deployment configuration
- `requirements-prod.txt` - Production dependencies

### 2. Push to GitHub

```bash
git add .
git commit -m "Deploy RAG backend with Qdrant + Cohere"
git push origin main
```

### 3. Deploy on Railway

1. Go to https://railway.app
2. Create a new project
3. Select "Deploy from GitHub"
4. Choose your repository
5. Click "Deploy"

### 4. Set Environment Variables on Railway

In Railway dashboard:
1. Go to Variables
2. Add all environment variables:
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `SITEMAP_URL`
   - `CORS_ORIGINS`

### 5. Monitor Deployment

Railway will automatically:
1. Build the Docker image
2. Start the container
3. Make available at a public URL

Check logs in Railway dashboard to verify:
```
INFO:     Application startup complete
```

## API Endpoints

### Health Check
**GET** `/health`

Returns system status and collection statistics.

### Chat Query
**POST** `/chat`

```json
{
  "question": "Your question here",
  "include_sources": true
}
```

Returns answer with citations from textbook.

### Reindex Content
**POST** `/reindex`

Triggers background job to re-ingest content from sitemap. Useful when textbook is updated.

## Content Ingestion Flow

1. **Startup** - On app startup, checks if Qdrant collection exists
2. **Discovery** - Fetches sitemap.xml from frontend URL
3. **Parsing** - Extracts all URLs from sitemap
4. **Fetching** - Downloads HTML content from each URL
5. **Extraction** - Extracts plain text from HTML
6. **Chunking** - Splits text into 500-word chunks with 100-word overlap
7. **Embedding** - Creates Cohere embeddings (4096 dimensions) for each chunk
8. **Storage** - Stores chunks with metadata in Qdrant
9. **Indexing** - Qdrant automatically indexes for semantic search

## Troubleshooting

### Error: COHERE_API_KEY not found
- Check environment variables are set correctly
- Verify API key is valid at https://dashboard.cohere.ai

### Error: QDRANT_URL connection failed
- Verify Qdrant cluster is running and accessible
- Check firewall allows HTTPS connections to your Qdrant URL
- Verify API key is correct

### Error: Sitemap parsing failed
- Verify sitemap URL is accessible (test in browser)
- Check sitemap.xml is valid XML
- Ensure frontend is deployed and sitemap is generated

### Chat endpoint returns empty results
- Check collection has documents: `/health` endpoint shows `total_documents > 0`
- Run `/reindex` to manually trigger content ingestion
- Check Cohere API key has sufficient quota

### Docker image too large
- This is normal. Image size is ~2-3GB due to Cohere embeddings
- Railway plan must support at least 4GB image builds

## Performance Notes

- **Embedding Speed**: ~100-500 chunks per minute depending on Cohere quota
- **Search Latency**: ~500-1000ms per query (embedding + vector search)
- **Collection Size**: ~250+ document chunks from 15 chapters
- **Memory**: Qdrant uses ~500MB-1GB for this dataset

## Monitoring

### Check Qdrant Collection

```bash
curl -H "Authorization: Bearer YOUR_QDRANT_API_KEY" \
  https://your-cluster.qdrant.io/collections/physical_ai_textbook
```

### Monitor API Usage

Check Railway dashboard:
- CPU usage
- Memory consumption
- API response times
- Error rates

## Integration with Frontend

The Docusaurus frontend can call the RAG backend:

```javascript
// In your React component
const response = await fetch('https://your-railway-url/chat', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    question: userQuestion,
    include_sources: true
  })
});

const data = await response.json();
console.log(data.answer);
console.log(data.citations);
```

## Further Reading

- [Qdrant Docs](https://qdrant.tech/documentation/)
- [Cohere Docs](https://docs.cohere.com/)
- [FastAPI Docs](https://fastapi.tiangolo.com/)
- [Railway Docs](https://docs.railway.app/)
