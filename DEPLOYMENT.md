# Deployment Guide - Physical AI Textbook RAG Chatbot API

This guide covers deploying the FastAPI chatbot backend to production.

## Recommended: Railway Deployment

Railway is the recommended platform for this Python FastAPI application.

**Sign up at: https://railway.app**

### Quick Start (5 minutes)

1. Connect your GitHub account to Railway
2. Create new project and select this GitHub repo
3. Add environment variables in Railway dashboard:
   - COHERE_API_KEY
   - OPENAI_API_KEY
   - QDRANT_URL
   - QDRANT_API_KEY
   - TEXTBOOK_BASE_URL=https://physical-ai-textbook-two.vercel.app
4. Railway automatically builds and deploys from GitHub
5. Your API will be live at: https://your-project.up.railway.app

### Update Vercel

Update `vercel.json` with your Railway API URL:

```json
{
  "rewrites": [
    {
      "source": "/api/:path*",
      "destination": "https://your-project.up.railway.app/api/:path*"
    }
  ]
}
```

## Alternative: Render.com

Similar to Railway with straightforward GitHub integration.

1. Sign up at https://render.com
2. Create Web Service from GitHub
3. Build: `cd backend && pip install -r requirements.txt`
4. Start: `cd backend && python -m uvicorn api.app:app --host 0.0.0.0 --port $PORT`
5. Add environment variables same as Railway

## Docker + Any Cloud

```bash
docker build -t physical-ai-api backend/
docker run -p 8000:8000 \
  -e COHERE_API_KEY=your_key \
  -e OPENAI_API_KEY=sk_your_key \
  -e QDRANT_URL=your_url \
  -e QDRANT_API_KEY=your_key \
  physical-ai-api:latest
```

## Testing

After deployment:

```bash
# Health check
curl https://your-api.example.com/api/health

# Test chat
curl -X POST https://your-api.example.com/api/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is physical AI?"}'
```

## Environment Variables

| Variable | Required | Default | Notes |
|----------|----------|---------|-------|
| COHERE_API_KEY | Yes | - | Get from cohere.ai |
| OPENAI_API_KEY | Yes | - | Get from openai.com |
| QDRANT_URL | Yes | - | Cloud instance URL |
| QDRANT_API_KEY | Yes | - | Qdrant API key |
| TEXTBOOK_BASE_URL | Yes | - | Website base URL |
| RETRIEVAL_LIMIT | No | 5 | Number of chunks |
| MAX_TOKENS | No | 500 | Max response length |
| TEMPERATURE | No | 0.0 | Generation randomness |
| LOG_LEVEL | No | INFO | DEBUG/INFO/WARN/ERROR |

## Cost Estimate (Free Tier)

- Railway: $5/month credit (free tier)
- Render: Free tier available
- Cohere: 1,000 API calls/month (free)
- OpenAI: Free trial $18 credit
- Qdrant: Free cloud tier

Total for production: ~$7-10/month

## Next Steps

1. Deploy using Railway or Render
2. Update Vercel configuration
3. Test the RAG pipeline end-to-end
4. Monitor API logs and performance
5. Set up caching and rate limiting if needed

See `backend/README.md` for local development.
