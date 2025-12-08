# RAG Backend Deployment Guide

Deploy the RAG chatbot backend to Railway (free tier with generous limits).

## Option 1: Deploy to Railway (Recommended - 5 Minutes)

Railway is the easiest option. Free tier includes 500 hours/month (always-on).

### Step 1: Create Railway Account
1. Go to https://railway.app
2. Sign up with GitHub (recommended)
3. Authorize Railway to access your GitHub account

### Step 2: Deploy from GitHub

1. In Railway Dashboard, click **"New Project"**
2. Select **"Deploy from GitHub repo"**
3. Find your repository: `physical-ai-textbook`
4. Select the repository
5. Railway will auto-detect:
   - **Dockerfile**: `backend/Dockerfile` ✓
   - **Port**: 8000 ✓

### Step 3: Configure Environment Variables

In Railway Project Settings → Variables, add:

```
PYTHONUNBUFFERED=1
CORS_ORIGINS=https://physical-ai-textbook.vercel.app,http://localhost:3000
CHROMADB_PATH=/tmp/embeddings
EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2
LLM_MODEL=template
LOG_LEVEL=INFO
```

### Step 4: Deploy

1. Click **"Deploy"**
2. Wait 3-5 minutes for build to complete
3. You'll get a public URL like: `https://physical-ai-chatbot-production-xyz.up.railway.app`

### Step 5: Test API

```bash
curl https://your-railway-url/health
# Response: {"status": "ok", "version": "1.0.0", ...}

curl -X POST https://your-railway-url/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "include_sources": true}'
```

## Option 2: Deploy to Render.com

Alternative to Railway with similar free tier.

### Steps:
1. Go to https://render.com
2. Click **"New +"** → **"Web Service"**
3. Connect GitHub repo
4. Set:
   - **Name**: `physical-ai-chatbot`
   - **Build Command**: (leave default)
   - **Start Command**: `python -m uvicorn main:app --host 0.0.0.0 --port 8000`
   - **Root Directory**: `backend`
5. Add environment variables (same as Railway)
6. Deploy

Free tier: 750 hours/month (spins down after 15 min inactivity)

## Option 3: Deploy Locally with Docker

For testing before cloud deployment.

```bash
cd backend

# Build image
docker build -t rag-chatbot:latest .

# Run container
docker run -p 8000:8000 \
  -e CORS_ORIGINS="http://localhost:3000" \
  -v $(pwd)/data:/app/data \
  rag-chatbot:latest

# Test
curl http://localhost:8000/health
```

## Integration with Frontend

Update your frontend to point to the deployed RAG API:

```javascript
// In your React component
const RAG_API_URL = process.env.REACT_APP_RAG_API_URL || 'http://localhost:8000';

async function askChatbot(question) {
  const response = await fetch(`${RAG_API_URL}/chat`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      query: question,
      include_sources: true
    })
  });
  return response.json();
}
```

Add to your `.env.local`:

```
REACT_APP_RAG_API_URL=https://your-railway-url
```

## Monitoring

### Health Check
```bash
curl https://your-deployment-url/health
```

Response includes:
- API version and status
- Uptime
- Total chunks indexed
- Database size

### Logs
- **Railway**: Dashboard → Deployments → Logs
- **Render**: Dashboard → Logs

## Troubleshooting

### Build Fails: "Module not found"
- Check `requirements.txt` has all dependencies
- Ensure `backend/` directory is in correct place

### API Returns 503 Service Unavailable
- Service might be starting up (takes 30s)
- Check logs for embedding model download status

### CORS Errors
- Update `CORS_ORIGINS` to include your frontend domain
- Must be HTTPS in production

### Out of Memory (OOM)
- Embedding models are large (~600MB)
- Use smaller model: `all-MiniLM-L6-v2` (384D, faster)
- Alternative: `all-distilroberta-v1` (768D, slower)

## Indexing Content

After deployment, index the textbook:

```bash
# Via HTTP (if you added /reindex endpoint)
curl -X POST https://your-deployment-url/reindex \
  -H "Content-Type: application/json" \
  -d '{"chapters": ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15"]}'
```

Or run indexing locally:

```bash
cd backend
python scripts/ingest.py
```

## Cost Estimates

| Service | Free Tier | Cost Beyond |
|---------|-----------|-------------|
| **Railway** | 500 hrs/mo | $5/month |
| **Render** | 750 hrs/mo | $7/month |
| **AWS Lambda** | 1M requests/mo | Pay per invocation |

## Next Steps

1. ✅ Deploy backend to Railway/Render
2. 📱 Update frontend with RAG API URL
3. 🧪 Test chatbot integration
4. 📊 Monitor performance and latency
5. 🔄 Set up auto-reindexing for new chapters

---

**Need help?**
- Railway Docs: https://docs.railway.app/
- Render Docs: https://render.com/docs
- FastAPI Docs: https://fastapi.tiangolo.com/
