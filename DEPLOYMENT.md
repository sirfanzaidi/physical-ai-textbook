# Deployment Guide

This guide explains how to deploy the Physical AI Textbook RAG system to production using Vercel (frontend) and Railway/Render (backend).

## Architecture

- **Frontend**: Docusaurus + RAG Chat UI (deployed on Vercel)
- **Backend**: FastAPI + OpenRouter + Qdrant (deployed on Railway or Render)
- **Vector Database**: Qdrant Cloud
- **LLM Provider**: OpenRouter (GPT-3.5-turbo for generation, Qwen for embeddings)

## Frontend Deployment (Vercel)

### Prerequisites
- GitHub account (repository connected to Vercel)
- Backend API URL (from Railway/Render deployment)

### Setup Steps

1. **Connect Repository**
   - Go to https://vercel.com
   - Import your GitHub repository
   - Select the root directory

2. **Configure Environment Variables**
   - In Vercel Project Settings > Environment Variables, add:
   ```
   BACKEND_URL=https://your-backend-url.railway.app
   ```
   - Replace with your actual backend URL

3. **Build Settings**
   - Build Command: `cd website && npm run build`
   - Output Directory: `website/build`
   - Framework Preset: Docusaurus

4. **Deploy**
   - Save settings and redeploy
   - Frontend will be available at your Vercel project URL

## Backend Deployment (Railway)

### Prerequisites
- Railway account (https://railway.app)
- GitHub repository (for automatic deployments)
- Environment variables configured

### Setup Steps

1. **Create New Project**
   - Go to Railway dashboard
   - Click "New Project" > "Deploy from GitHub"
   - Select your repository

2. **Configure Build**
   - Railway will detect `railway.json` automatically
   - Build: Dockerfile in `backend` folder
   - No additional build config needed

3. **Set Environment Variables**
   - In Railway Project Settings > Variables, add:
   ```
   OPENROUTER_API_KEY=sk-or-v1-xxxx
   QDRANT_URL=https://your-qdrant-cluster.us-east4-0.gcp.cloud.qdrant.io
   QDRANT_API_KEY=your-qdrant-api-key
   QDRANT_COLLECTION_NAME=physical_ai_textbook
   LOG_LEVEL=INFO
   ```

4. **Deploy**
   - Railway will auto-deploy on git push
   - Get your backend URL from Railway dashboard
   - Copy the public URL (e.g., https://physical-ai-textbook-backend.railway.app)

5. **Update Frontend**
   - Set Vercel `BACKEND_URL` environment variable to your Railway URL
   - Redeploy frontend

## Backend Deployment (Render - Alternative)

### Setup Steps

1. **Create New Web Service**
   - Go to https://render.com
   - Create > Web Service
   - Connect your GitHub repository

2. **Configure Service**
   - Environment: Python 3.11
   - Build Command: See `render.yaml` (auto-detected)
   - Start Command: See `render.yaml` (auto-detected)
   - Plan: Free (or paid for production)

3. **Set Environment Variables**
   Same as Railway setup above

4. **Deploy**
   - Render will build and deploy automatically
   - Get your backend URL from Render dashboard
   - Update Vercel `BACKEND_URL` environment variable

## Environment Variables Reference

### Backend (.env file)
```
OPENROUTER_API_KEY=sk-or-v1-xxxx
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
QDRANT_URL=https://your-qdrant-cluster.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
QDRANT_COLLECTION_NAME=physical_ai_textbook
CORS_ORIGINS=https://your-frontend-url.vercel.app
LOG_LEVEL=INFO
```

### Frontend (Vercel)
```
BACKEND_URL=https://your-backend-url.railway.app
```

## Testing Deployment

### Test Backend Health
```bash
curl https://your-backend-url.railway.app/api/health
```

Expected: `{"status":"healthy"}`

### Test Frontend API
1. Open your Vercel frontend URL
2. Open DevTools > Network tab
3. Try a chat query
4. Verify requests go to `/api/chat-stream` (not `/api/api/chat-stream`)

## Troubleshooting

- **404 error on frontend**: Check Vercel `BACKEND_URL` environment variable
- **401 error**: Verify `OPENROUTER_API_KEY` in backend
- **Streaming not working**: Check browser console and BACKEND_URL configuration
- **Vector dimension mismatch**: Collection auto-creates with correct 2560-dim vectors

## Monitoring

- **Backend**: Railway/Render dashboard logs
- **Frontend**: Vercel analytics tab
- **Look for**: "RAG services initialized successfully" in backend logs
