# Chatbot Setup Guide

Your FastAPI backend is running successfully! However, the chatbot needs a valid **OpenAI API key** to generate answers.

## Current Status

✅ **Backend Server**: Running on `http://localhost:8000`
✅ **API Health**: Healthy
✅ **Vector Database**: Connected to Qdrant
✅ **Embeddings**: Cohere API working

❌ **OpenAI Integration**: Needs valid API key

## Getting Started with OpenAI

### Step 1: Create OpenAI Account

1. Go to https://platform.openai.com
2. Sign up with your email or Google account
3. Verify your email

### Step 2: Create API Key

1. Navigate to https://platform.openai.com/account/api-keys
2. Click "Create new secret key"
3. Copy the key (starts with `sk-`)
4. **Save it somewhere safe** - you won't be able to see it again

### Step 3: Add to .env File

Edit `backend/.env`:

```bash
# Replace this:
OPENAI_API_KEY=sk-your-key-here

# With your actual key:
OPENAI_API_KEY=sk-proj-your-actual-key-here
```

### Step 4: Restart Backend Server

**Stop the current server** (Ctrl+C in the terminal where it's running)

**Restart it:**

```bash
cd backend
python -m uvicorn api.app:app --reload --host 0.0.0.0 --port 8000
```

Wait for: `Uvicorn running on http://0.0.0.0:8000`

### Step 5: Test the Chatbot

```bash
# Test the API
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is physical AI?"}'
```

Or use the interactive Swagger UI:
- **Swagger**: http://localhost:8000/api/docs
- **ReDoc**: http://localhost:8000/api/redoc

## Costs

- **Free Trial**: $18 credit (enough for 1000+ queries)
- **gpt-4o-mini**: ~$0.00015 per 1k input tokens, $0.0006 per 1k output tokens
- **Typical query**: ~200 tokens → ~$0.00015 per query
- **Budget**: $18 credit = ~120,000 queries

## Alternative: Use Mock Responses (Local Testing)

If you don't have an OpenAI key yet, you can use mock responses for testing:

Edit `backend/api/services/generation.py`:

```python
async def generate_answer(...) -> str:
    # For testing without OpenAI API key:
    if openai_client is None or "sk-" not in openai_client.api_key:
        return "This is a mock answer for testing. Please add a valid OpenAI API key to get real answers."
    
    # ... rest of the function
```

Then restart the server.

## Troubleshooting

### "Invalid API key" Error
- ✅ Check that you copied the full key (starts with `sk-`)
- ✅ Verify the key is in `backend/.env`
- ✅ Restart the backend server after updating

### "Timeout" Error
- Check your internet connection
- Verify OpenAI status: https://status.openai.com

### "Rate limit exceeded"
- You've exceeded the free trial quota
- Upgrade to paid tier at https://platform.openai.com/account/billing/overview

## Testing the API

### Health Check
```bash
curl http://localhost:8000/api/health
```

### Chat Endpoint
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is humanoid robotics?",
    "chapter_filter": null,
    "selected_text": null
  }'
```

### Response Example
```json
{
  "answer": "Humanoid robotics is the field of designing and building robots with a human-like form...",
  "sources": [
    {
      "chapter_num": 2,
      "chapter_title": "Basics of Humanoid Robotics",
      "chapter_url": "https://physical-ai-textbook-two.vercel.app/docs/chapter-2",
      "snippet": "Humanoid robots are robots designed to mimic human anatomy..."
    }
  ],
  "retrieved_chunks_count": 3
}
```

## API Documentation

Once the server is running with a valid OpenAI key, access:

- **Swagger UI**: http://localhost:8000/api/docs
- **ReDoc**: http://localhost:8000/api/redoc
- **Health Check**: http://localhost:8000/api/health
- **Chat**: POST http://localhost:8000/api/chat

## Environment Variables

| Variable | Required | Status | Example |
|----------|----------|--------|---------|
| COHERE_API_KEY | Yes | ✅ Set | KEAWurw7... |
| OPENAI_API_KEY | Yes | ❌ Placeholder | sk-proj-... |
| QDRANT_URL | Yes | ✅ Set | https://...gcp.cloud.qdrant.io |
| QDRANT_API_KEY | Yes | ✅ Set | eyJhbGci... |
| TEXTBOOK_BASE_URL | Yes | ✅ Set | https://physical-ai-textbook-two.vercel.app |

## Next Steps

1. ✅ Get OpenAI API key from https://platform.openai.com/account/api-keys
2. ✅ Update `backend/.env` with your key
3. ✅ Restart the backend server
4. ✅ Test the chatbot at http://localhost:3000 (or use API directly)
5. ✅ Deploy to Railway/Render when ready (see DEPLOYMENT.md)

## Need Help?

- OpenAI Docs: https://platform.openai.com/docs
- API Reference: http://localhost:8000/api/docs
- Issues: https://github.com/sirfanzaidi/physical-ai-textbook/issues

