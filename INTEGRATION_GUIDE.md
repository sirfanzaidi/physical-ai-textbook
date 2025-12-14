# RAG Chatbot Integration Guide

Complete guide for integrating the RAG chatbot backend with the Docusaurus textbook frontend.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│              Docusaurus Website (Vercel)                     │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  React Components + Pages                           │   │
│  │  ├─ website/src/pages/chat.tsx (Chat Page)          │   │
│  │  ├─ website/src/components/RAGChat.tsx (Wrapper)    │   │
│  │  └─ website/src/config/ragConfig.ts (Config)        │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  Static Assets (website/static/)                    │   │
│  │  ├─ chat-widget.js (Vanilla JS component)           │   │
│  │  ├─ utils.js (API client + utilities)               │   │
│  │  └─ chat-widget.css (Responsive styling)            │   │
│  └─────────────────────────────────────────────────────┘   │
└────────────────┬─────────────────────────────────────────────┘
                 │ HTTP/REST
                 │ (via vercel.json rewrites)
                 ↓
┌─────────────────────────────────────────────────────────────┐
│          FastAPI Backend (Railway/Render)                    │
│  ├─ POST /api/ingest - Upload and index books              │
│  ├─ POST /api/chat - Query with RAG                        │
│  ├─ POST /api/chat-stream - Streaming responses            │
│  └─ GET /api/health - Health check                         │
└────────────────┬─────────────────────────────────────────────┘
                 │
        ┌────────┴─────────┐
        ↓                  ↓
   Qdrant Cloud      Neon PostgreSQL
   (Vector DB)       (Metadata - Optional)
```

## Files Created for Integration

### Frontend Components

1. **website/src/components/RAGChat.tsx**
   - React wrapper component for the Vanilla JS chat widget
   - Handles script loading, CSS loading, and initialization
   - Props: bookId, apiBaseURL, enableStreaming, enableHistory, height

2. **website/src/components/RAGChat.module.css**
   - Styling for the RAG Chat wrapper
   - Responsive layout, dark mode support

3. **website/src/pages/chat.tsx**
   - Dedicated full-page chat interface
   - Displays the RAG Chat component with information sections
   - Integrated into Docusaurus routing

4. **website/src/pages/chat.module.css**
   - Styling for the chat page
   - Hero header, info sections, example queries

5. **website/src/config/ragConfig.ts**
   - Environment-aware configuration
   - Auto-detects localhost vs. production API URLs
   - Manages book ID, streaming settings, etc.

### Frontend Assets

Files copied from `frontend/` to `website/static/`:
- `chat-widget.js` - Vanilla JS component (460+ lines)
- `utils.js` - API client and utilities (320+ lines)
- `chat-widget.css` - Responsive styling (440+ lines)

### Configuration Files

1. **website/docusaurus.config.ts** (Updated)
   - Added "💬 Ask RAG Bot" link to navbar
   - Points to `/chat` route

2. **website/vercel.json** (New)
   - Configures API rewrites for production
   - Routes `/api/*` requests to backend
   - Sets up CORS headers

## Setup Instructions

### 1. Development Setup

#### Frontend (Docusaurus)

```bash
cd website
npm install
npm run start
```

- Frontend runs at: http://localhost:3000
- Chat page: http://localhost:3000/chat
- Navbar includes "💬 Ask RAG Bot" link

#### Backend (FastAPI)

```bash
# From root directory
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
uvicorn backend.app.main:app --reload
```

- Backend runs at: http://localhost:8000
- RAG Chat will connect to: http://localhost:8000
- API docs: http://localhost:8000/docs

### 2. Upload & Index a Book

```bash
# From another terminal, after starting the backend
curl -X POST http://localhost:8000/api/ingest \
  -F "file=@sample.pdf" \
  -F "book_id=physical-ai-textbook"
```

Expected response:
```json
{
  "success": true,
  "book_id": "physical-ai-textbook",
  "chunk_count": 245,
  "total_tokens": 98750,
  "message": "Successfully ingested..."
}
```

### 3. Test the Integration

1. Open http://localhost:3000/chat
2. Type a question: "What is the main focus of this textbook?"
3. Watch the response stream in real-time
4. View citations with page numbers
5. Check your message history persists in localStorage

## Production Deployment

### Prerequisites

- Vercel account (for frontend)
- Railway or Render account (for backend)
- Cohere API key
- Qdrant Cloud instance
- Neon PostgreSQL (optional)

### Frontend Deployment (Vercel)

1. **Connect GitHub repository**
   ```bash
   # Vercel auto-detects Docusaurus setup
   cd website
   npm run build  # Test build locally
   ```

2. **Deploy to Vercel**
   - Connect repository at https://vercel.com/new
   - Select "Docusaurus" template
   - Build command: `npm run build`
   - Output directory: `build`
   - Root directory: `website`

3. **Update vercel.json**
   ```json
   {
     "rewrites": [
       {
         "source": "/api/:path*",
         "destination": "https://your-backend-url.railway.app/api/:path*"
       }
     ]
   }
   ```
   Replace `https://your-backend-url.railway.app` with your actual backend URL.

### Backend Deployment (Railway)

1. **Push code to GitHub**
   ```bash
   git add .
   git commit -m "Add RAG chatbot integration"
   git push
   ```

2. **Deploy to Railway**
   - Sign up at https://railway.app
   - Connect GitHub account
   - Create new project from this repository
   - Set environment variables:
     ```
     COHERE_API_KEY=your_key
     QDRANT_URL=https://your.qdrant.io
     QDRANT_API_KEY=your_key
     QDRANT_COLLECTION=book_vectors
     ENVIRONMENT=production
     ```

3. **Get Backend URL**
   - Railway generates a public URL (e.g., `https://api.railway.app`)
   - Update `vercel.json` with this URL

4. **Index the Book**
   ```bash
   curl -X POST https://your-backend-url/api/ingest \
     -F "file=@book.pdf" \
     -F "book_id=physical-ai-textbook"
   ```

### Environment Variables

**Frontend (Vercel)** - No sensitive variables needed (uses relative /api path)

**Backend (Railway)** - Add to Railway dashboard:
```
COHERE_API_KEY=sk-...
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=xxx
QDRANT_COLLECTION=book_vectors
DATABASE_URL=postgresql://...  (optional)
ENVIRONMENT=production
```

## Configuration

### Book ID

Set in `website/src/config/ragConfig.ts`:
```typescript
export const getRagConfig = (): RAGChatConfig => {
  return {
    bookId: 'physical-ai-textbook',  // ← Change this
    // ...
  };
};
```

Must match the book_id used during ingestion.

### API URL

Automatically handled by `ragConfig.ts`:
- **Development**: `http://localhost:8000`
- **Production**: `/api` (via vercel.json rewrites)
- **Staging**: Configure as needed

### Streaming & History

Toggle in `website/src/config/ragConfig.ts`:
```typescript
{
  enableStreaming: true,   // Character-by-character response
  enableHistory: true,     // localStorage persistence
}
```

## Customization

### Chat Widget Appearance

Edit `website/static/chat-widget.css`:
- Colors: Update `#667eea` (primary) and `#764ba2` (secondary)
- Sizing: Change max-height (default: 600px)
- Typography: Modify font-family, sizes

### Chat Page Layout

Edit `website/src/pages/chat.tsx`:
- Adjust width, padding, spacing
- Add/remove info sections
- Customize example queries

### Component Props

Modify `website/src/components/RAGChat.tsx`:
- Add new configuration options
- Change script loading strategy
- Add error handling

## Testing Checklist

- [ ] Frontend builds: `cd website && npm run build`
- [ ] Backend runs: `uvicorn backend.app.main:app --reload`
- [ ] Health check passes: `curl http://localhost:8000/api/health`
- [ ] Book indexing works: `curl -X POST http://localhost:8000/api/ingest ...`
- [ ] Chat page loads: http://localhost:3000/chat
- [ ] Widget initializes in console: no errors
- [ ] Can type and submit query
- [ ] Response streams in real-time
- [ ] Citations display with page numbers
- [ ] Message history saves to localStorage
- [ ] Dark mode works
- [ ] Mobile responsive (test at 480px, 768px)
- [ ] Select-text mode works (highlight text, click "Ask")

## Troubleshooting

### Widget Doesn't Load

**Error**: "Cannot find RAGChatWidget"

Solutions:
1. Verify files in `website/static/`: chat-widget.js, utils.js, chat-widget.css
2. Check browser console for 404 errors
3. Ensure scripts load in correct order (utils.js first, then chat-widget.js)
4. Verify paths in RAGChat.tsx component

### API Connection Fails

**Error**: "Failed to connect to API"

Solutions:
1. Check backend is running: `curl http://localhost:8000/api/health`
2. Verify API URL in ragConfig.ts
3. Check CORS headers in vercel.json
4. In production, ensure backend URL in vercel.json is correct
5. Check browser Network tab for actual request URL

### Book Not Found

**Error**: "No chunks found for book"

Solutions:
1. Verify book_id matches between ingestion and query
2. Check book was successfully indexed: `curl http://localhost:8000/api/health`
3. Verify Qdrant collection exists and has chunks
4. Re-index book: `curl -X POST http://localhost:8000/api/ingest ...`

### Messages Don't Persist

**Issue**: Conversation lost on page reload

Solutions:
1. Ensure `enableHistory: true` in ragConfig.ts
2. Check browser localStorage enabled (not in private mode)
3. Verify localStorage size not exceeded (5-10MB limit)
4. Clear old data: `localStorage.clear()` in console

## Performance Optimization

### Frontend

- Code splitting: Docusaurus handles automatically
- Widget lazy loading: Load scripts on chat page only
- CSS optimization: Minified in production

### Backend

- Indexing: ~5 min for 500-page book
- Query latency: <5s p95 target
- Streaming chunks: ~0.01s delay for smooth display
- Reranking: Top 8-10 results from top 20-30 initial matches

## Monitoring

### Frontend Metrics

- Page load time (should be <3s)
- Widget initialization time
- API request latency
- Error rates

### Backend Metrics

- API response times
- Cohere API usage
- Qdrant query performance
- Database connection pool utilization

Use services like:
- Vercel Analytics (frontend)
- Railway Metrics (backend)
- Sentry (error tracking)

## Next Steps

1. **Test locally** - Follow development setup above
2. **Deploy frontend** - Push to Vercel
3. **Deploy backend** - Push to Railway
4. **Index book** - Upload and process PDF
5. **Monitor** - Watch metrics and logs
6. **Iterate** - Adjust chunk size, top_k based on accuracy tests

## Support

- Frontend issues: Check `website/src/` files
- Backend issues: Check `backend/` README
- Integration issues: Check vercel.json configuration
- Chat widget issues: Check browser console for errors

---

**Successfully integrated! Your textbook now has AI-powered RAG chatbot capabilities.** 📚💬
