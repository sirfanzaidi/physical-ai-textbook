# Quick Test Guide - RAG Chatbot Integration

## Pre-Test Cleanup (Important!)

```javascript
// Open browser DevTools console and run:
localStorage.clear();
location.reload();
```

Then hard refresh:
- **Windows/Linux**: `Ctrl+Shift+R`
- **Mac**: `Cmd+Shift+R`

---

## Step 1: Start Backend

```bash
# From root directory
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
uvicorn backend.app.main:app --reload
```

**Expected output:**
```
INFO:     Uvicorn running on http://127.0.0.1:8000
INFO:     Application startup complete
```

---

## Step 2: Upload a Test Book

```bash
# In another terminal
curl -X POST http://localhost:8000/api/ingest \
  -F "file=@frontend/demo.html" \
  -F "book_id=physical-ai-textbook"
```

**Expected response:**
```json
{
  "success": true,
  "book_id": "physical-ai-textbook",
  "chunk_count": 2,
  "total_tokens": 123,
  "message": "Successfully ingested..."
}
```

---

## Step 3: Start Docusaurus Site

```bash
cd website
npm run start
```

**Expected output:**
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus server started on http://localhost:3000
```

---

## Step 4: Test the Chat Page

### Open Chat Page

1. Go to http://localhost:3000
2. Click **"💬 Ask RAG Bot"** in the navbar
3. Or navigate directly to http://localhost:3000/chat

### Check Console (DevTools)

- **Open**: `F12` or `Cmd+Option+I`
- **Go to**: Console tab
- **Should see**: No red errors about "Identifier already declared"
- **Should see**: Chat widget initializing messages

### Test the Widget

1. **Type a question**: "What is this about?"
2. **Submit**: Press Enter or click Send
3. **Watch for**:
   - ✅ Response starts streaming
   - ✅ Text appears character-by-character
   - ✅ No JavaScript errors in console
   - ✅ Response completes with citations
   - ✅ Confidence score displays

### Test Features

**Message History:**
- Reload page (`F5`)
- Previous messages should be visible
- ✅ Conversation persists

**Dark Mode:**
- Use system setting or browser extension
- Widget should adapt colors
- ✅ Dark mode works

**Mobile View:**
- DevTools → Toggle device toolbar (`Ctrl+Shift+M`)
- Resize to 480px width
- ✅ Widget is responsive

---

## Step 5: Check API Connection

In browser console:

```javascript
// Check if backend is reachable
fetch('http://localhost:8000/api/health')
  .then(r => r.json())
  .then(d => console.log('Health:', d));
```

**Expected output:**
```
Health: {
  status: "healthy",
  environment: "development",
  services: {
    cohere: "connected",
    qdrant: "connected",
    neon: "not_configured"
  }
}
```

---

## Verification Checklist

- [ ] No "Identifier already declared" errors
- [ ] No 404 errors for `/chat-widget.js`, `/utils.js`, `/chat-widget.css`
- [ ] Chat page loads at `/chat`
- [ ] Can type and submit a question
- [ ] Response streams in real-time
- [ ] Citations display with format "[1] (Chapter X, p. Y)"
- [ ] Confidence score shows (0-100%)
- [ ] Message history persists on page reload
- [ ] Dark mode works
- [ ] Mobile responsive
- [ ] Select-text "Ask about this" button appears when text selected

---

## Troubleshooting

### "Identifier already declared" Error

**Solution:**
```javascript
// In console:
localStorage.clear();
// Then hard refresh (Ctrl+Shift+R)
```

### Chat widget doesn't show

**Check:**
1. DevTools → Network tab
2. Look for `chat-widget.js`, `utils.js`, `chat-widget.css`
3. Should be 200 (not 404)
4. If 404, files not in `website/static/`

**Fix:**
```bash
cp frontend/chat-widget.js website/static/
cp frontend/utils.js website/static/
cp frontend/chat-widget.css website/static/
```

### "Cannot connect to API"

**Check:**
1. Backend is running: `http://localhost:8000/api/health`
2. CORS not blocking (check Network tab, look for CORS errors)
3. API URL is correct in `ragConfig.ts` (should be `http://localhost:8000` for dev)

**Fix:**
```typescript
// website/src/config/ragConfig.ts
// Check isDevelopment detection:
const isDevelopment = window.location.hostname === 'localhost';
console.log('isDevelopment:', isDevelopment);
```

### Book not found in chat

**Check:**
1. Book was indexed: Run ingest curl command again
2. book_id matches: Should be `physical-ai-textbook`
3. Query is related to content: Try "What is this about?"

**Fix:**
```bash
# Re-index the book
curl -X POST http://localhost:8000/api/ingest \
  -F "file=@frontend/demo.html" \
  -F "book_id=physical-ai-textbook"
```

---

## Performance Expectations

| Metric | Target | Typical |
|--------|--------|---------|
| Page Load | <2s | 0.5-1s |
| Widget Init | <1s | 0.2-0.5s |
| First Response | <5s | 2-4s |
| Stream Chunks | Real-time | 0.01s between chunks |

---

## Advanced: Manual Testing

### Test Hot Reload

1. Edit `website/src/pages/chat.tsx` (add a space somewhere)
2. Save the file
3. Watch page reload in browser
4. Chat should still work
5. ✅ No duplicate declaration errors

### Test Multiple Components

Edit `website/src/pages/chat.tsx`:
```tsx
// Add a second chat widget below the first
<RAGChat
  bookId="physical-ai-textbook"
  height="300px"
/>
```

Both widgets should work independently.

### Test API Latency

In console:
```javascript
// Time a query
const start = Date.now();
fetch('http://localhost:8000/api/chat', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    query: 'What is this?',
    book_id: 'physical-ai-textbook',
    mode: 'full'
  })
}).then(r => r.json()).then(d => {
  console.log('Latency:', Date.now() - start, 'ms');
  console.log('Response:', d);
});
```

---

## Next Steps

✅ **If all tests pass:**
1. You're ready to deploy!
2. See `INTEGRATION_GUIDE.md` for production setup
3. Update backend URL in `vercel.json` when deploying

⚠️ **If tests fail:**
1. Check troubleshooting section above
2. Review `INTEGRATION_FIX.md` for technical details
3. Check browser console for specific error messages

---

## Support

- 📖 Full integration guide: `INTEGRATION_GUIDE.md`
- 🔧 Technical fix details: `INTEGRATION_FIX.md`
- 📚 Main README: `README.md`
- 🚀 Backend quickstart: `backend/QUICKSTART.md`

---

**Happy testing! 🎉**
