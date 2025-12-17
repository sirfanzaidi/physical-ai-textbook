# Backend Embedding Fix - Step-by-Step Guide

## Problem Summary

The chat widget is working, but responses are empty because the Cohere API changed its response format.

**Error:** `'EmbedByTypeResponseEmbeddings' object is not subscriptable`

**Location:** `backend/ingestion/embedder.py` (lines 63 and 100)

---

## The Issue Explained

### Old Code (Broken)
```python
response = self.client.embed(texts=[...], model="embed-v4.0")
embeddings = response.embeddings  # ❌ Tries to use as list
return embeddings  # ❌ Fails because it's an object, not a list
```

### New Cohere API
The Cohere API now returns a response object where `response.embeddings` is an object with attributes, not a direct list.

**Error Message:**
```
'EmbedByTypeResponseEmbeddings' object is not subscriptable
```

This means you can't do `response.embeddings[0]` like the code tries to do.

---

## How to Fix It

### Step 1: Open the File

```
File: backend/ingestion/embedder.py
```

### Step 2: Find the Problem Areas

There are **2 places** that need fixing:

#### **Location 1: `embed_texts` method (around line 56-71)**

**Current code:**
```python
try:
    response = self.client.embed(
        texts=texts,
        model=self.model,
        input_type="search_document",
    )

    embeddings = response.embeddings  # ← LINE 63 - PROBLEM HERE

    logger.info(...)
    return embeddings

except Exception as e:
    raise EmbeddingError(...)
```

**Fixed code:**
```python
try:
    response = self.client.embed(
        texts=texts,
        model=self.model,
        input_type="search_document",
    )

    # Handle different response formats
    if hasattr(response, 'embeddings'):
        emb = response.embeddings
        # Check if it's iterable (list or object with __iter__)
        if isinstance(emb, list):
            embeddings = emb
        elif hasattr(emb, '__iter__'):
            embeddings = list(emb)
        else:
            # Try to get data attribute
            embeddings = getattr(emb, 'data', [])
    else:
        embeddings = []

    logger.info(...)
    return embeddings

except Exception as e:
    raise EmbeddingError(...)
```

#### **Location 2: `embed_query` method (around line 93-107)**

**Current code:**
```python
try:
    response = self.client.embed(
        texts=[query],
        model=self.model,
        input_type="search_query",
    )

    embedding = response.embeddings[0]  # ← LINE 100 - PROBLEM HERE

    logger.info(...)
    return embedding

except Exception as e:
    raise EmbeddingError(...)
```

**Fixed code:**
```python
try:
    response = self.client.embed(
        texts=[query],
        model=self.model,
        input_type="search_query",
    )

    # Handle different response formats
    if hasattr(response, 'embeddings'):
        emb = response.embeddings
        # Convert to list if needed
        if isinstance(emb, list):
            embeddings = emb
        elif hasattr(emb, '__iter__'):
            embeddings = list(emb)
        else:
            embeddings = getattr(emb, 'data', [])

        # Return first embedding or empty list
        embedding = embeddings[0] if embeddings else []
    else:
        embedding = []

    logger.info(...)
    return embedding

except Exception as e:
    raise EmbeddingError(...)
```

---

## Step 3: Test the Fix

### Restart Backend

```bash
# Kill old backend process
# Then restart:
cd "D:\physical-ai-textbook"
python -m uvicorn backend.app.main:app --reload
```

### Test Embedding

```bash
# Try to ingest a book
curl -X POST http://localhost:8000/api/ingest \
  -F "file=@frontend/demo.html" \
  -F "book_id=physical-ai-textbook"
```

**Expected:** Success response (not "Failed to generate embeddings" error)

### Test Chat

In the browser chat widget:
```
Ask: "What is this about?"
Expected: Should get a response (may be generic if book is empty)
```

---

## Alternative: Downgrade Cohere Package

If you don't want to modify the code, try downgrading to an older Cohere version:

```bash
pip install cohere==4.0.0
```

Or check what version you have:

```bash
pip show cohere | grep Version
```

---

## Verification Checklist

After fixing:

- [ ] Backend restarts without errors
- [ ] No "Failed to generate embeddings" errors
- [ ] Book ingestion works: `curl -X POST /api/ingest ...`
- [ ] Chat responses are no longer empty
- [ ] Citations appear in chat
- [ ] Confidence scores show

---

## If You Get Different Errors

### Error: `module 'cohere' has no attribute 'ClientV2'`

**Fix:** Update Cohere package
```bash
pip install --upgrade cohere
```

### Error: `response has no attribute 'embeddings'`

**Fix:** The Cohere API might have changed again. Check the actual attributes:

In Python:
```python
import cohere
client = cohere.ClientV2(api_key="your_key")
response = client.embed(texts=["test"], model="embed-v4.0")
print(dir(response))  # See all attributes
print(response)  # Print the response to inspect
```

### Error: API Key Issues

**Check:**
```bash
# Make sure .env has COHERE_API_KEY set
cat .env | grep COHERE_API_KEY
```

---

## Summary

| Before Fix | After Fix |
|-----------|-----------|
| Embedding fails | Embedding works |
| Can't ingest books | Can ingest books |
| Can't retrieve context | Can retrieve context |
| Chat responses empty | Chat responses populated |
| No citations | Citations appear |

---

## Files to Modify

Only **1 file** needs changes:
- `backend/ingestion/embedder.py`

**Lines to change:**
- ~63 (in `embed_texts` method)
- ~100 (in `embed_query` method)

---

**Status:** Ready to implement
**Difficulty:** Low (straightforward code change)
**Time:** ~5 minutes
**Impact:** High (enables full chat functionality)

---

For questions, check the test output and error messages in the backend logs.
