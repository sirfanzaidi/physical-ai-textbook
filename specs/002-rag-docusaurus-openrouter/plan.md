# Implementation Plan: RAG Chatbot with Docusaurus & OpenRouter

**Branch**: `002-rag-docusaurus-openrouter` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)

## Summary

Integrate a Retrieval-Augmented Generation (RAG) chatbot into the Physical AI & Humanoid Robotics Docusaurus textbook using OpenRouter API for embeddings and LLM completions. Users can ask questions about book content from any page via a floating widget, with optional support for asking about selected text passages. The backend ingestion pipeline extracts markdown from Docusaurus, chunks semantically, generates embeddings via OpenRouter Qwen model, and stores in Qdrant Cloud. Responses stream in real-time with source citations.

---

## Technical Context

**Language/Version**: Python 3.11+ (FastAPI backend), TypeScript/React (Docusaurus widget)

**Primary Dependencies**:
- Backend: FastAPI, uvicorn, LangChain or LlamaIndex (RAG pipeline), OpenRouter Python SDK, qdrant-client
- Frontend: React (Docusaurus v3+), Highlight.js or native window.getSelection (text selection), axios (API calls)

**Storage**:
- Primary: Qdrant Cloud (vector database for embeddings + chunk metadata)
- Optional: Neon Serverless Postgres (persistent chat history and source metadata)

**Testing**: pytest (backend unit/integration), Jest/React Testing Library (frontend component tests)

**Target Platform**: Web (Docusaurus site on GitHub Pages; FastAPI backend on Render/Fly.io/Vercel)

**Project Type**: Web application (separate frontend/backend repositories or monorepo with workspaces)

**Performance Goals**:
- Query latency (p95): <5 seconds
- Ingestion throughput: Full Docusaurus site (<100K vectors) in <30 minutes
- Concurrent users: ≥10 simultaneous queries without degradation
- Embedding generation: <2 seconds per query via OpenRouter

**Constraints**:
- Free-tier resource limits (Qdrant Cloud free tier, OpenRouter free credits)
- No real-time book updates (manual re-ingestion required)
- Select-text minimum: 10 characters (FR-008a)
- No hallucinations outside book content (zero-leakage RAG)

**Scale/Scope**: Single published book (Physical AI textbook ~200-500 pages), estimated 50K-100K semantic chunks, <100K vectors in Qdrant

---

## Constitution Check

✅ **No violations detected** - Project adheres to all architectural principles:
- Clean separation: Frontend (Docusaurus), Backend (FastAPI), External Services (OpenRouter, Qdrant)
- Security: API keys in `.env`, no client-side credentials
- Testability: Clear API contracts, mockable external services
- Maintainability: Well-defined responsibilities per component

---

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-docusaurus-openrouter/
├── spec.md                    # Feature specification (completed)
├── plan.md                    # This file (architecture & design)
├── checklists/
│   └── requirements.md        # Quality validation (passed)
├── research.md                # Phase 0 research findings
├── data-model.md              # Phase 1 data design
├── api-contracts.md           # Phase 1 API specifications
├── quickstart.md              # Phase 1 onboarding guide
└── tasks.md                   # Phase 2 task breakdown (/sp.tasks output)
```

### Source Code (repository structure)

```text
# Backend (FastAPI RAG service)
backend/
├── app/
│   ├── main.py                      # FastAPI app initialization
│   ├── config.py                    # Settings (API keys, Qdrant URL, etc.)
│   ├── dependencies.py              # Dependency injection (OpenRouter, Qdrant clients)
│   ├── api/
│   │   ├── routes/
│   │   │   ├── ingest.py           # POST /api/ingest (book ingestion)
│   │   │   ├── chat.py             # POST /api/chat (Q&A queries)
│   │   │   └── health.py           # GET /api/health (liveness)
│   │   └── models.py               # Pydantic request/response schemas
│   ├── services/
│   │   ├── ingestion/
│   │   │   ├── docusaurus_crawler.py    # Extract markdown from Docusaurus build
│   │   │   ├── chunker.py               # Semantic chunking (800-1200 chars)
│   │   │   └── pipeline.py              # Orchestrate ingest flow
│   │   ├── embedding/
│   │   │   └── openrouter_client.py     # OpenRouter embedding API client
│   │   ├── retrieval/
│   │   │   ├── qdrant_store.py          # Qdrant vector DB operations
│   │   │   └── reranker.py              # Optional: Re-ranking retrieved chunks
│   │   └── generation/
│   │       └── rag_chat.py              # RAG pipeline + OpenRouter LLM calls
│   ├── utils/
│   │   ├── logging.py                   # Structured logging
│   │   ├── errors.py                    # Custom exceptions
│   │   └── validators.py                # Input validation (selected_text 10-char min)
│   └── db/
│       └── postgres.py                  # Optional: Neon Postgres client (chat history)
├── tests/
│   ├── unit/
│   │   ├── test_chunker.py
│   │   ├── test_embeddings.py
│   │   └── test_validators.py
│   ├── integration/
│   │   ├── test_ingest_pipeline.py
│   │   └── test_rag_query.py
│   └── fixtures/
│       └── sample_books.py
├── requirements.txt              # Python dependencies
├── pyproject.toml               # Project metadata
└── .env.example                 # Environment template (never commit .env)

# Frontend (Docusaurus widget)
website/
├── src/
│   ├── components/
│   │   ├── RAGChat/
│   │   │   ├── index.tsx                # Main chat widget component
│   │   │   ├── ChatWindow.tsx           # Chat message display
│   │   │   ├── InputBox.tsx             # Query input + send button
│   │   │   ├── FloatingButton.tsx       # Floating action button
│   │   │   ├── TextSelectionHandler.ts  # Capture selected text (window.getSelection)
│   │   │   └── RAGChat.module.css       # Widget styling
│   │   └── ...existing Docusaurus components
│   ├── services/
│   │   ├── apiClient.ts                 # Axios wrapper for /api/chat calls
│   │   └── textSelection.ts             # Text selection utility functions
│   ├── hooks/
│   │   └── useRAGChat.ts                # Custom hook for chat state management
│   └── config/
│       └── ragConfig.ts                 # Backend URL, model config (already exists)
├── static/
│   ├── chat-widget.js                   # Vanilla JS widget loader (if not using React)
│   ├── chat-widget.css                  # Standalone widget styles
│   └── utils.js                         # API client utilities
├── docs/                                # Existing Docusaurus documentation
├── docusaurus.config.js                 # Docusaurus configuration (add swizzle)
├── package.json                         # Node dependencies
└── .env.example                         # Frontend env template (API base URL)
```

**Structure Decision**: Web application with separate backend (Python/FastAPI) and frontend (React/Docusaurus). Both can be deployed independently. Monorepo structure allows shared utilities (e.g., Pydantic models → TypeScript types) if needed later.

---

## Architectural Decisions

### AD-001: OpenRouter as Unified AI Provider

**Decision**: Use OpenRouter API for both embeddings (Qwen 3 Embedding 4B) and chat completions (Qwen 3 8B or stronger model).

**Rationale**:
- Single API key/endpoint simplifies deployment and reduces configuration complexity
- OpenRouter abstracts model switching (can swap Qwen → Claude → LLaMA without backend code changes)
- Cost-effective for free tier (Docusaurus site likely <100K vectors)
- Avoids vendor lock-in with multiple API providers

**Alternatives Rejected**:
- Separate providers (Cohere, HuggingFace): More configuration burden, separate rate limits
- Local embedding models (SentenceTransformers): Requires server-side compute (not free-tier friendly)
- Claude API directly: Vendor lock-in, no embedding model

**Implications**:
- Single OpenRouter API key required; must be stored in `.env`
- Fallback/failover strategy needed if OpenRouter is down

---

### AD-002: Qdrant Cloud for Vector Storage

**Decision**: Use Qdrant Cloud Free Tier for vector database.

**Rationale**:
- Free tier supports ~100K vectors (sufficient for single book)
- Low operational overhead (managed cloud service)
- Good semantic search performance for RAG workloads
- Python client (qdrant-client) is mature and well-documented

**Alternatives Rejected**:
- Self-hosted Qdrant: Requires server; free tier deployment options limited
- Pinecone: Cloud-hosted, but free tier more restrictive; vendor lock-in
- Weaviate: More complex deployment; steeper learning curve

**Implications**:
- Cluster URL and API key required (from Qdrant Cloud console)
- Collection schema must be defined upfront (vector dimensions, metadata fields)
- No built-in backup; manual export/snapshot recommended before deploys

---

### AD-003: LangChain/LlamaIndex for RAG Orchestration

**Decision**: Use LangChain or LlamaIndex for RAG pipeline (chunking, retrieval, prompt augmentation).

**Rationale**:
- Abstracts OpenRouter API interactions (RetrieverQA, LLMChain patterns)
- Handles streaming responses (for real-time UI updates)
- Built-in memory management and prompt templates
- Active community and extensive documentation

**Alternatives Rejected**:
- Raw OpenRouter API calls: More boilerplate, harder to maintain
- Custom RAG pipeline: Reinvents wheels (chunking, prompt engineering, streaming)

**Implications**:
- Additional dependency; requires understanding of LangChain patterns
- May need custom loaders for Docusaurus markdown format
- Streaming integration must be tested thoroughly

---

### AD-004: GitHub Pages + Separate FastAPI Backend

**Decision**: Keep Docusaurus on GitHub Pages (static hosting); host FastAPI backend separately (Render, Fly.io, or Vercel Functions).

**Rationale**:
- GitHub Pages: Free, automatic deployment on push, low maintenance
- Separate backend: Allows independent scaling, API changes don't require site rebuild
- CORS: Frontend and backend on different origins → need proper CORS headers (already implemented in Phase 1)

**Alternatives Rejected**:
- Monolithic deployment (backend + frontend together): More complex build, harder to iterate independently
- Backend on GitHub Pages: GitHub Pages doesn't support Python; would need Node.js wrapper

**Implications**:
- CORS headers must be configured on FastAPI (already done: `allow_origins` from `.env`)
- API base URL must be configurable per environment (dev: localhost:8000, prod: FastAPI domain)
- SSL/TLS required for production (handled by deployment platform)

---

## Data Model

### Entities

**Chunk**:
- `id` (UUID): Unique identifier
- `chapter_id` (str): Chapter or doc name from Docusaurus
- `content` (str): 800-1200 char text segment
- `embedding` (array[float]): 1024-dim vector from OpenRouter Qwen
- `metadata` (dict): `{page_num, section_name, source_url, chapter_title}`
- `created_at` (datetime): Ingestion timestamp

**Query**:
- `id` (UUID): Unique query ID
- `query_text` (str): User's question (1-5000 chars, FR-001)
- `mode` (enum): "full" | "selected"
- `selected_text` (str, optional): Highlighted passage (min 10 chars, FR-008a)
- `book_id` (str): Which book (always "physical-ai-textbook" for MVP)
- `response_text` (str): RAG-generated answer
- `citations` (array[Citation]): Source chunks with metadata
- `latency_ms` (float): Query completion time
- `created_at` (datetime): Query timestamp

**Citation**:
- `chunk_id` (UUID): Reference to source chunk
- `chapter_name` (str): Chapter title
- `section_name` (str): Section within chapter
- `text_snippet` (str): ~200 char excerpt from chunk
- `source_url` (str): Link to Docusaurus page

**Session** (Optional, Neon Postgres):
- `id` (UUID): Session ID
- `queries` (array[UUID]): List of query IDs in session
- `created_at` (datetime)
- `updated_at` (datetime)

### Qdrant Collection Schema

```json
{
  "collection_name": "book_vectors",
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  },
  "payload_schema": {
    "chunk_id": { "type": "keyword" },
    "chapter_id": { "type": "keyword" },
    "chapter_name": { "type": "text" },
    "section_name": { "type": "text" },
    "content": { "type": "text" },
    "source_url": { "type": "keyword" },
    "page_num": { "type": "integer" }
  }
}
```

---

## API Contracts

### Ingestion Endpoint

**POST /api/ingest**

**Purpose**: Upload Docusaurus markdown, chunk, embed, and upsert to Qdrant

**Request**:
```json
{
  "source": "docusaurus_build",  // or "file" for manual upload
  "book_id": "physical-ai-textbook",
  "docs_path": "./docs",  // Path to docs folder in build output
  "chunk_size": 800,      // Chars per chunk (default 800-1200)
  "overlap": 150          // Char overlap between chunks
}
```

**Response** (success):
```json
{
  "status": "success",
  "chunks_created": 8742,
  "vectors_upserted": 8742,
  "ingestion_time_seconds": 420,
  "message": "Docusaurus content indexed successfully"
}
```

**Response** (error):
```json
{
  "status": "error",
  "error_code": "INVALID_DOCS_PATH",
  "detail": "Docs path not found: ./docs"
}
```

### Chat Endpoint

**POST /api/chat-stream**

**Purpose**: Retrieve relevant chunks, augment prompt, stream LLM response

**Request**:
```json
{
  "query": "What is the main difference between forward and inverse kinematics?",
  "book_id": "physical-ai-textbook",
  "mode": "full",  // "full" or "selected"
  "selected_text": null,  // Required if mode="selected"; min 10 chars
  "top_k": 5  // Number of chunks to retrieve (default 5)
}
```

**Response** (streaming, JSON-lines format):
```
{"status": "retrieving"}
{"chunks_retrieved": 5}
{"response": "Forward kinematics "}
{"response": "computes the position "}
{"response": "and orientation "}
...
{"citations": [{"chunk_id": "...", "chapter_name": "Kinematics", ...}]}
{"latency_ms": 2340}
```

### Health Endpoint

**GET /api/health**

**Response**:
```json
{
  "status": "healthy",
  "services": {
    "qdrant": "connected",
    "openrouter": "connected"
  }
}
```

---

## Implementation Phases

### Phase 0: Research & Discovery (Week 1)

**Deliverables**:
1. Research OpenRouter API pricing, rate limits, model availability
2. Validate Qdrant Cloud free tier capacity (vector limits, collections)
3. Document Docusaurus markdown extraction approach (crawler vs. build output)
4. Prototype embedding generation via OpenRouter SDK
5. Create `research.md` with findings

**Key Questions to Resolve**:
- [ ] Exact vector dimension from Qwen embedding model?
- [ ] OpenRouter free tier credit limits and overage costs?
- [ ] Docusaurus build output format (markdown or HTML)?
- [ ] Best chunking approach for code-heavy textbook?

**Exit Criteria**: All research questions answered, OpenRouter account created with API key, Qdrant cluster provisioned

---

### Phase 1: Backend Infrastructure (Week 2-3)

**Deliverables**:

1. **FastAPI App Setup**
   - Create `app/main.py` with CORS, logging, error handling
   - Config management (`.env`, environment-based settings)
   - Health check endpoint

2. **Ingestion Pipeline**
   - `app/services/ingestion/docusaurus_crawler.py`: Extract markdown from Docusaurus build
   - `app/services/ingestion/chunker.py`: Semantic chunking (800-1200 chars, respects code blocks/tables)
   - `app/services/embedding/openrouter_client.py`: OpenRouter API wrapper for embeddings
   - `app/services/retrieval/qdrant_store.py`: Qdrant upsert operations
   - `app/api/routes/ingest.py`: Ingestion endpoint with validation

3. **RAG Pipeline**
   - `app/services/retrieval/qdrant_store.py`: Semantic search (retrieve top-K chunks)
   - `app/services/generation/rag_chat.py`: LangChain/LlamaIndex RAG orchestration
   - `app/api/routes/chat.py`: Chat endpoint with streaming response
   - `app/utils/validators.py`: Input validation (selected_text 10-char min, query length limits)

4. **Data & Error Handling**
   - Pydantic models for request/response schemas
   - Custom exceptions for RAG-specific errors (no relevant content, API unavailable)
   - Structured logging (query latency, error traces)

5. **Unit Tests**
   - Test chunker with edge cases (code blocks, tables, long lines)
   - Test embedding client (mocked OpenRouter API)
   - Test validators (selected_text validation)
   - Test RAG pipeline end-to-end

**Exit Criteria**:
- FastAPI app runs locally with `/api/health` responding
- Ingestion pipeline successfully chunks sample Docusaurus content
- Chat endpoint retrieves and responds to queries (streaming works)
- All unit tests passing
- API contracts documented (`api-contracts.md`)

---

### Phase 2: Frontend Widget (Week 3-4)

**Deliverables**:

1. **React Chat Component**
   - `components/RAGChat/index.tsx`: Main component with state management
   - `components/RAGChat/ChatWindow.tsx`: Render messages and citations
   - `components/RAGChat/InputBox.tsx`: Query input + send button
   - `components/RAGChat/FloatingButton.tsx`: Floating action button to open/close chat

2. **Text Selection Handler**
   - `components/RAGChat/TextSelectionHandler.ts`: window.getSelection() integration
   - Detect user text selection on page
   - Show "Ask about this" button near selected text
   - Pass selected_text to backend query

3. **API Client**
   - `services/apiClient.ts`: Axios wrapper for `/api/chat-stream` calls
   - Handle streaming response (Server-Sent Events or fetch streaming)
   - Parse JSON-lines response format

4. **Styling**
   - `RAGChat.module.css`: Chat widget UI (dark mode support)
   - Responsive design (mobile-friendly)
   - No layout shift on Docusaurus page

5. **Integration Tests**
   - Test text selection capture
   - Test API calls (mocked backend)
   - Test streaming response parsing
   - Test widget rendering without errors

**Exit Criteria**:
- Floating button appears on all Docusaurus pages
- Chat opens/closes smoothly
- Text selection detection works (tested on multiple browsers)
- API calls reach local backend
- Widget doesn't break Docusaurus styling or navigation
- All component tests passing

---

### Phase 3: Docusaurus Integration (Week 4-5)

**Deliverables**:

1. **Theme Component Swizzle**
   - Swizzle Docusaurus Footer or Layout component
   - Inject floating button globally on all pages
   - Configure backend URL from `docusaurus.config.js`

2. **Environment Configuration**
   - `website/src/config/ragConfig.ts`: Backend URL per environment (dev: localhost:8000, prod: deployed API)
   - Environment detection (localhost vs. vercel.app domain)

3. **Documentation**
   - `quickstart.md`: How to build Docusaurus, run ingestion, test chatbot locally
   - Configuration guide: Setting OpenRouter key, Qdrant cluster details
   - Troubleshooting: Common issues (CORS, API unavailable, no chunks found)

4. **CI/CD**
   - GitHub Actions workflow: Auto-build Docusaurus on push
   - Optional: Auto-deploy FastAPI backend on push (Render/Fly.io)
   - Optional: Auto-ingest updated Docusaurus build to Qdrant (manual trigger or scheduled)

**Exit Criteria**:
- Floating widget appears on production Docusaurus site
- Widget communicates with production FastAPI backend
- CORS headers correctly configured (no browser warnings)
- Ingestion completed for full book (all chapters indexed)
- Citations link correctly to Docusaurus pages

---

### Phase 4: Testing, Polish & Launch (Week 5-6)

**Deliverables**:

1. **Blind Testing**
   - 50+ unseen test queries from domain experts
   - Measure accuracy (target ≥90%)
   - Identify failure modes (chunking issues, retrieval gaps, hallucinations)

2. **Performance Testing**
   - Latency under normal load (<5s p95)
   - Concurrent user load testing (≥10 simultaneous)
   - Embedding API rate limit handling

3. **Security Review**
   - No hardcoded credentials, all in `.env`
   - CORS properly restricted to known origins
   - Input validation against injection attacks
   - API key rotation strategy

4. **Documentation & Runbooks**
   - Deployment guide (Render/Fly.io)
   - Troubleshooting runbook (API errors, Qdrant connection issues)
   - Model configuration guide (swapping embedding/LLM models)

5. **Polish**
   - UX improvements based on user testing feedback
   - Error message clarity and helpfulness
   - Accessibility (ARIA labels, keyboard navigation)
   - Mobile responsiveness polish

**Exit Criteria**:
- ≥90% accuracy on blind tests
- p95 latency <5 seconds under 10 concurrent users
- Security review passed
- Documentation complete
- Ready for production launch

---

## Risk Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|-----------|
| OpenRouter API rate limits during ingestion | Medium | High | Implement exponential backoff retry logic; batch requests; estimate ingestion time upfront |
| Qdrant free tier capacity exceeded (>100K vectors) | Low | High | Monitor chunk count during ingestion; implement pagination/archival if needed; warn on setup |
| Poor chunking breaks semantic search | Medium | Medium | Test with code-heavy chapters; adjust chunk size/overlap based on blind test results |
| Text selection not working on some browsers | Low | Medium | Test on Chrome, Firefox, Safari, Edge; use fallback to full-book mode if selection fails |
| OpenRouter LLM hallucinations (answers outside book) | Medium | High | Enforce strict retrieval constraints; test answer grounding; use strong models; document limitations |
| CORS issues blocking widget on production | Low | High | Test with production domain early; verify `allow_origins` configuration |

---

## Complexity Assessment

**No constitution violations.** Project adheres to architectural principles:
- ✅ Clear separation of concerns (frontend, backend, services)
- ✅ No hardcoded secrets (all in `.env`)
- ✅ Testable components (mocked API clients)
- ✅ Documented contracts (API schemas)

---

## Next Steps

1. **Phase 0 (Research)**: Execute research tasks, validate assumptions, document findings in `research.md`
2. **Phase 1 (Backend)**: Implement FastAPI app, ingestion, RAG pipeline, chat endpoint
3. **Phase 2 (Frontend)**: Build React widget, text selection handler, API integration
4. **Phase 3 (Integration)**: Swizzle Docusaurus, configure environments, set up CI/CD
5. **Phase 4 (Testing)**: Blind testing, performance validation, security review, launch

**Estimated Timeline**: 6 weeks (1 week research, 3 weeks development, 2 weeks testing/polish)

**Parallel Work**: Frontend and backend can be developed in parallel after Phase 1 design is complete.

---

## Success Metrics (from Spec)

- ✅ SC-001: ≥90% accuracy on 50+ blind test queries
- ✅ SC-002: Select-text mode zero-leakage retrieval
- ✅ SC-003: p95 latency <5 seconds
- ✅ SC-004: Full-book ingestion <30 minutes
- ✅ SC-005: Valid source citations linking to Docusaurus pages
- ✅ SC-006: Security review passed (HTTPS, env vars, CORS)
- ✅ SC-007: ≥10 concurrent users supported
- ✅ SC-008: ≥80% positive user feedback

---

**Plan Status**: ✅ Ready for Phase 0 execution and `/sp.tasks` phase breakdown
