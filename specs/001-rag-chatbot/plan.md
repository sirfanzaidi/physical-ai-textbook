# Implementation Plan: Integrated RAG Chatbot for Published Books

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-14 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification with 4 user stories, 17 functional requirements (15 + 2 new: FR-001a atomic units, FR-007a select-text validation), 8 success criteria

## Summary

Build a high-quality, embeddable RAG chatbot for published books using Cohere's latest models (embed-v4.0, rerank-v4.0-pro, command-a-03-2025) for accurate, grounded responses with citations. The system supports querying full book content or user-selected text only (zero hallucination guarantee). Powered by FastAPI backend, Qdrant Cloud Free Tier vector storage, and Neon Serverless Postgres metadata. Target: ≥90% accuracy on blind tests, <5s p95 latency, fully embeddable in web-based book viewers.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**:
- FastAPI 0.109+ (web framework)
- Cohere SDK (embed-v4.0, rerank-v4.0-pro, command-a-03-2025)
- qdrant-client (vector search)
- psycopg2-binary (Neon Postgres)
- PyPDF2 or unstructured (PDF parsing)
- pydantic-settings, python-dotenv (configuration)

**Storage**:
- Qdrant Cloud Free Tier (vector embeddings, collection: book_vectors, dimension 1024)
- Neon Serverless PostgreSQL (metadata: chunks_metadata table with id, chunk_id, page_num, text_hash)

**Testing**: pytest (unit/integration), blind eval (50+ queries for accuracy)

**Target Platform**: Linux/macOS/Windows (backend), browser-based (frontend widget)

**Project Type**: Web application (FastAPI backend + Vanilla JS frontend widget)

**Performance Goals**:
- Query latency: p95 < 5 seconds
- Blind test accuracy: ≥90% of responses grounded in book
- Indexing: 500-page book in <5 minutes
- Concurrent queries: Support 10-50 simultaneous users (free-tier baseline)

**Constraints**:
- Free-tier only (Qdrant Cloud Free Tier, Neon free tier, Vercel free tier)
- 500-page book maximum per index
- Cohere API exclusive (no OpenAI, no local models for generation)
- No hardcoded credentials (all via .env)
- Select-text mode: zero leakage from non-selected content

**Scale/Scope**:
- Single book per API instance (MVP)
- 1-3 concurrent users (free-tier baseline)
- 50,000 chunks maximum (free-tier Qdrant limit)

## Constitution Check

**Principles Evaluated**:
1. ✅ **Functionality & Usability** — Accurate RAG with select-text support, <5s latency
2. ✅ **Integration & Scalability** — Stateless FastAPI, embeddable widget, serverless infrastructure
3. ✅ **Documentation** — Inline comments, Swagger UI, README, architecture diagrams
4. ✅ **API Usage & Budget** — Cohere exclusive, Qdrant Cloud Free, Neon free tier, monthly monitoring
5. ✅ **Accuracy & Relevance (NON-NEGOTIABLE)** — 90%+ blind test target, zero hallucination, grounded responses
6. ✅ **Security & Privacy** — .env credentials, HTTPS production, no PII in logs
7. ✅ **Innovation with Tools** — SpecifyKit for structured dev, Claude for code generation, PHRs for traceability
8. ✅ **Technology Stack** — FastAPI, Qdrant Cloud, Neon, Cohere, Vanilla JS

**Gate Status**: ✅ **PASS** — All 8 principles aligned. No violations. Ready for Phase 0 Research.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── spec.md                      # Feature specification (complete)
├── plan.md                      # This file (implementation plan)
├── research.md                  # Phase 0: Research & unknowns resolution
├── data-model.md                # Phase 1: Entity design & database schema
├── contracts/                   # Phase 1: API endpoint specifications
│   ├── ingest.md               # POST /ingest endpoint contract
│   ├── query.md                # POST /query endpoint contract
│   └── query-stream.md         # POST /query-stream endpoint contract
├── quickstart.md                # Phase 1: Setup & local dev guide
├── checklists/
│   └── requirements.md          # Specification quality validation
├── .env.example                 # Configuration template (credentials template)
└── README.md                    # Project overview & architecture guide
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── __init__.py
│   ├── main.py                          # FastAPI app entry
│   ├── config.py                        # Settings & environment loading
│   ├── middleware.py                    # CORS, logging, error handlers
│   └── api/
│       ├── __init__.py
│       ├── routes.py                    # /ingest, /chat, /chat-stream, /health
│       └── models.py                    # Pydantic request/response schemas
├── ingestion/
│   ├── __init__.py
│   ├── main.py                          # Book upload & chunking pipeline
│   ├── chunker.py                       # Semantic chunking (300-500 tokens)
│   ├── embedder.py                      # Cohere embed-v4.0 integration
│   └── storage.py                       # Qdrant upsert + Neon metadata
├── retrieval/
│   ├── __init__.py
│   ├── retriever.py                     # Query embedding + vector search
│   ├── reranker.py                      # Cohere rerank-v4.0-pro
│   └── augmenter.py                     # Document assembly for RAG
├── generation/
│   ├── __init__.py
│   ├── rag_chat.py                      # Cohere Chat API with documents
│   └── prompts.py                       # System prompts for grounded generation
├── database/
│   ├── __init__.py
│   ├── qdrant_client.py                 # Qdrant connection & operations
│   └── postgres_client.py               # Neon metadata storage
├── utils/
│   ├── __init__.py
│   ├── logging.py                       # Structured logging
│   ├── errors.py                        # Custom exceptions
│   └── metrics.py                       # Performance & accuracy tracking
└── tests/
    ├── unit/
    │   ├── test_chunker.py             # Chunking logic
    │   ├── test_embedder.py            # Cohere embed API
    │   ├── test_retriever.py           # Vector search + reranking
    │   └── test_rag_chat.py            # Generation logic
    ├── integration/
    │   ├── test_ingest_to_query.py     # End-to-end: upload → index → query
    │   ├── test_select_text_mode.py    # Select-text isolation (zero-leakage)
    │   └── test_concurrent_queries.py  # Latency & throughput
    └── blind_eval/
        └── test_accuracy_50_queries.py # 50+ blind test queries for accuracy

frontend/
├── chat-widget.js                       # Vanilla JS chat widget
├── chat-widget.css                      # Responsive styling
├── demo.html                            # Example book page with widget embedded
└── utils.js                             # Message formatting, API calls

.env.example                            # Credential template (NEVER commit .env)
requirements.txt                        # Python dependencies
README.md                               # Setup & deployment guide
Makefile or docker-compose.yml          # Local dev orchestration (optional)
```

**Structure Decision**:
Selected **Option 2 (Web Application)** with clear backend/frontend separation. Backend organized by domain (ingestion, retrieval, generation) following hexagonal architecture. Frontend is a lightweight Vanilla JS widget for iframe embedding (no framework overhead). Tests co-located with source code by layer (unit, integration, blind eval) for easy discovery.

## Implementation Phases (7 phases, 2-4 weeks)

### Phase 1: Environment Setup & Connections (Days 1-3)

**Goal**: Establish secure, verified connections to all external services.

**Deliverables**:
- Git repo initialized with directory structure
- Dependencies installed (cohere, qdrant-client, fastapi, uvicorn, psycopg2, PyPDF2)
- .env configured with credentials (never committed)
- Qdrant collection created (book_vectors, dimension 1024)
- Neon metadata table created (chunks_metadata)
- Smoke tests pass for all service connections

**Key Activities**:
1. Initialize git and create directory structure
2. Install Python 3.11+, pip dependencies
3. Configure environment variables from .env.example
4. Create Qdrant collection: `book_vectors` with payload schema (page, section, chapter, text_hash)
5. Create Neon table: `chunks_metadata` (id, chunk_id, page_num, section_name, chapter_name, text_hash, created_at)
6. Write smoke tests: Cohere embed API, Qdrant upsert, Neon INSERT
7. Run smoke tests, verify all connections green

**Success Criteria**:
- ✅ Repo structure complete
- ✅ Dependencies installed
- ✅ All service connections verified (Cohere, Qdrant, Neon)
- ✅ Smoke tests PASS

---

### Phase 2: Book Ingestion Pipeline (Days 4-8)

**Goal**: Accept PDF/text books, chunk semantically, embed with Cohere, store in Qdrant + Neon.

**Deliverables**:
- `ingestion/main.py`: Book upload & chunking
- `ingestion/chunker.py`: Semantic chunking (300-500 tokens, 200-token overlap) with **atomic-unit preservation** (FR-001a)
- `ingestion/embedder.py`: Cohere embed-v4.0 batching
- `ingestion/storage.py`: Qdrant upsert + Neon metadata
- Sample book indexed & verified in Qdrant dashboard
- Unit tests for chunking, embedding, storage (including atomic-unit validation)

**Key Activities**:
1. Implement PyPDF2/unstructured parsing for PDF/text extraction
2. **Implement semantic chunker with atomic-unit preservation (FR-001a)**:
   - Recursive character splitting, preserve boundaries
   - Treat code blocks, tables, equations, lists as **atomic units** (never split)
   - Expand chunk boundary to include entire unit (may exceed 500 tokens) rather than breaking it
   - Post-indexing validation: Verify no code blocks or tables truncated mid-unit
3. Implement Cohere embed-v4.0 integration (input_type="search_document", batch processing)
4. Implement Qdrant upsert: payload schema with text, page, section, chapter
5. Implement Neon metadata logging: chunk_id, page_num, text_hash, created_at
6. Implement re-indexing: delete old chunks, upsert new (idempotent)
7. Test with 2-3 sample books (50-500 pages), including books with code/tables
8. Verify chunks discoverable in Qdrant Dashboard

**Success Criteria**:
- ✅ 2-3 sample books indexed successfully
- ✅ Chunks correctly split (300-500 tokens, or larger if atomic units require)
- ✅ **No code blocks or tables split mid-unit (FR-001a validation)**
- ✅ Embeddings stored in Qdrant with metadata
- ✅ Metadata logged in Neon
- ✅ Re-indexing works without duplication
- ✅ Unit tests PASS (chunking, embedding, storage, atomic-unit preservation)

---

### Phase 3: Advanced Retrieval System (Days 9-13)

**Goal**: Implement accurate retrieval (initial search → rerank) for both full-book and select-text modes.

**Deliverables**:
- `retrieval/retriever.py`: Query embedding + initial vector search
- `retrieval/reranker.py`: Cohere rerank-v4.0-pro (or fast)
- `retrieval/augmenter.py`: Document assembly for RAG
- Support for select-text mode: constrained retrieval
- Retrieval test: >95% relevant chunks in top results on 30+ queries
- Integration tests for both modes

**Key Activities**:
1. Implement query embedding: Cohere embed-v4.0 (input_type="search_query")
2. Implement initial vector search: Qdrant top_k=20-30 with similarity threshold
3. Implement reranking: Cohere rerank-v4.0-pro on top results, final top_k=8-10
4. Implement select-text mode:
   - Accept selected_text parameter + query
   - Temporarily embed selected chunks
   - Use Qdrant filters or metadata matching to isolate retrieval to selected passage only
   - Return results ONLY from selected passage (zero-leakage guarantee)
5. Implement document assembly: List of {id, title, text, source (page, section)} for Cohere Chat
6. Test retrieval: 30+ sample queries, verify relevant chunks in top-8
7. Test select-text: 10+ queries, verify zero leakage from non-selected content

**Success Criteria**:
- ✅ Initial retrieval: >95% recall on 30+ queries
- ✅ Reranking reduces false positives, improves precision
- ✅ Select-text mode: 100% zero-leakage (no results from outside selected passage)
- ✅ Document assembly correct for Cohere Chat API
- ✅ Integration tests PASS (both modes)

---

### Phase 4: FastAPI Backend & RAG Chat (Days 14-19)

**Goal**: Implement REST API endpoints with Cohere Chat for grounded generation.

**Deliverables**:
- `app/main.py`: FastAPI app with CORS, error handling
- `app/api/routes.py`: Endpoints (/ingest, /chat, /chat-stream, /health)
- `app/api/models.py`: Request/response Pydantic schemas
- `generation/rag_chat.py`: Cohere Chat API integration with 'documents' parameter
- `generation/prompts.py`: System prompts enforcing grounding + citations
- API documentation (Swagger UI)
- Error handling, input validation, structured logging

**Key Activities**:
1. Initialize FastAPI app with middleware (CORS, logging, error handlers)
2. Implement POST /ingest:
   - Accept file upload (PDF/text)
   - Validate file (max 500 pages)
   - Trigger ingestion pipeline
   - Return status + chunk count
3. Implement POST /chat:
   - Accept query, mode ("full" | "selected"), selected_text (if mode="selected")
   - **Implement select-text validation (FR-007a)**:
     - If mode="selected", selected_text MUST be ≥10 characters
     - Return HTTP 400 with "Selected text must be at least 10 characters" if <10 chars
     - Return HTTP 400 with "Selected text is required for select-text mode" if empty/null
   - Retrieve documents (retrieval pipeline)
   - Call Cohere Chat with 'documents' parameter
   - Return response + citations
4. Implement POST /chat-stream:
   - Same as /chat but streaming response (for low latency)
5. Implement GET /health: Simple health check
6. Implement system prompt:
   ```
   "You are a helpful assistant answering questions based only on provided documents.
    If the answer is not in the documents, say 'I don't have that information in the book.'
    Always cite sources (chapter, page, or section) when available."
   ```
7. Implement input validation:
   - Query length <5000 chars
   - **Selected text minimum 10 characters (FR-007a)**
   - File size limits, content type validation
8. Implement structured logging: Query, response, latency, accuracy flag
9. Write integration tests: end-to-end query flows (include select-text edge cases)

**Success Criteria**:
- ✅ All 4 endpoints functional + documented
- ✅ Cohere Chat integration with documents + citations
- ✅ System prompt enforces grounding (no hallucinations)
- ✅ Input validation + error handling complete
- ✅ Structured logging captures all queries
- ✅ Integration tests PASS
- ✅ Swagger UI accessible at /docs

---

### Phase 5: Embeddable Frontend Widget (Days 20-23)

**Goal**: Create lightweight Vanilla JS chat widget for iframe embedding.

**Deliverables**:
- `frontend/chat-widget.js`: Chat UI component
- `frontend/chat-widget.css`: Responsive styling
- `frontend/demo.html`: Example book page with embedded widget
- Widget supports message history, select-text → "Ask" feature
- Zero hardcoded API keys (calls backend securely)

**Key Activities**:
1. Implement chat widget (Vanilla JS, no framework):
   - Message display + input field
   - Send query to backend /chat endpoint
   - Display response + citations
   - Support streaming responses
2. Implement select-text feature with validation (FR-007a):
   - Detect text selection in parent document
   - Show "Ask about this" button
   - **Validate selection length**:
     - Reject empty selections
     - Warn users if selection 1–9 characters (optional "too short" indicator)
     - Allow selections ≥10 characters
   - Pass selected_text to backend /chat with mode="selected"
   - Handle backend error responses (HTTP 400 for <10 chars)
3. Implement message history: Store in browser localStorage
4. Implement responsive design: Mobile-friendly, customizable CSS
5. Implement iframe embedding: No API keys in frontend, all calls via backend
6. Create demo.html: Sample book page with embedded widget
7. Test in multiple browsers (Chrome, Firefox, Safari)

**Success Criteria**:
- ✅ Widget loads and renders in iframe
- ✅ Queries send to backend, responses display correctly
- ✅ Select-text feature captures highlight + sends to backend
- ✅ Message history persists in session
- ✅ No hardcoded API keys in frontend
- ✅ Works across browsers and devices

---

### Phase 6: Testing, Optimization & Security (Days 24-27)

**Goal**: Comprehensive testing, accuracy validation, security hardening, performance optimization.

**Deliverables**:
- Unit tests: >80% coverage (chunking, embedding, retrieval, generation)
- Integration tests: End-to-end flows (ingest → query, select-text)
- Blind accuracy test: 50+ unseen queries, ≥90% accuracy target
- Security audit: Input validation, prompt injection prevention, credential protection
- Performance audit: <5s p95 latency, Cohere/Qdrant/Neon usage monitoring
- Optimization report: Chunk size, top_k, rerank model, embedding dimensions

**Key Activities**:
1. Write unit tests:
   - Chunking: Verify token counts, boundary preservation
   - Embedding: Verify Cohere API integration, batch processing
   - Retrieval: Verify top-k relevance, reranking effectiveness
   - Generation: Verify system prompt enforcement, citation format
2. Write integration tests:
   - Ingest → Query pipeline
   - Select-text zero-leakage validation
   - Concurrent query handling
   - Error handling (API failures, timeout)
3. Prepare blind eval dataset: 50+ queries NOT used during development
4. Run blind eval: Evaluate accuracy, citation quality, hallucination detection
5. Security audit:
   - Input validation: Query length, file size, content type
   - Prompt injection: Test with adversarial inputs
   - Credential protection: Verify no keys in logs, no hardcoded secrets
   - HTTPS: Configure for production
6. Performance audit:
   - Measure latency: p50, p95, p99
   - Monitor API usage: Cohere tokens, Qdrant requests, Neon queries
   - Identify bottlenecks (embedding, retrieval, generation)
   - Optimize: Adjust chunk size, top_k, rerank model as needed
7. Generate reports: Test coverage, accuracy metrics, security findings

**Success Criteria**:
- ✅ Unit test coverage: >80%
- ✅ All integration tests PASS
- ✅ Blind eval accuracy: ≥90%
- ✅ Select-text zero-leakage: 100%
- ✅ p95 latency: <5 seconds
- ✅ No security vulnerabilities found
- ✅ No hardcoded credentials in codebase

---

### Phase 7: Documentation & Final Polish (Days 28-30)

**Goal**: Complete documentation, deployment guide, code review, live demo.

**Deliverables**:
- `README.md`: Setup, ingestion, deployment guide
- `quickstart.md`: Local dev quick-start
- `research.md`: Technology research & rationale
- `data-model.md`: Entity design & schemas
- `contracts/`: API endpoint specifications (OpenAPI)
- Code review: PEP 8 compliance, inline comments, error handling
- Demo: Live example with sample book + widget
- Deployment guide: Vercel for backend, static hosting for widget

**Key Activities**:
1. Write comprehensive README:
   - Project overview
   - Setup instructions (Python 3.11+, dependencies)
   - Configuration (.env template)
   - Ingestion guide (upload a book, verify in dashboard)
   - API documentation (reference Swagger UI)
   - Deployment notes (Vercel, GitHub Pages, Docker)
2. Write quickstart.md:
   - Local dev setup (5 minutes)
   - Run sample book through pipeline
   - Test API endpoints with curl
   - Embed widget in local HTML
3. Write research.md:
   - Cohere model choices (embed-v4.0, rerank-v4.0-pro, command-a-03-2025)
   - Qdrant vs. alternatives (Pinecone, Milvus, Weaviate)
   - Chunking strategy rationale (semantic vs. fixed-size)
   - Trade-offs (accuracy vs. latency vs. cost)
4. Write data-model.md:
   - Entity diagrams (Book, Chunk, Query, Session)
   - Database schema (Qdrant payload, Neon tables)
   - Indexes & constraints
5. Write contract specs (OpenAPI YAML):
   - POST /ingest, POST /chat, POST /chat-stream, GET /health
   - Request/response schemas, error codes
6. Code review:
   - PEP 8 compliance
   - Docstrings on public methods
   - Error handling for edge cases
   - Logging coverage
7. Create demo:
   - Sample book HTML page
   - Embedded chat widget
   - Example queries + responses
   - Video walkthrough (optional)

**Success Criteria**:
- ✅ README complete + clear
- ✅ quickstart.md works end-to-end
- ✅ All code PEP 8 compliant
- ✅ Public methods documented
- ✅ API contracts in OpenAPI format
- ✅ Demo deployable + functional
- ✅ Deployment guide provided

---

## Key Technical Decisions & Rationale

### 1. Cohere Models (Exclusive)
- **embed-v4.0**: Latest, multimodal, supports input_type distinction (search_document vs. search_query)
- **rerank-v4.0-pro**: Best accuracy for reranking; alternative rerank-v4.0-fast if latency critical
- **command-a-03-2025**: Top-performing for RAG; use Chat endpoint with 'documents' parameter for auto-grounding + citations
- **Rationale**: Cohere-exclusive stack simplifies integration, reduces costs (single vendor), ensures grounded generation

### 2. Chunking Strategy (300-500 tokens, semantic boundaries, atomic-unit preservation - FR-001a)
- **Why not fixed-size?** Semantic boundaries preserve meaning; fixed-size splits concepts, code, tables
- **Why 300-500?** Balance between retrieval granularity (small = precise) and context (large = cover full idea). Cohere context limits: embed supports up to 2048 tokens; chat supports up to 4096
- **Atomic Unit Preservation (FR-001a)**: Code blocks, tables, mathematical equations, and structured content MUST NEVER be split mid-unit. If a chunk would split these units, expand boundary to include entire unit (may exceed 500 tokens). Post-indexing validation ensures no code blocks or tables are truncated.
- **Overlap (200 tokens)** Preserves context across chunk boundaries for reranking

### 3. Qdrant Cloud Free Tier
- **Limits**: ~10GB storage = ~50k chunks @ 1024-dim embeddings
- **Why not self-hosted?** Free tier eliminates ops burden; sufficient for MVP (500-page book = ~2k-5k chunks)
- **Alternative**: Pinecone (managed, cheaper at scale), Milvus (open-source, self-hosted)

### 4. Neon Postgres (Metadata Only)
- **Purpose**: Track chunk lineage (page, section, chapter) for source attribution
- **Why not Qdrant metadata alone?** Qdrant payload is optimized for vectors, not relational queries; Neon provides full SQL for re-indexing, analytics
- **Rationale**: Minimal cost (free tier sufficient), simple schema, allows future analytics

### 5. Select-Text Isolation & Validation (FR-007a)
- **Validation (FR-007a)**: Backend MUST enforce minimum 10 characters for selected_text. Reject (HTTP 400) if <10 characters with clear error message.
- **Implementation**: Accept selected_text parameter; filter Qdrant results by text_hash matching selected passage
- **Alternative**: Temporary collection per session (complex), API-level filtering (simple but less precise)
- **Rationale**: Zero-leakage guarantee critical for user trust; filtering ensures results come ONLY from selected passage. 10-character minimum prevents noise from trivial selections and ensures meaningful context for retrieval.

### 6. FastAPI + Vanilla JS (No Framework)
- **FastAPI**: Async, Pydantic validation, auto-docs (Swagger UI), lightweight
- **Vanilla JS**: No build step, no dependencies, easy to embed in iframe, works everywhere
- **Rationale**: Minimum deployment overhead; free-tier hosting compatible (Vercel, GitHub Pages)

## Complexity Tracking

No Constitution violations identified. All principles (Functionality, Integration, Documentation, API Usage, Accuracy, Security, Innovation, Stack) are satisfied by this plan. No additional justifications needed.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
