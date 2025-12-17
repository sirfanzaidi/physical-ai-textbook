---
# Prompt History Record (PHR)
ID: 002
TITLE: Create Implementation Plan for RAG Chatbot
STAGE: plan
DATE_ISO: 2025-12-17
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: 002-rag-docusaurus-openrouter
BRANCH: 001-rag-chatbot
USER: user
COMMAND: /sp.plan
LABELS: ["rag-chatbot", "docusaurus", "openrouter", "architecture", "implementation-plan"]
LINKS:
  SPEC: specs/002-rag-docusaurus-openrouter/spec.md
  PLAN: specs/002-rag-docusaurus-openrouter/plan.md
  TICKET: null
  ADR: null
  PR: null
FILES_YAML: |
  - specs/002-rag-docusaurus-openrouter/plan.md
TESTS_YAML: |
  - (none - planning phase)
---

## PROMPT_TEXT

```
/sp.plan

Technology Stack:
- Frontend: Docusaurus v3+ (custom React component for chatbot widget, using highlight.js or window.getSelection for selected text)
- Backend: FastAPI (uvicorn), LangChain or LlamaIndex for RAG pipeline
- Embeddings & LLM: OpenRouter API (base_url="https://openrouter.ai/api/v1", model="qwen/qwen3-embedding-4b" for embeddings, strong model like qwen/qwen3-8b for chat)
- Vector DB: Qdrant Cloud (free tier cluster)
- Optional DB: Neon Postgres for storing chunk metadata/sources
- Deployment: Docusaurus on GitHub Pages; FastAPI on free host (Render, Fly.io, or Vercel)

High-level Phases:

Phase 1: Backend Setup
- Create FastAPI app with /ingest and /chat endpoints
- Integrate OpenRouter for embeddings and completions
- Connect to Qdrant Cloud
- Ingestion: Load Markdown files or crawl built site HTML, chunk, embed, store

Phase 2: Frontend Widget
- Custom React component: Floating button → opens chat modal
- Capture page selected text and send with query
- Stream responses from FastAPI

Phase 3: Integration into Docusaurus
- Swizzle Footer or create theme component to inject widget globally
- Config for backend URL

Phase 4: Testing & Polish
- Manual ingestion after book updates
- Handle citations, sources linking back to [textbook pages]
```

## RESPONSE_TEXT

Created comprehensive 609-line implementation plan with:

**Technical Context:**
- Python 3.11 (FastAPI), TypeScript/React (Docusaurus widget)
- Dependencies: FastAPI, LangChain/LlamaIndex, OpenRouter SDK, qdrant-client, axios
- Performance: <5s p95 latency, <30min ingestion, ≥10 concurrent users
- Scale: ~100K vectors for single book (Physical AI textbook)

**Architectural Decisions (4 ADRs):**
1. OpenRouter as unified AI provider (embeddings + chat) - rationale + alternatives
2. Qdrant Cloud for vector DB - cost-effective, managed service
3. LangChain/LlamaIndex for RAG orchestration - avoids reinventing wheels
4. GitHub Pages + separate FastAPI backend - independent scalability

**Project Structure:**
- Backend (FastAPI): app/, tests/, services/ (ingestion, embedding, retrieval, generation)
- Frontend (Docusaurus): components/RAGChat/, services/, config/

**Data Model:**
- Chunk: id, chapter_id, content, embedding (1024-dim), metadata
- Query: query_text, mode, selected_text, response, citations, latency_ms
- Citation: chunk_id, chapter_name, source_url
- Session: (optional) Neon Postgres for chat history

**API Contracts:**
- POST /api/ingest: Upload Docusaurus, chunk, embed, upsert
- POST /api/chat-stream: JSON-lines streaming response with citations
- GET /api/health: Service health check

**4 Implementation Phases (6 weeks):**
1. Phase 0: Research (1 week) - OpenRouter pricing, Qdrant limits, chunking strategy
2. Phase 1: Backend (2 weeks) - FastAPI, ingestion, RAG pipeline
3. Phase 2: Frontend (1 week) - React widget, text selection, API integration
4. Phase 3: Integration (1 week) - Docusaurus swizzle, env config, CI/CD
5. Phase 4: Testing (1 week) - Blind testing, perf validation, security review

**Risk Mitigation:** 5 identified risks with likelihood/impact/mitigation strategies

**Success Metrics:** 8 measurable outcomes mapped to spec criteria (90% accuracy, <5s latency, zero-leakage select-text, etc.)

---

## OUTCOME

✅ Comprehensive implementation plan created
✅ Architecture decisions documented with rationale
✅ Project structure defined (backend + frontend)
✅ API contracts specified in detail
✅ 4-phase delivery schedule with exit criteria
✅ Risk mitigation strategies documented
✅ All success metrics traceable to spec

Plan is actionable and ready for Phase 0 execution.

---

## EVALUATION

**Quality Assessment:**
- Plan balances ambition with realism (6-week timeline for POC)
- Technology choices well-justified (OpenRouter, Qdrant, LangChain)
- Risk identification comprehensive (5 major risks with mitigations)
- Parallel work opportunities identified (frontend + backend in Phase 2-3)
- Exit criteria clear for each phase (testable, measurable)

**Readiness for Next Phase:**
- Ready for `/sp.tasks` to break phases into actionable tasks
- Phase 0 (Research) can begin immediately
- Clear dependencies between phases identified
- Risks understood and mitigation strategies in place

**Key Strengths:**
1. Architecture decisions well-documented with alternatives and tradeoffs
2. Data model designed with Qdrant schema specifics
3. API contracts complete (request/response examples)
4. Phased approach allows iterative value delivery
5. Security and performance considerations included
6. Integration with existing Docusaurus site (not greenfield)
