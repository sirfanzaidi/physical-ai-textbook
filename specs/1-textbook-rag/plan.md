# Implementation Plan: AI-Native Textbook with RAG Chatbot

**Branch**: `1-textbook-rag` | **Date**: 2025-12-06 | **Spec**: `/specs/1-textbook-rag/spec.md`
**Input**: Feature specification for 6-chapter Physical AI & Humanoid Robotics textbook with Docusaurus + RAG chatbot backend

**Note**: Phase 0 (Research) complete. Phase 1 (Design & Contracts) in progress. See `.specify/templates/commands/plan.md` for execution workflow.

## Summary

Build a professional, short, clean AI-native textbook (6 chapters, ~50-60 pages) for Physical AI & Humanoid Robotics, deployed as a static Docusaurus site on GitHub Pages with an integrated RAG chatbot backend. The system enables readers to select text from chapters and ask the Qdrant-backed chatbot questions answered strictly from book content (no hallucinations). Key features: semantic chunking (200-500 tokens), concurrent content authoring with CI-driven re-indexing, RAG accuracy validation (≥95% on 20+ test queries per module), and optional Urdu localization. Technical stack: Docusaurus 3.x (frontend), FastAPI (RAG backend), ChromaDB or Qdrant (vector DB), sentence-transformers/all-MiniLM-L6-v2 (embeddings), GitHub Pages (static hosting), Vercel (optional for RAG API). All components on free tier. Implementation phases: (1) Docusaurus skeleton + chapter authoring; (2) RAG backend + vector indexing; (3) bonuses (auth, personalization); (4) testing & deployment.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Docusaurus frontend), Python 3.11 (FastAPI RAG backend)
**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React, MDX, Docusaurus i18n
- Backend: FastAPI, sentence-transformers (all-MiniLM-L6-v2), ChromaDB or Qdrant client
- Infrastructure: GitHub Pages (static), Vercel or Railway (RAG API), GitHub Actions (CI/CD)

**Storage**:
- Vector DB: ChromaDB (embedded in FastAPI) or Qdrant Cloud free tier (1GB, up to 1M vectors)
- Code/Content: Git + GitHub (Markdown chapters in `docs/`)
- Metadata: Optional Neon PostgreSQL free tier (if user sessions/logs needed)
- Embeddings: sentence-transformers/all-MiniLM-L6-v2 (384 dimensions, <100KB total for 6 chapters)

**Testing**:
- Link validation: Docusaurus link checker (CI)
- Code example validation: Python/bash/YAML syntax checks (CI)
- RAG accuracy: Manual validation suite (≥3 queries per chapter, 18+ total per module)
- Performance: P95 latency benchmarks (<2 sec for chatbot queries)

**Target Platform**: Web (browser-based textbook), API backend (serverless-friendly)
**Project Type**: Web application + Python API backend (Option 2: Monorepo with two deployments)
**Performance Goals**:
- Docusaurus build: <2 min locally, <4 min CI
- RAG query latency: <2 sec p95 end-to-end
- Embedding generation: <100ms per query
- Vector search: <50ms on <1000 vectors
- Page load: <2 sec (static HTML advantage)

**Constraints**:
- All components must use free or non-proprietary tiers (Constitution V)
- RAG chatbot accuracy ≥95% on validation set (SC-002, Constitution III)
- No hallucinations or external knowledge synthesis
- Code examples run locally in <5 seconds
- Total storage: ~50-60 pages, 24,000-36,000 words
- CI must block merge on broken links, failed code validation, or missing learning objectives (FR-007)

**Scale/Scope**:
- Estimated content: 6 chapters, ~10 pages each, ~500 words/page = 30,000 words
- Estimated chunks: 60-120 total (200-500 tokens per chunk)
- Estimated vectors: 60-120 (384 dimensions each = ~90 KB total)
- User base: Students (read-only access, no authentication required for MVP)
- Deployment capacity: GitHub Pages (unlimited static hosting) + Vercel/Railway free tier (RAG API)

## Constitution Check

*GATE: Must pass before Phase 1 design. Re-check after Phase 1 completion.*

### Core Principles Alignment

| Principle | Requirement | Plan Status | Notes |
|-----------|-----------|-------------|-------|
| **I. Content-First Design** | Accuracy over breadth; precise & verifiable | ✅ PASS | Spec mandates RAG accuracy ≥95% (SC-002); all content indexed semantically; no synthesis outside book scope |
| **II. Progressive Disclosure** | Linear knowledge path; chapters build systematically | ✅ PASS | 6-chapter structure defined (Intro → Humanoid → ROS 2 → Simulation → Vision-Language-Action → Capstone); constitution specifies learning objectives + exercises |
| **III. Searchability via RAG** | Accuracy-first; no hallucinations; semantic chunks 200-500 tokens | ✅ PASS | ChromaDB/Qdrant selected for semantic retrieval; validation suite (FR-011) mandates ≥3 queries/chapter; stale chunks prevented via blue-green collection swap |
| **IV. Platform Standardization** | Docusaurus + MDX; auto-sidebar; relative paths; GitHub Pages | ✅ PASS | Docusaurus 3.x specified; MDX for content; auto-sidebar generation required (FR-002); GitHub Pages static hosting confirmed |
| **V. Minimal, Free-Tier Stack** | All dependencies free; no proprietary services | ✅ PASS | ChromaDB (free, embedded) or Qdrant Cloud free tier (1GB); sentence-transformers all-MiniLM-L6-v2 (free, open-source); FastAPI (free); GitHub Pages (free); no OpenAI/Pinecone/etc required |
| **VI. Accessibility & i18n** | i18n optional; Docusaurus i18n workflow; complete ≥1 chapter before release | ✅ PASS | Urdu translation is P3 (optional); Docusaurus i18n workflow specified (FR-009); does NOT block MVP launch |
| **VII. Test Coverage** | Automated link checks, code syntax validation, learning objective alignment, RAG chunk quality | ✅ PASS | CI checks mandated (FR-007): broken links, code execution, learning objectives; admin dashboard for RAG validation (FR-011) |
| **VIII. Simplicity & Minimalism** | Max 6 chapters, 50-60 pages; fast builds; no heavy GPU; clean UI; YAGNI | ✅ PASS | 6-chapter hard limit; 50-60 page constraint; lightweight embedding model; Docusaurus defaults UI; no over-engineering |

### Gate Result: **PASS** ✅

All core principles align with technical plan. No violations or exceptions required.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Option 2: Web application (Docusaurus frontend + FastAPI backend)

# Docusaurus textbook (static site generator frontend)
website/
├── docusaurus.config.js          # Docusaurus config, navbar, footer, i18n setup
├── sidebars.js                   # Auto-generated or custom sidebar structure
├── package.json
├── docs/                         # All chapter content (MDX)
│   ├── 01-introduction/
│   │   ├── index.mdx
│   │   ├── _category_.json       # Sidebar metadata
│   │   └── assets/               # Images, diagrams
│   ├── 02-humanoid-basics/
│   ├── 03-ros2-fundamentals/
│   ├── 04-digital-twin-simulation/
│   ├── 05-vision-language-action/
│   └── 06-capstone/
├── src/
│   ├── components/
│   │   ├── ChatbotPanel.tsx      # Select-text + chatbot UI component
│   │   ├── PersonalizeButton.tsx # Personalization hook (P3 future)
│   │   ├── UrduToggle.tsx        # Language switcher (P3 future)
│   │   └── [other Docusaurus-standard components]
│   ├── pages/
│   └── styles/
├── static/                       # Static assets (favicons, etc.)
└── build/                        # Generated output (local builds only)

# FastAPI RAG backend
backend/
├── main.py                       # FastAPI app entry point
├── requirements.txt              # Python dependencies (FastAPI, sentence-transformers, etc.)
├── .env.example                  # Environment variables template
├── config.py                     # Configuration (API keys, model paths, etc.)
├── src/
│   ├── models/
│   │   ├── chunk.py              # DataClass: Chapter chunk with metadata
│   │   ├── query.py              # DataClass: Chatbot query request/response
│   │   └── validation.py         # DataClass: RAG validation test results
│   ├── services/
│   │   ├── embeddings.py         # Embedding generation (sentence-transformers)
│   │   ├── vector_db.py          # ChromaDB/Qdrant client wrapper
│   │   ├── chunking.py           # Semantic chunking (200-500 token split)
│   │   ├── retrieval.py          # RAG retrieval logic (semantic search)
│   │   └── validation.py         # RAG accuracy validation suite (FR-011)
│   └── api/
│       ├── endpoints.py          # FastAPI route handlers
│       │   - POST /chat          # Chatbot query endpoint
│       │   - POST /reindex       # Admin re-indexing trigger
│       │   - POST /validate      # Admin validation suite
│       │   - GET /health         # Health check
│       └── middleware.py         # Rate limiting, logging, error handling
├── data/
│   └── embeddings/               # ChromaDB persistent storage (if embedded)
├── scripts/
│   ├── ingest_chapters.py        # Batch ingest chapters from website/docs/
│   ├── reindex.py                # Delta/full re-indexing orchestration
│   ├── validate_rag.py           # Run validation suite (18+ test queries)
│   └── generate_test_queries.py  # Generate per-chapter test queries
└── tests/
    ├── unit/
    │   ├── test_chunking.py
    │   ├── test_embeddings.py
    │   ├── test_retrieval.py
    │   └── test_api.py
    ├── integration/
    │   ├── test_end_to_end.py
    │   └── test_validation_suite.py
    └── fixtures/
        └── sample_chapters.json  # Test data

# CI/CD Configuration
.github/
├── workflows/
│   ├── docusaurus-build.yml      # Build + deploy to GitHub Pages
│   ├── rag-indexing.yml          # Delta re-index + validation
│   ├── link-checker.yml          # External link validation (FR-008)
│   ├── code-validation.yml       # Syntax checks for examples (FR-007)
│   └── learning-objectives.yml   # Verify learning objectives (FR-010)
└── pull_request_template.md      # PR template with constitution checklist

# Documentation
.specify/
├── memory/
│   └── constitution.md           # (already exists)
├── templates/
│   └── phr-template.prompt.md    # PHR template
└── scripts/
    ├── powershell/
    │   ├── setup-plan.ps1        # (already exists)
    │   └── update-agent-context.ps1 # (already exists)
    └── bash/
        └── create-phr.sh         # (already exists)

specs/1-textbook-rag/
├── spec.md                       # (already exists)
├── plan.md                       # This file
├── research.md                   # Phase 0 output (generated next)
├── data-model.md                 # Phase 1 output (to generate)
├── quickstart.md                 # Phase 1 output (to generate)
├── contracts/
│   ├── api-schema.openapi.yaml   # Phase 1 output
│   └── data-contracts.md         # Phase 1 output
└── tasks.md                      # Phase 2 output (generated by /sp.tasks)

# Root project files
.gitignore
README.md
```

**Structure Decision**: Monorepo with separate frontend (Docusaurus) and backend (FastAPI) deployments:
- **Frontend** (`website/`): Docusaurus 3.x with auto-sidebar, MDX chapters, ChatbotPanel component
- **Backend** (`backend/`): FastAPI Python app with ChromaDB or Qdrant integration, chunking/embedding/retrieval services, validation suite
- **CI/CD** (`.github/workflows/`): Four separate workflows (Docusaurus build, RAG indexing, link checking, code validation) with concurrency control for re-indexing
- **Rationale**: Separation of concerns (static content vs dynamic RAG backend); independent deployment cadences; allows scaling backend independently; Vercel/Railway can host RAG API while GitHub Pages hosts static textbook

## Complexity Tracking

> **Constitution Check PASSED. No violations requiring justification.**

However, two architectural decisions bear explicit documentation (for potential ADRs):

| Decision | Rationale | Alternatives Rejected |
|----------|-----------|----------------------|
| **Vector DB: ChromaDB (embedded) vs Qdrant (managed)** | ChromaDB ideal for 60-120 vectors (textbook scale); Qdrant over-engineered but available if ops capacity exists; research shows both free-tier viable | FAISS (requires persistence layer DIY); in-memory only (no durability) |
| **Embedding Model: sentence-transformers/all-MiniLM-L6-v2** | Lightweight (384 dims), fast (<100ms), truly free (no API calls/rate limits), meets <2s p95 target easily; aligns with Constitution V (minimal stack) | OpenAI text-embedding-3-small (free tier too restrictive for batch indexing); Ollama (slower, larger model) |
| **Concurrent Indexing: GitHub Actions concurrency + blue-green collection swap** | Prevents stale chunks; zero downtime; simple fail-safe; validation before swap | Blocking chatbot during re-index (poor UX); incremental indexing only (risky for embedding model changes) |
| **Monorepo (website/ + backend/)** | Simplified CI/CD; single repository; independent deployments; front-end developers don't need Python env setup; matches GitHub Pages + Vercel workflow | Separate repos (coordination overhead); single repo with nested builds (slower) |

---

## Key Architectural Decisions (For ADR)

### Decision 1: Vector Database Selection (ChromaDB for MVP)

**Status**: Proposed (awaiting user consent for ADR documentation)

**Summary**: Use ChromaDB (embedded in FastAPI) for Phase 1-2; optionally upgrade to Qdrant Cloud free tier if operational capacity available.

**Context**:
- Textbook scale: 60-120 vectors (384 dimensions) = ~90 KB total storage
- Latency target: <2 seconds p95 for RAG query
- Cost constraint: Free tier only

**Decision**: ChromaDB embedded in FastAPI backend
- ✅ Sub-20ms query latency on 60-120 vectors
- ✅ Zero external service dependencies
- ✅ Built-in persistence via DuckDB
- ✅ Simplest deployment (no separate service)
- ⚠️ Migration path available if >200K vectors needed

**Alternatives Considered**:
- Qdrant Cloud (1GB free tier): Managed, production-grade, but +20-50ms network latency (still acceptable)
- In-memory only: Fast but loses durability; unacceptable for production textbook
- FAISS: Fastest but requires custom persistence + metadata layer

---

### Decision 2: Embedding Model Selection (sentence-transformers/all-MiniLM-L6-v2)

**Status**: Proposed (awaiting user consent for ADR documentation)

**Summary**: Use sentence-transformers/all-MiniLM-L6-v2 for embeddings; fallback to all-mpnet-base-v2 if accuracy <95%.

**Context**:
- Embedding generation must be fast (<100ms per query) + free
- 384-dim vectors sufficient for textbook-scale retrieval
- No external APIs allowed (Constitution V)

**Decision**: sentence-transformers/all-MiniLM-L6-v2
- ✅ 30-80ms embedding latency (leaves budget for search + LLM)
- ✅ Completely free + open-source
- ✅ 384 dimensions (minimal storage: ~90KB total)
- ✅ Robust on domain-specific text (robotics, AI)
- ✅ Runs on CPU (no GPU required)

**Alternatives Considered**:
- OpenAI text-embedding-3-small: Free tier too restrictive (3 req/min) for batch indexing
- Ollama nomic-embed-text: Slower (100ms+) and larger (~400MB download)
- ONNX models: Unnecessary complexity for Python backend

---

### Decision 3: Concurrent Indexing & Blue-Green Swap

**Status**: Proposed (awaiting user consent for ADR documentation)

**Summary**: Use GitHub Actions concurrency groups + Qdrant/ChromaDB collection aliases for atomic, zero-downtime re-indexing.

**Context**:
- FR-006 requires blocking chatbot during re-indexing (ambiguous: hard block vs soft fail?)
- Concurrent chapter PRs merge frequently (risk of stale chunks or dropped queries)
- Target: <5 minute re-indexing, no downtime for readers

**Decision**: Blue-green collection swap
1. Create new collection (green)
2. Index all chapters into green (validation happens here)
3. Atomically swap alias: prod → green
4. Delete old collection

- ✅ Zero downtime for chatbot
- ✅ No stale chunks (validation before swap)
- ✅ Instant rollback if validation fails
- ✅ Atomic alias switch (no partial states)

**Alternatives Considered**:
- Hard-block chatbot with "maintenance in progress": Poor UX; 5 min downtime
- Incremental indexing only: Risky for embedding model changes; doesn't fully re-validate
- Tombstone soft-delete: Adds storage complexity; Qdrant already handles this internally

---

## Research Summary (Phase 0 Outputs)

*Consolidated findings from research agents:*

### Vector Database Research
- ChromaDB: 60-120 vector sweet spot; <20ms latency; embedded no external dependencies
- Qdrant: Production-grade; same free-tier cost; +20-50ms network latency if cloud
- FAISS: Fastest but requires custom persistence layer (too much integration work)
- **Recommendation**: ChromaDB (embedded) with clear upgrade path to Qdrant Cloud

### Concurrent Authoring & Re-indexing
- GitHub Actions concurrency (queue-based, not cancel-based) prevents duplicate writes
- Delta indexing (only modified chapters) achieves 7-13 sec re-index; full re-index 30-60 sec
- Blue-green collection swap prevents stale chunks; atomic alias switch is key
- Batch embedding: 5 requests/sec (100 chunks/request) = 1 second for full textbook
- **Recommendation**: Delta daily + full weekly; validate before swap

### Embedding Models
- OpenAI free tier: 3 req/min (unsuitable for batch indexing)
- sentence-transformers/all-MiniLM-L6-v2: 30-80ms, free, 384 dims, battle-tested
- Ollama: Slower (100ms+), larger model downloads, overkill for 60-120 vectors
- **Recommendation**: all-MiniLM-L6-v2 with fallback to all-mpnet-base-v2 if validation <95%

---
