# RAG Chatbot for Published Books — Specification & Planning

**Feature**: 001-rag-chatbot
**Branch**: `001-rag-chatbot`
**Status**: Specification Complete (Ready for Architecture Planning)
**Created**: 2025-12-14

## Overview

This directory contains the specification, planning artifacts, and supporting documentation for the **Integrated RAG Chatbot Development** project. The chatbot enables seamless integration with digital books, allowing readers to:

1. **Ask questions** about book content and receive grounded answers
2. **Select text** and ask questions about specific passages (zero leakage from rest of book)
3. **Admins**: Upload books and manage indexed content
4. **Validate accuracy** via blind testing before production launch

## Key Constraints

- **API**: Cohere API (exclusive) for embeddings and generation
- **Vector DB**: Qdrant Cloud Free Tier
- **Metadata DB**: Neon Serverless PostgreSQL (free tier)
- **Backend**: FastAPI (Python)
- **Book Scope**: Up to 500 pages per book
- **Accuracy Target**: ≥90% on blind test queries
- **Latency Target**: p95 < 5 seconds
- **Timeline**: 2-4 week sprint

## Files in This Directory

### Specification & Planning

- **`spec.md`** — Complete feature specification
  - 4 prioritized user stories (P1 MVP + P2 features)
  - 15 functional requirements
  - 8 success criteria
  - 5 edge cases
  - Key entities and data model
  - Assumptions and out-of-scope items

- **`checklists/requirements.md`** — Specification quality validation
  - Content quality checks
  - Requirement completeness
  - Feature readiness verification
  - Sign-off for architecture phase

### Configuration

- **`.env.example`** — Environment variable template
  - Cohere API key
  - Qdrant configuration (URL, API key)
  - Neon database URL
  - FastAPI settings
  - Chunking parameters
  - Security settings

### Supporting Documents (To Be Created)

- `plan.md` — Architecture and design (created by `/sp.plan`)
- `research.md` — Technology research and trade-offs
- `data-model.md` — Entity relationships and schemas
- `contracts/` — API endpoint contracts
- `tasks.md` — Implementation task list (created by `/sp.tasks`)

## Specification Summary

### User Stories

| # | Title | Priority | Type | Independent Test |
|---|-------|----------|------|-----------------|
| US1 | Query Book Content with RAG | P1 🎯 MVP | Core feature | Index book → Query → Verify grounded answers |
| US2 | Select Text and Ask Questions | P1 🎯 MVP | Differentiator | Highlight → Chat → Zero-leakage validation |
| US3 | Admin: Index and Manage Books | P2 | Setup | Upload PDF → Index chunks → Verify retrieval |
| US4 | Track Accuracy and Quality | P2 | QA/Launch | 50+ blind tests → ≥90% accuracy gate |

### Functional Requirements Highlights

- **Core RAG**: Semantic chunking (300–500 tokens), Cohere embeddings, Qdrant storage, grounded generation
- **Select-Text**: Constrained retrieval from highlighted passages only (zero leakage)
- **Admin API**: `/index` endpoint for book upload and indexing
- **User API**: `/query` endpoint with source attribution
- **Security**: Credentials in `.env`, HTTPS enforcement, PII protection
- **Quality**: ≥90% blind test accuracy, <5 sec p95 latency

### Success Criteria

| # | Criterion | Target | Measurable |
|---|-----------|--------|-----------|
| SC-001 | Blind test accuracy | ≥90% on 50+ queries | ✅ Measurable |
| SC-002 | Select-text zero-leakage | 100% no out-of-selection info | ✅ Testable |
| SC-003 | Query latency | p95 < 5 seconds | ✅ Measurable |
| SC-004 | Book size support | Up to 500 pages | ✅ Testable |
| SC-005 | Embeddability | Zero setup errors | ✅ Testable |
| SC-006 | Security review | Pass (no hardcoded secrets) | ✅ Testable |
| SC-007 | User satisfaction | 80%+ helpful responses | ✅ Measurable |
| SC-008 | Code quality | PEP 8, error handling | ✅ Testable |

## Next Steps

### For Architecture Planning (Phase 2)

Run `/sp.plan` to:
1. Research Cohere API integration patterns
2. Design Qdrant vector storage schema
3. Design FastAPI endpoint structure
4. Design select-text constraint logic
5. Design accuracy validation pipeline
6. Create implementation plan with phase breakdown

### For Implementation (Phase 3)

After planning, run `/sp.tasks` to:
1. Break plan into testable tasks
2. Organize by user story (independent implementation)
3. Track dependencies and parallelization opportunities
4. Provide exact file paths for each task

## Development Environment Setup

### Prerequisites

1. Create a `.env` file by copying `.env.example`:
   ```bash
   cp .env.example .env
   ```

2. Fill in actual credentials:
   - Get Cohere API key from: https://dashboard.cohere.com
   - Create Qdrant Cloud Free Tier account: https://cloud.qdrant.io
   - Create Neon PostgreSQL account: https://neon.tech

3. Install Python 3.11+ and dependencies (to be defined in plan.md)

### Quick Start (After Implementation)

```bash
# 1. Activate environment
source venv/bin/activate  # Linux/Mac
# or
.\venv\Scripts\activate  # Windows

# 2. Index a sample book
python -m backend.indexing.main --book-path ./samples/book.pdf

# 3. Start API server
uvicorn backend.api.main:app --reload

# 4. Access Swagger UI
# Open: http://localhost:8000/docs

# 5. Test query endpoint
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{"query": "What is the main topic of chapter 1?", "book_id": "sample-book"}'

# 6. Test select-text query
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this passage",
    "book_id": "sample-book",
    "selected_text": "The specific passage from the book",
    "select_text_only": true
  }'
```

## Key Design Decisions

### Why Cohere (Exclusive)?

- **Embeddings**: High-quality Cohere Embed API for semantic search
- **Generation**: Cohere Command models for accurate, grounded RAG responses
- **Cost Predictability**: Single vendor, easy to monitor and optimize usage
- **No Hallucinations**: Cohere models trained for factual, grounded generation

### Why Qdrant Cloud Free Tier?

- **Managed Service**: No ops overhead; automatic backups and scaling
- **Free Tier Limits**: Sufficient for MVP (10GB vectors ≈ 1M chunks @ 100-dim embeddings)
- **Semantic Search**: Native vector search with exact/approximate distance metrics
- **Alternative**: Self-hosted Qdrant if free tier limit exceeded

### Why Select-Text Zero-Leakage?

- **Precision**: Users want answers specific to their highlighted passage
- **Verification**: Easy to validate by checking response doesn't reference other sections
- **Implementation**: Constrain retrieval to selected passage metadata before generation

### Why ≥90% Accuracy Target?

- **User Trust**: Hallucinations break credibility; responses must be grounded
- **Measurable**: Blind test with 50+ unseen queries
- **Iteration**: If below 90%, adjust chunking, retrieval, or prompt strategies

## Architecture Overview (High-Level)

```
┌─────────────────────────────────────┐
│ Web-Based Book Viewer (UI Layer)    │
│ ┌─────────────────────────────────┐ │
│ │ Reader: Book Display + Chat      │ │
│ │ Select Text → Open Chat → Query  │ │
│ │ Admin: Book Upload Interface     │ │
│ └─────────────────────────────────┘ │
└──────────────┬──────────────────────┘
               │
        HTTP/REST API
               │
┌──────────────▼──────────────────────┐
│ FastAPI Backend (API Layer)         │
│ ┌─────────────────────────────────┐ │
│ │ /query - User queries           │ │
│ │ /select-text-query - Select-text│ │
│ │ /index - Admin indexing         │ │
│ │ /health - Health check          │ │
│ └─────────────────────────────────┘ │
└──────────┬──────────────┬────────────┘
           │              │
    ┌──────▼───────────────▼─────┐
    │ RAG Pipeline (Logic Layer)  │
    │ ┌──────────────────────────┐│
    │ │ Retrieval (Qdrant search)││
    │ │ Augmentation             ││
    │ │ Generation (Cohere API)  ││
    │ └──────────────────────────┘│
    └──────────┬──────────┬────────┘
               │          │
    ┌──────────▼────────┐ │
    │ Qdrant Cloud Free │ │ Cohere API
    │ (Vector DB)       │ │ (Embeddings
    │ Chunks +          │ │  + Generation)
    │ Embeddings        │ │
    └───────────────────┘ │
                          │
                    ┌─────▼────────┐
                    │ Neon (optional)
                    │ Sessions/Meta │
                    └────────────────┘
```

## Files to Review Before Planning

1. **Constitution**: `.specify/memory/constitution.md` (v2.0.0 — RAG-centric principles)
2. **This Spec**: `spec.md` (user stories, requirements, success criteria)
3. **Checklist**: `checklists/requirements.md` (validation passed)

## Questions or Clarifications?

Before proceeding to `/sp.plan`, verify:

- [ ] 4 user stories are clear and achievable
- [ ] 15 functional requirements are complete
- [ ] 8 success criteria are measurable
- [ ] Select-text zero-leakage requirement is understood
- [ ] 90% accuracy target is achievable with Cohere
- [ ] 500-page book limit is acceptable for MVP
- [ ] Out-of-scope items won't block MVP launch

If any clarifications needed, use `/sp.clarify` to refine the spec before moving to planning.

---

**Specification Status**: ✅ **COMPLETE** — Ready for `/sp.plan` (architecture design)
