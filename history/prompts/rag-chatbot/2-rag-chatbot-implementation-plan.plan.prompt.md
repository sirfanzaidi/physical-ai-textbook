---
id: 2
title: RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-14
surface: agent
model: claude-haiku-4-5-20251001
feature: rag-chatbot
branch: 001-rag-chatbot
user: user
command: /sp.plan
labels: ["rag-chatbot", "plan", "cohere-api", "fastapi", "qdrant", "implementation"]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/plan.md
tests:
  - none
---

## Prompt

```
/sp.plan Integrated RAG Chatbot for Published Book (Cohere-Powered, Accurate & Embeddable)

Project Overview
Develop a high-quality, embeddable Retrieval-Augmented Generation (RAG) chatbot for a digital published book. The chatbot answers questions based on full book content or only user-selected text, using Cohere's latest models for embeddings, reranking, and generation. Backend powered by FastAPI, vector storage in Qdrant Cloud Free Tier, metadata in Neon Serverless Postgres. Prioritize accurate, grounded responses with citations, minimal hallucinations, fast response times (<5s), and secure credential handling.

Key Technologies & Models (as of December 14, 2025)
- Cohere SDK for:
  - Embeddings: embed-v4.0 (multimodal, high-performance; input_type="search_document" for storage, "search_query" for queries; supports matryoshka dimensions)
  - Rerank: rerank-v4.0-pro (preferred for accuracy) or rerank-v4.0-fast (for speed); multilingual, 32k context
  - Generation: command-a-03-2025 (top-performing for RAG, agents, multilingual; use Chat endpoint with 'documents' parameter for automatic grounding + citations)
- Qdrant for vector search
- FastAPI for backend APIs
- Vanilla JS widget for frontend embedding

Milestones & Detailed Phases (2-4 Week Timeline)
[7 detailed phases with specific deliverables, activities, and success criteria]
```

## Response snapshot

Created comprehensive Implementation Plan (7 phases, 2-4 weeks) for RAG Chatbot. All principles from Constitution v2.0.0 validated. Structure: FastAPI backend (ingestion, retrieval, generation, database modules) + Vanilla JS widget. Detailed phases:

**Phase 1 (Days 1-3)**: Environment Setup
- Initialize repo, install dependencies (Python 3.11+, FastAPI, Cohere SDK, qdrant-client, psycopg2)
- Configure .env (never commit), create Qdrant collection (book_vectors, dim 1024), Neon metadata table
- Smoke tests verify Cohere/Qdrant/Neon connections

**Phase 2 (Days 4-8)**: Book Ingestion Pipeline
- PDF/text parsing, semantic chunking (300-500 tokens, 200 overlap)
- Cohere embed-v4.0 (input_type="search_document") batch embedding
- Qdrant upsert (vectors + metadata), Neon metadata logging
- Support re-indexing (delete old, upsert new, idempotent)
- Test with 2-3 books (50-500 pages)

**Phase 3 (Days 9-13)**: Advanced Retrieval System
- Query embedding: embed-v4.0 (input_type="search_query")
- Initial search: top_k=20-30 from Qdrant
- Reranking: rerank-v4.0-pro, final top_k=8-10
- Select-text mode: Constrained retrieval (zero-leakage guarantee)
- Document assembly for Cohere Chat

**Phase 4 (Days 14-19)**: FastAPI Backend & RAG Chat
- Endpoints: POST /ingest, POST /chat, POST /chat-stream, GET /health
- Cohere Chat API with 'documents' parameter (auto-grounding + citations)
- System prompt: "Answer only from documents. If unsure, say 'I don't know.'"
- Input validation, error handling, structured logging

**Phase 5 (Days 20-23)**: Embeddable Frontend Widget
- Vanilla JS chat widget (no framework, lightweight)
- Message history, select-text → "Ask about this" button
- Iframe-embeddable, responsive CSS, no hardcoded API keys
- demo.html example

**Phase 6 (Days 24-27)**: Testing, Optimization & Security
- Unit tests: >80% coverage (chunking, embedding, retrieval, generation)
- Integration tests: End-to-end flows, select-text zero-leakage
- Blind eval: 50+ queries, ≥90% accuracy target
- Security audit: Input validation, prompt injection, credential protection
- Performance audit: p95 latency <5s, Cohere/Qdrant/Neon usage monitoring

**Phase 7 (Days 28-30)**: Documentation & Final Polish
- README (setup, ingestion, deployment), quickstart (5-min setup)
- research.md (Cohere model choices, rationale, alternatives)
- data-model.md (entities, schemas, indexes)
- API contracts (OpenAPI specs for /ingest, /chat, /chat-stream)
- Code review (PEP 8, docstrings, error handling)
- Demo + deployment guide

**Technical Context**:
- Language: Python 3.11+
- Backend: FastAPI 0.109+, Cohere SDK, qdrant-client, psycopg2
- Storage: Qdrant Cloud Free (1024-dim, book_vectors collection), Neon PostgreSQL (chunks_metadata table)
- Testing: pytest (unit/integration), blind eval (50+ queries)
- Performance: p95 latency <5s, 90%+ blind test accuracy, 500-page max book, ~50k chunk limit

**Constitution Check**: ✅ **PASS**
- All 8 principles aligned (Functionality, Integration, Documentation, API Usage, Accuracy, Security, Innovation, Stack)
- No violations. Gate status: Ready for Phase 0 Research.

**Key Technical Decisions**:
1. **Cohere Exclusive**: embed-v4.0 + rerank-v4.0-pro + command-a-03-2025 for accuracy + cost predictability
2. **Semantic Chunking (300-500 tokens)**: Preserves meaning vs. fixed-size; balance retrieval granularity + context
3. **Qdrant Cloud Free Tier**: Managed service, no ops overhead, sufficient for MVP (~2k-5k chunks per 500-page book)
4. **Neon Postgres Metadata**: Track lineage (page, section, chapter) for source attribution
5. **Select-Text Isolation**: Filter by text_hash (zero-leakage guarantee)
6. **FastAPI + Vanilla JS**: Minimal deployment overhead, free-tier hosting compatible

## Outcome

- ✅ Impact: Establishes detailed, phase-based implementation roadmap (7 phases, 2-4 weeks). All technical decisions justified. Select-text zero-leakage design explicit. Cohere-exclusive stack locked. 90% accuracy gate clear. Ready for `/sp.tasks` to generate individual implementation tasks.
- 🧪 Tests: Phase 6 defines comprehensive test strategy (unit >80% coverage, integration, blind eval 50+ queries, security, performance)
- 📁 Files: specs/001-rag-chatbot/plan.md (complete with 7 phases, technical decisions, complexity tracking)
- 🔁 Next prompts: `/sp.tasks` to generate task list with dependencies; `/sp.research` if any unknowns need resolution; implementation begins with Phase 1
- 🧠 Reflection: Plan is detailed, actionable, and grounded in Constitution principles. Each phase has clear deliverables and success criteria. Select-text zero-leakage is architecturally ensured. Blind test accuracy (90%+) gates production release. Free-tier stack limits concurrency but suitable for MVP.

## Evaluation notes (flywheel)

- Failure modes observed: None (plan is comprehensive and aligned with spec)
- Graders run and results (PASS/FAIL): PASS (all Constitution principles evaluated ✅; technical context complete; 7 phases detailed; success criteria measurable; no unresolved NEEDS CLARIFICATION)
- Prompt variant (if applicable): null
- Next experiment: Run `/sp.tasks` to break plan into individual, parallelizable tasks with clear dependencies and file paths
