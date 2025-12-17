---
# Prompt History Record (PHR)
ID: 001
TITLE: Create RAG Docusaurus OpenRouter Specification
STAGE: spec
DATE_ISO: 2025-12-17
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: 002-rag-docusaurus-openrouter
BRANCH: 001-rag-chatbot (preparation for 002 branch)
USER: user
COMMAND: /sp.specify
LABELS: ["rag-chatbot", "docusaurus", "openrouter", "specification", "feature-definition"]
LINKS:
  SPEC: specs/002-rag-docusaurus-openrouter/spec.md
  TICKET: null
  ADR: null
  PR: null
FILES_YAML: |
  - specs/002-rag-docusaurus-openrouter/spec.md
  - specs/002-rag-docusaurus-openrouter/checklists/requirements.md
TESTS_YAML: |
  - (none - specification phase)
---

## PROMPT_TEXT

```
/sp.specify

Project Extension: Integrate a Retrieval-Augmented Generation (RAG) chatbot into the existing Docusaurus book "Physical AI & Humanoid Robotics".

Core Features:
- Floating chatbot widget embedded site-wide in the Docusaurus book (GitHub Pages deployed).
- Answers questions based on the book's content only (no external knowledge).
- Supports answering based on user-selected text (highlight text → ask about it).
- Uses OpenRouter API for both embeddings and chat completions (OpenAI-compatible endpoint).
- Embeddings model: Qwen series via OpenRouter (prefer qwen/qwen3-embedding-4b or qwen/qwen3-embedding-0.6b for multilingual and efficiency).
- Vector database: Qdrant Cloud Free Tier.
- Metadata storage (optional chunks/sources): Neon Serverless Postgres (free tier).
- Backend: FastAPI server for ingestion pipeline and query endpoint.
- Frontend: Custom React widget (floating button + chat interface) using MDX/components in Docusaurus, communicating with FastAPI backend.

Requirements:
- Ingestion script: Crawl/extract all Markdown content from built Docusaurus site, chunk text, generate embeddings via OpenRouter, upsert to Qdrant.
- RAG Query Flow: Retrieve relevant chunks from Qdrant → Augment prompt → Generate response via OpenRouter (e.g., a strong Qwen3 or other model).
- Support for selected text: Widget captures highlighted text on page and includes it in query context.
- Secure: API keys stored server-side (env vars), no client exposure.
- Deployable: FastAPI on free platform (e.g., Render/Deta/Vercel), separate from GitHub Pages.
- Educational: Chatbot introduces itself as book assistant, cites sources with links to docs pages.

Deliverable: Updated Docusaurus repo with embedded chatbot + separate backend repo for FastAPI RAG service.
```

## RESPONSE_TEXT

Created comprehensive feature specification for RAG Chatbot with Docusaurus & OpenRouter integration:

**Specification Summary:**
- **5 User Stories** (P1/P2 prioritized): Core Q&A, select-text, ingestion, streaming, history
- **18 Functional Requirements**: Clear, testable, technology-agnostic
- **10 Success Criteria**: Measurable outcomes (90% accuracy, <5s latency, etc.)
- **6 Edge Cases**: Slow ingestion, API unavailability, large content, sync issues, cross-browser selection, mobile support
- **8 Assumptions**: Documented defaults for Docusaurus structure, OpenRouter availability, Qdrant tier, embedding dimensions
- **Out of Scope**: Listed deferred features (multi-language, analytics, personalization, scaling)

**Key Deliverables:**
- `specs/002-rag-docusaurus-openrouter/spec.md` - Full specification (300+ lines)
- `specs/002-rag-docusaurus-openrouter/checklists/requirements.md` - Quality validation checklist (✅ all items pass)

**Feature Readiness:** ✅ Complete and ready for planning phase
- All mandatory sections completed
- No clarifications needed (reasonable defaults used)
- User scenarios independently testable
- Success criteria measurable and technology-agnostic

**Next Steps:** Run `/sp.plan` to create architecture and implementation roadmap

---

## OUTCOME

✅ Feature specification created successfully
✅ Quality checklist passed all validation items
✅ Spec is complete, unambiguous, and ready for planning
✅ PHR recorded for traceability

---

## EVALUATION

**Quality Assessment:**
- Specification is comprehensive and well-structured
- User scenarios clearly prioritized with business rationale
- Requirements are testable and avoid implementation specifics
- Success criteria are measurable and include both quantitative and qualitative metrics
- Edge cases identified and mitigated
- Assumptions documented to reduce future confusion

**Readiness for Next Phase:**
- Ready for `/sp.plan` to create detailed architecture
- Can proceed directly to implementation planning
- No clarifications or revisions needed
