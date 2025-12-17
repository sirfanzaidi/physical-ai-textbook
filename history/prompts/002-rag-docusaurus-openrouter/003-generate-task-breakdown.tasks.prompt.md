---
# Prompt History Record (PHR)
ID: 003
TITLE: Generate Task Breakdown for RAG Chatbot Implementation
STAGE: tasks
DATE_ISO: 2025-12-17
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: 002-rag-docusaurus-openrouter
BRANCH: 001-rag-chatbot
USER: user
COMMAND: /sp.tasks
LABELS: ["rag-chatbot", "docusaurus", "openrouter", "task-breakdown", "implementation-tasks"]
LINKS:
  SPEC: specs/002-rag-docusaurus-openrouter/spec.md
  PLAN: specs/002-rag-docusaurus-openrouter/plan.md
  TASKS: specs/002-rag-docusaurus-openrouter/tasks.md
  TICKET: null
  ADR: null
  PR: null
FILES_YAML: |
  - specs/002-rag-docusaurus-openrouter/tasks.md
TESTS_YAML: |
  - (testing tasks included in Phase 8 - T079-T085)
---

## PROMPT_TEXT

```
/sp.tasks

Generate prioritized task list for adding RAG chatbot.

Separate into two repos:
1. Backend (fastapi-rag-backend)
2. Updated Docusaurus book repo

Tasks:

Backend:
- Initialize FastAPI project (main.py, requirements.txt with fastapi, uvicorn, openai, qdrant-client, langchain or llama-index)
- Setup OpenRouter client (openai.OpenAI(base_url="https://openrouter.ai/api/v1", api_key=OPENROUTER_API_KEY))
- Create Qdrant collection (dimension based on Qwen embedding size)
- Ingestion endpoint/script: Load docs content, chunk (recursive splitter), embed with Qwen model, upsert with metadata (source URL, page title)
- Chat endpoint: Accept query + optional selected_text, retrieve top-k from Qdrant, build prompt, generate via OpenRouter, stream response with sources

Frontend/Docusaurus:
- Create src/components/ChatbotWidget.jsx (React chat UI with floating button, modal, message history, selected text capture)
- Use window.getSelection() to get highlighted text on "Ask about selection" button
- Integrate with backend via fetch/stream
- Swizzle theme/Footer or add to ClassicTheme to inject widget globally
- Add env var or config for backend URL

Final Tasks:
- Deployment instructions (Qdrant Cloud setup, Neon if used, FastAPI deploy)
- Manual ingestion guide
- Test with book-specific questions

Each task actionable with file paths and descriptions.
```

## RESPONSE_TEXT

Generated comprehensive 547-line task breakdown with:

**100 Total Tasks** organized in 8 phases:
- Phase 1: Setup (9 tasks) - Backend + Frontend project initialization
- Phase 2: Foundational (13 tasks) - Infrastructure blocking all user stories
- Phase 3: US1 (17 tasks) - Query book content anywhere (MVP core)
- Phase 4: US2 (16 tasks) - Ask about selected text (zero-leakage RAG)
- Phase 5: US3 (11 tasks) - Ingest Docusaurus content (pipeline)
- Phase 6: US4 (4 tasks) - Stream responses real-time (P2 enhancement)
- Phase 7: US5 (4 tasks) - Persist chat history (P2 enhancement)
- Phase 8: Polish (32 tasks) - Testing, documentation, security, deployment

**Detailed Task Structure:**
- Each task follows strict format: `[ID] [P?] [Story?] Description with file path`
- T001-T100 sequential numbering
- [P] marks parallelizable tasks (different files, no dependencies)
- [US1-US5] labels map tasks to user stories for traceability
- Exact file paths included (e.g., `fastapi-rag-backend/app/main.py`)

**Separated Repos:**
- Backend: `fastapi-rag-backend/` (separate repository)
- Frontend: `physical-ai-textbook/website/` (update existing Docusaurus)

**Dependencies & Parallel Execution:**
- Phase dependency chain: Setup → Foundational → User Stories → Polish
- Parallel opportunities identified per phase (e.g., all backend [P] tasks can run together)
- Team scaling: Solo dev (~4-5 weeks sequential), Team of 3 (~3-4 weeks parallel)
- User story execution: Can sequence or parallelize after Phase 2 complete

**MVP Definition:**
- Critical: US1 + US2 + US3 (P1 user stories)
- Optional: US4 + US5 (P2 enhancements)
- Testable at each checkpoint

**Key Sections:**
- Parallel examples: Dev team of 3, solo developer sequential paths
- Acceptance criteria checklists per user story
- Testing tasks included (T079-T085) - unit + integration + blind testing
- Deployment preparation (T093-T096)
- Metrics to track (time, accuracy, latency, uptime)

**Actionable Format:**
- All tasks immediately executable (file paths, clear descriptions)
- No vague tasks or ambiguity
- Dependencies explicit (e.g., T041 depends on T040)
- Integration points clear (e.g., T037 swizzles Docusaurus to inject widget)

---

## OUTCOME

✅ 100 actionable tasks generated
✅ Organized by 5 user stories (P1 + P2)
✅ Clear separation: Backend (fastapi-rag-backend), Frontend (website/)
✅ Dependencies and parallelization documented
✅ MVP path defined (US1 + US2 + US3)
✅ Multiple execution strategies provided (solo, team of 3)
✅ Acceptance criteria and metrics included
✅ All tasks immediately implementable

**Status**: Ready to begin Phase 1 Setup

---

## EVALUATION

**Quality Assessment:**
- All tasks follow strict checklist format (ID + [P?] + [Story?] + path + description)
- Excellent parallelization opportunities (Teams can work on separate stories simultaneously)
- User stories are independently testable and deliverable
- Clear milestones/checkpoints after each user story
- Realistic time estimates and parallel execution examples
- MVP scope is well-defined (P1 only = launchable product)

**Readiness for Execution:**
- All tasks are specific and actionable
- File paths are accurate per architecture plan
- No ambiguous or vague task descriptions
- Task dependencies are explicit
- Sequential and parallel paths provided for different team sizes

**Strengths:**
1. Excellent parallelization for team of 3 (parallel user story development)
2. Clear dependency graph prevents blocking issues
3. Acceptance criteria per story ensures quality gates
4. MVP definition prevents scope creep (P1 only = launch)
5. Realistic time estimates with parallel benefits
6. Testing strategy integrated throughout (not bolted on at end)
7. Separation of repos (backend/frontend) enables independent deployment
8. Detailed parallel examples help teams understand execution
