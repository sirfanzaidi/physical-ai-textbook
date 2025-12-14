---
description: "Task list for RAG Chatbot implementation (7 phases, 4 user stories)"
---

# Tasks: Integrated RAG Chatbot for Published Books

**Input**: Design documents from `/specs/001-rag-chatbot/`
**Prerequisites**: plan.md (UPDATED with FR-001a & FR-007a), spec.md (UPDATED with FR-001a & FR-007a), .env.example (COMPLETE), data-model.md (COMPLETE)
**New Requirements**: FR-001a (Atomic Units in Chunking), FR-007a (Select-Text Validation - Minimum 10 Characters)

**Tests**: NOT included (not explicitly requested in spec)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions
- Note: Setup and Foundational phases have NO story labels (shared infrastructure)

## Path Conventions

- **Backend**: `backend/src/`, `backend/tests/`
- **Frontend**: `frontend/`
- **Configuration**: `.env.example`, `requirements.txt`
- **Paths shown below assume full repo structure from plan.md**

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, environment configuration, and service connection verification

- [ ] T001 Initialize Git repository structure with directories per plan.md
- [ ] T002 Create `requirements.txt` with Python dependencies (FastAPI, Cohere SDK, qdrant-client, psycopg2, PyPDF2, python-dotenv, pydantic-settings)
- [ ] T003 Create `backend/__init__.py` and establish backend package structure
- [ ] T004 Create `frontend/` directory structure (chat-widget.js, chat-widget.css, demo.html, utils.js)
- [ ] T005 [P] Create `.env.example` template with placeholders (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DB_URL, QDRANT_COLLECTION)
- [ ] T006 [P] Create `backend/app/config.py` to load environment variables via pydantic-settings
- [ ] T007 [P] Create `backend/app/main.py` with FastAPI app initialization and middleware (CORS, logging)
- [ ] T008 Create Qdrant collection: `book_vectors` (dimension 1024, metric cosine) via initialization script `backend/scripts/setup_qdrant.py`
- [ ] T009 Create Neon metadata table `chunks_metadata` via migration script `backend/scripts/setup_neon.py`
- [ ] T010 Write smoke test `backend/tests/unit/test_connections.py` to verify Cohere, Qdrant, Neon connectivity
- [ ] T011 Run smoke tests and confirm all external service connections are green

**Checkpoint**: Foundation infrastructure ready - all external services verified connected

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure and utilities that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T012 [P] Create `backend/app/middleware.py` with error handling, CORS, request logging middleware
- [ ] T013 [P] Create `backend/app/api/models.py` with Pydantic schemas for request/response validation
- [ ] T014 [P] Create `backend/utils/errors.py` with custom exception classes (CoherAPIError, QdrantError, NeonError, ValidationError)
- [ ] T015 [P] Create `backend/utils/logging.py` with structured logging setup (JSON format, query tracking)
- [ ] T016 [P] Create `backend/utils/metrics.py` with latency and accuracy tracking utilities
- [ ] T017 Create `backend/database/qdrant_client.py` with Qdrant connection, upsert, search, delete operations
- [ ] T018 Create `backend/database/postgres_client.py` with Neon connection, insert, query, delete operations for chunks_metadata
- [ ] T019 [P] Create `backend/ingestion/chunker.py` with semantic chunking logic (300-500 tokens, 200 overlap, recursive splitting) **with atomic-unit preservation (FR-001a)**
- [ ] T020 Create `backend/ingestion/embedder.py` with Cohere embed-v4.0 integration (input_type="search_document" for storage, batch processing)
- [ ] T021 Create `backend/retrieval/retriever.py` with Qdrant search logic (top_k=20-30, similarity threshold)
- [ ] T022 Create `backend/retrieval/reranker.py` with Cohere rerank-v4.0-pro integration (final top_k=8-10)
- [ ] T023 Create `backend/retrieval/augmenter.py` to assemble document list for Cohere Chat API
- [ ] T024 Create `backend/generation/prompts.py` with system prompts enforcing grounding and citations
- [ ] T025 Create `backend/generation/rag_chat.py` with Cohere Chat API integration (documents parameter, streaming)
- [ ] T026 [P] Create `backend/app/api/routes.py` stub with route definitions (POST /ingest, POST /chat, POST /chat-stream, GET /health)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query Book Content with RAG (Priority: P1) 🎯 MVP

**Goal**: Users can ask questions about indexed book content and receive accurate, grounded answers within 5 seconds

**Independent Test**: Can be fully tested by loading a sample book, indexing into Qdrant, submitting /query endpoint calls, verifying grounded responses

### Implementation for User Story 1

- [ ] T027 [P] [US1] Create `backend/ingestion/main.py` with book file upload and processing pipeline
- [ ] T028 [US1] Implement PDF/text extraction in `backend/ingestion/main.py` using PyPDF2/unstructured
- [ ] T029 [P] [US1] Write unit test `backend/tests/unit/test_chunker.py` for semantic chunking (token counts, boundary preservation) **including atomic-unit validation (FR-001a)**: verify code blocks, tables, equations are never split mid-unit
- [ ] T030 [P] [US1] Write unit test `backend/tests/unit/test_embedder.py` for Cohere embed-v4.0 integration
- [ ] T031 [US1] Implement POST /ingest endpoint in `backend/app/api/routes.py` to accept book files, validate (max 500 pages), trigger ingestion
- [ ] T032 [US1] Implement query embedding in `backend/retrieval/retriever.py` using Cohere embed-v4.0 (input_type="search_query")
- [ ] T033 [US1] Implement Qdrant vector search in `backend/retrieval/retriever.py` (top_k=20-30 with threshold)
- [ ] T034 [US1] Implement reranking in `backend/retrieval/reranker.py` using Cohere rerank-v4.0-pro (final top_k=8-10)
- [ ] T035 [US1] Implement document assembly in `backend/retrieval/augmenter.py` with id, title, text, source (page, section)
- [ ] T036 [US1] Implement Cohere Chat integration in `backend/generation/rag_chat.py` with documents parameter
- [ ] T037 [P] [US1] Write unit test `backend/tests/unit/test_retriever.py` for vector search and reranking logic
- [ ] T038 [P] [US1] Write unit test `backend/tests/unit/test_rag_chat.py` for generation logic (system prompt enforcement)
- [ ] T039 [US1] Implement POST /chat endpoint in `backend/app/api/routes.py` to accept query, retrieve docs, call Cohere Chat, return response + citations
- [ ] T040 [US1] Implement GET /health endpoint in `backend/app/api/routes.py` for health checks
- [ ] T041 [P] [US1] Write integration test `backend/tests/integration/test_ingest_to_query.py` for end-to-end flow (index → query → response)
- [ ] T042 [US1] Index sample book (50-100 pages) and test 10+ queries manually via /query endpoint
- [ ] T043 [US1] Verify responses are grounded in book text (no hallucinations) and contain source attribution

**Checkpoint**: User Story 1 should be fully functional and independently testable. Can submit queries and receive accurate, grounded responses.

---

## Phase 4: User Story 2 - Select Text and Ask Questions (Priority: P1)

**Goal**: Users can highlight text passages and ask questions about ONLY that passage with zero leakage from rest of book

**Independent Test**: Can be fully tested by highlighting passages, submitting select-text queries, verifying zero leakage (no results from outside selected passage)

### Implementation for User Story 2

- [ ] T044 [P] [US2] Implement select-text mode in `backend/retrieval/retriever.py` with text_hash filtering (constrain results to selected passage)
- [ ] T045 [US2] Modify POST /chat endpoint to accept `mode` ("full" | "selected") and `selected_text` parameters
- [ ] T046 [US2] Implement selected text validation in `backend/app/api/routes.py` **(FR-007a)**: minimum 10 characters, return HTTP 400 with message "Selected text must be at least 10 characters" if <10 chars, and "Selected text is required for select-text mode" if empty/null
- [ ] T047 [P] [US2] Write unit test `backend/tests/unit/test_select_text_filtering.py` to verify text_hash matching, zero-leakage constraint, **and FR-007a validation**: test empty selection, 1-9 char rejection, 10+ char acceptance with specific HTTP 400 error messages
- [ ] T048 [P] [US2] Write integration test `backend/tests/integration/test_select_text_mode.py` (10+ select-text queries, verify zero leakage, **FR-007a edge cases**: test empty selection, 5-char selection (expect HTTP 400), 10-char selection (expect success), etc.)
- [ ] T049 [US2] Create `frontend/chat-widget.js` Vanilla JS chat component with message display and input field
- [ ] T050 [US2] Implement text selection detection in `frontend/chat-widget.js` and "Ask about this" button **(FR-007a frontend validation)**: reject empty selections, warn if 1-9 characters, allow ≥10 characters, handle HTTP 400 errors from backend
- [ ] T051 [US2] Implement message history storage in `frontend/chat-widget.js` via browser localStorage
- [ ] T052 [US2] Create `frontend/chat-widget.css` with responsive styling (mobile-friendly, customizable)
- [ ] T053 [P] [US2] Create `frontend/utils.js` with message formatting, API call helpers, response parsing
- [ ] T054 [US2] Implement streaming support in `frontend/chat-widget.js` for real-time response display
- [ ] T055 [US2] Implement POST /chat-stream endpoint in `backend/app/api/routes.py` for streaming responses (reduces latency perception)
- [ ] T056 [US2] Create `frontend/demo.html` example page with embedded chat widget in iframe
- [ ] T057 [US2] Test select-text feature manually: highlight passages, ask questions, verify responses are ONLY from selection

**Checkpoint**: User Stories 1 AND 2 should both work independently. Full-book and select-text modes fully functional.

---

## Phase 5: User Story 3 - Admin: Index and Manage Book Content (Priority: P2)

**Goal**: Admins can upload books, manage indexing, and re-index updated content without duplication

**Independent Test**: Can be fully tested by uploading PDFs/text files to /ingest endpoint, verifying chunks in Qdrant, testing re-indexing

### Implementation for User Story 3

- [ ] T058 [P] [US3] Create `backend/ingestion/storage.py` with Qdrant upsert logic (idempotent, with book_id deduplication)
- [ ] T059 [P] [US3] Create `backend/ingestion/storage.py` with Neon metadata logging (chunk_id, page_num, section_name, created_at)
- [ ] T060 [US3] Implement book file validation in `backend/ingestion/main.py` (max 500 pages, check file size)
- [ ] T061 [US3] Implement re-indexing logic in `backend/ingestion/main.py` (delete old chunks by book_id, upsert new)
- [ ] T062 [US3] Implement warning for large books in `backend/ingestion/main.py` (>500 pages: warn or provide guidance)
- [ ] T063 [P] [US3] Write unit test `backend/tests/unit/test_ingestion_main.py` for file validation and book processing
- [ ] T064 [P] [US3] Write integration test `backend/tests/integration/test_book_reindexing.py` (upload → delete → re-upload, verify no duplication)
- [ ] T065 [US3] Enhance POST /ingest endpoint with detailed status response (book_id, chunk_count, estimated storage used)
- [ ] T066 [US3] Test indexing with 2-3 sample books (50, 200, 500 pages) to verify chunking and storage

**Checkpoint**: User Story 3 should allow admins to upload and re-index books without duplication or data loss.

---

## Phase 6: User Story 4 - Track Accuracy and Quality (Priority: P2)

**Goal**: Developers can validate chatbot accuracy (≥90% blind test target) and monitor performance metrics

**Independent Test**: Can be fully tested by running 50+ blind test queries and evaluating accuracy, latency, and hallucination detection

### Implementation for User Story 4

- [ ] T067 [P] [US4] Create `backend/tests/blind_eval/test_accuracy_50_queries.py` with 50+ unseen test queries (NOT used during development)
- [ ] T068 [US4] Create `backend/utils/metrics.py` enhancements for accuracy tracking (query, response, accuracy_flag, latency)
- [ ] T069 [P] [US4] Implement structured logging in `backend/utils/logging.py` to capture all queries with timestamps and responses
- [ ] T070 [P] [US4] Implement latency tracking in `backend/generation/rag_chat.py` (p50, p95, p99 latencies)
- [ ] T071 [US4] Create evaluation script `backend/scripts/evaluate_accuracy.py` to run blind tests and generate accuracy report
- [ ] T072 [US4] Run blind eval on sample book: 50+ queries, measure accuracy %, latency distribution, identify failure modes
- [ ] T073 [US4] Document accuracy findings in `backend/metrics/accuracy_report.md` (accuracy %, citation quality, hallucination detection)
- [ ] T074 [P] [US4] Write test `backend/tests/unit/test_metrics.py` for latency and accuracy tracking utilities

**Checkpoint**: User Story 4 should enable accuracy validation with blind tests ≥90% accuracy gates production release.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories, documentation, and final code review

- [ ] T075 [P] Create comprehensive `README.md` with project overview, setup instructions, API documentation, deployment guide
- [ ] T076 [P] Create `backend/quickstart.md` with local dev 5-minute setup (Python env, dependencies, .env config)
- [ ] T077 [P] Create `backend/DATA_MODEL.md` with entity diagrams (Book, Chunk, Query, Session) and database schemas
- [ ] T078 [P] Create API contracts in `backend/contracts/` directory (OpenAPI YAML for /ingest, /chat, /chat-stream, /health)
- [ ] T079 Run full test suite: `pytest backend/tests/ -v --cov=backend --cov-report=term` to verify >80% code coverage
- [ ] T080 [P] Code review: PEP 8 compliance check `flake8 backend/` and `black --check backend/`
- [ ] T081 [P] Add docstrings to all public methods in `backend/app/`, `backend/ingestion/`, `backend/retrieval/`, `backend/generation/`
- [ ] T082 [P] Add error handling for edge cases: API timeouts, invalid file formats, empty selections, concurrent updates
- [ ] T083 [P] Create security audit checklist: Input sanitization, prompt injection prevention, credential protection, HTTPS enforcement
- [ ] T084 [P] Update `frontend/demo.html` with multiple example queries and responses demonstrating both full-book and select-text modes
- [ ] T085 [P] Performance optimization: Tune chunk_size, top_k, rerank_model based on blind eval results
- [ ] T086 [P] Monitor Cohere API usage: Estimate monthly token usage, verify within free tier limits
- [ ] T087 [P] Monitor Qdrant storage: Estimate chunk storage size, verify within free tier (~10GB for ~50k chunks)
- [ ] T088 Create `DEPLOYMENT.md` with production deployment guide (Vercel for backend, GitHub Pages for widget)
- [ ] T089 Create `TROUBLESHOOTING.md` with common issues and solutions (API errors, latency, accuracy troubleshooting)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User Story 1 (P1): No dependencies on other stories - can start after Foundational
  - User Story 2 (P1): No dependencies on other stories - can start after Foundational (benefits from US1 working, but can be tested independently)
  - User Story 3 (P2): No dependencies on other stories - can start after Foundational
  - User Story 4 (P2): Depends on US1 + US2 implementation (needs working chatbot to test)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1 MVP)**: Can start after Foundational - No dependencies on other stories
  - Independent test: Index sample book → query → verify grounded responses
  - Can be deployed as standalone MVP

- **User Story 2 (P1)**: Can start after Foundational - Independent from US1 (but best after US1 core works)
  - Independent test: Select-text queries → verify zero-leakage
  - Frontend and backend components can be built in parallel

- **User Story 3 (P2)**: Can start after Foundational - No dependencies on US1/US2
  - Independent test: Upload books → verify in Qdrant → test re-indexing
  - Can be implemented in parallel with US1/US2

- **User Story 4 (P2)**: Depends on US1 + US2 (needs working chatbot + select-text to test)
  - Independent test: Run 50+ blind queries → measure accuracy
  - Can start after US1/US2 core functionality is working

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- **Setup Phase**: All [P] tasks can run in parallel (T005, T006, T007)
- **Foundational Phase**: All [P] tasks can run in parallel (T012-T026 where marked [P])
- **User Story 1**:
  - Tests can run in parallel (T029, T030, T037, T038)
  - Ingestion/retrieval/generation can be built in parallel (T027-T028 → T032-T034 → T035 → T036)
- **User Story 2**:
  - Frontend and backend can be built in parallel (T049-T054 in parallel with T044-T048)
- **User Story 3**:
  - Unit tests can run in parallel (T058, T059, T063, T064)
- **Polish Phase**: All [P] tasks can run in parallel (T075-T087 where marked [P])

---

## Parallel Example: User Story 1 (MVP)

```bash
# Launch all tests for User Story 1 together:
Task: T029 - Write unit test for chunker
Task: T030 - Write unit test for embedder
Task: T037 - Write unit test for retriever
Task: T038 - Write unit test for rag_chat

# Launch all core implementation together (after tests):
Task: T027 - Create ingestion main
Task: T032 - Implement query embedding
Task: T033 - Implement Qdrant search
Task: T035 - Implement document assembly
Task: T036 - Implement Cohere Chat

# Final integration:
Task: T039 - Implement /chat endpoint
Task: T041 - Integration test
Task: T042-T043 - Manual verification
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (environment, services verified)
2. Complete Phase 2: Foundational (all base utilities and clients ready)
3. Complete Phase 3: User Story 1 (query functionality working)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy to staging and demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (Enhanced)
4. Add User Story 3 → Test independently → Integrate with demo (Admin feature)
5. Add User Story 4 → Run blind tests → Verify accuracy gate ✓ (Launch ready!)

### Parallel Team Strategy

With 3-4 developers:

1. Everyone completes Setup + Foundational together (~3 days)
2. Once Foundational is done:
   - Developer A: User Story 1 (query core RAG)
   - Developer B: User Story 2 (select-text + frontend)
   - Developer C: User Story 3 (admin indexing)
   - Developer D: Documentation + Polish (after stories)
3. Developer A finishes US1 → enables US4 to start (accuracy testing)
4. Merge stories and integrate → Single cohesive chatbot
5. Final polish and deployment

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing (TDD approach optional but recommended)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
