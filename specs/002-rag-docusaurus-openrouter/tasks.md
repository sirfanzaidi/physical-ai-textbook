# Tasks: RAG Chatbot with Docusaurus & OpenRouter

**Input**: Design documents from `/specs/002-rag-docusaurus-openrouter/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

**Repos**:
- **Backend**: `fastapi-rag-backend/` (separate repository)
- **Frontend**: `physical-ai-textbook/website/` (update existing Docusaurus repo)

---

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story label (US1, US2, US3, US4, US5)
- **Exact file paths** are included in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize both backend and frontend projects

### Backend Setup

- [ ] T001 Create backend repository structure: `fastapi-rag-backend/` with `app/`, `tests/`, `requirements.txt`, `.env.example`, `pyproject.toml`
- [ ] T002 [P] Initialize FastAPI project in `fastapi-rag-backend/app/main.py` with CORS middleware, logging, error handlers
- [ ] T003 [P] Create `fastapi-rag-backend/requirements.txt` with dependencies: fastapi, uvicorn, openai, qdrant-client, langchain, pydantic, python-dotenv, httpx, aiohttp
- [ ] T004 [P] Create `fastapi-rag-backend/app/config.py` with Settings class (OpenRouter key, Qdrant URL, API endpoints, log level)
- [ ] T005 [P] Create `fastapi-rag-backend/.env.example` template with OPENROUTER_API_KEY, QDRANT_URL, QDRANT_API_KEY, OPENROUTER_MODEL, LOG_LEVEL

### Frontend Setup

- [ ] T006 [P] Create `website/src/config/ragConfig.ts` with API base URL configuration (already exists - verify and update if needed)
- [ ] T007 [P] Create `website/.env.example` with REACT_APP_RAG_API_BASE_URL, REACT_APP_BOOK_ID

### Common

- [ ] T008 [P] Setup Git workflow: branch naming, commit conventions, PR template
- [ ] T009 [P] Create README files for both repos with setup instructions, tech stack, deployment guide outline

**Checkpoint**: Both projects initialized with basic structure and dependencies ready to install

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before user story implementation

### Backend Foundational

- [ ] T010 [P] Create `fastapi-rag-backend/app/dependencies.py` with dependency injection: OpenRouter client factory, Qdrant client factory, logging setup
- [ ] T011 [P] Create `fastapi-rag-backend/app/utils/logging.py` with structured logging configuration (JSON format, request IDs, latency tracking)
- [ ] T012 [P] Create `fastapi-rag-backend/app/utils/errors.py` with custom exceptions: RAGError, RetrievalError, GenerationError, ValidationError, OpenRouterError
- [ ] T013 [P] Create `fastapi-rag-backend/app/utils/validators.py` with input validation: validate_query_length (max 5000 chars), validate_selected_text (min 10 chars), validate_book_id
- [ ] T014 Create `fastapi-rag-backend/app/api/models.py` with Pydantic request/response schemas: ChatRequest, ChatResponse, IngestRequest, IngestResponse, Citation, HealthResponse, ErrorResponse
- [ ] T015 [P] Create `fastapi-rag-backend/app/api/routes/__init__.py` router initialization
- [ ] T016 [P] Create `fastapi-rag-backend/app/api/routes/health.py` with GET /api/health endpoint (returns service status and connectivity checks)
- [ ] T017 Create `fastapi-rag-backend/app/db/qdrant_schemas.py` with Qdrant collection schema definition (vector size, distance metric, payload schema with chunk_id, chapter_id, chapter_name, section_name, content, source_url, page_num)

### Frontend Foundational

- [ ] T018 [P] Create `website/src/services/apiClient.ts` with Axios wrapper for API calls (base URL from config, error handling, request/response interceptors)
- [ ] T019 [P] Create `website/src/hooks/useRAGChat.ts` custom hook with chat state management (query history, pending state, error handling, selected_text management)
- [ ] T020 [P] Create `website/src/utils/textSelection.ts` with utility functions: getSelectedText(), normalizeSelection(), sanitizeText()
- [ ] T021 [P] Create `website/src/components/RAGChat/RAGChat.module.css` with base styling (floating button, chat modal, message display, input box, responsive design)

### OpenRouter Setup (Shared)

- [ ] T022 Create `fastapi-rag-backend/app/services/openrouter_client.py` with OpenRouter API client: embeddings method (Qwen model, retries, error handling), chat method (streaming support, temperature config)

**Checkpoint**: All foundational infrastructure ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Query Book Content Anywhere (Priority: P1) 🎯 MVP

**Goal**: Users can ask questions about book content from any Docusaurus page via a floating widget and receive answers with source citations

**Independent Test**:
1. Start FastAPI backend with indexed test content
2. Load Docusaurus page with widget
3. Type question about book content
4. Verify response appears within 5s with citations linking to source chapters
5. Verify answer is grounded in book content only

### Backend Implementation for US1

- [ ] T023 [P] Create `fastapi-rag-backend/app/services/retrieval/qdrant_store.py` with methods: initialize_collection(), search_vectors(query_embedding, top_k), get_chunk_by_id() (depends on T017)
- [ ] T024 [P] Create `fastapi-rag-backend/app/services/generation/rag_chat.py` with RAGChatbot class: retrieve_context(query, top_k), build_prompt(query, context, mode), generate_response_stream(prompt) using LangChain/LlamaIndex (depends on T022)
- [ ] T025 Create `fastapi-rag-backend/app/api/routes/chat.py` with POST /api/chat-stream endpoint: accept ChatRequest, call RAGChatbot, stream JSON-lines response with response chunks + citations + latency (depends on T024)
- [ ] T026 Add validation in `fastapi-rag-backend/app/api/routes/chat.py`: enforce selected_text min 10 chars if mode="selected", return 400 with clear error message (depends on T013)
- [ ] T027 Add error handling in `fastapi-rag-backend/app/api/routes/chat.py`: catch RetrievalError, GenerationError, provide user-friendly messages, log detailed traces (depends on T012)
- [ ] T028 Add streaming response in `fastapi-rag-backend/app/api/routes/chat.py`: implement JSON-lines streaming for real-time text display on frontend (depends on T025)

### Frontend Implementation for US1

- [ ] T029 [P] Create `website/src/components/RAGChat/ChatWindow.tsx` component: render message list, display citations with links, show loading state, scroll to latest message
- [ ] T030 [P] Create `website/src/components/RAGChat/InputBox.tsx` component: query input field, character counter (max 5000), send button, disabled state while loading
- [ ] T031 [P] Create `website/src/components/RAGChat/FloatingButton.tsx` component: floating action button with icon, opens/closes chat modal, badge if unread messages
- [ ] T032 [P] Create `website/src/components/RAGChat/Citation.tsx` component: render single citation with chapter/section info, clickable link to source page
- [ ] T033 Create `website/src/components/RAGChat/index.tsx` main component: combine ChatWindow + InputBox + FloatingButton, manage chat state via useRAGChat hook, handle API calls
- [ ] T034 Create `website/src/services/streamingClient.ts` helper: parse JSON-lines streaming response, accumulate chunks, handle connection errors, timeout after 30s
- [ ] T035 Update `website/src/components/RAGChat/index.tsx` to integrate streaming: call apiClient.chatStream(), update UI incrementally as chunks arrive, display citations when available
- [ ] T036 Add error handling in `website/src/components/RAGChat/index.tsx`: catch API errors, display user-friendly messages ("Chatbot unavailable", "Invalid input"), retry logic with exponential backoff

### Integration for US1

- [ ] T037 Create `website/src/components/RAGChat/ThemeSwizzle.tsx` or update `website/src/theme/Layout/index.tsx` to inject floating button globally on all pages (ensure no conflicts with Docusaurus styling)
- [ ] T038 Test widget styling: verify widget doesn't break Docusaurus navigation, footer, or mobile layout; runs unit tests for component rendering (depends on T037)
- [ ] T039 Integration test for US1: start backend with test data, load Docusaurus page, submit query, verify response arrives with citations (create manual test checklist in `fastapi-rag-backend/tests/integration/test_chat_endpoint.py`)

**Checkpoint**: User Story 1 complete - users can ask full-book questions with citations. Ready to test independently and demo.

---

## Phase 4: User Story 2 - Ask About Selected Text (Priority: P1)

**Goal**: Users can highlight text and ask targeted questions about just that passage with zero-leakage retrieval

**Independent Test**:
1. Load Docusaurus page with chat widget
2. Highlight a text passage (ensure >10 chars)
3. Click "Ask about this" button (should appear near selection)
4. Submit query
5. Verify response only uses content from highlighted passage
6. Test with query that has answer outside selection → verify "not in selected passage" response

### Backend Implementation for US2

- [ ] T040 [P] Create `fastapi-rag-backend/app/services/retrieval/chunk_matcher.py` with method: find_overlapping_chunks(selected_text, all_chunks) to identify chunks that overlap with selected passage
- [ ] T041 Update `fastapi-rag-backend/app/services/generation/rag_chat.py` method retrieve_context() to accept optional selected_text parameter and constrain retrieval to overlapping chunks only (depends on T040)
- [ ] T042 Update `fastapi-rag-backend/app/api/routes/chat.py` to pass selected_text from request to retrieval layer, add check: if mode="selected" but no overlapping chunks found, respond with "This information is not in the selected passage" (depends on T041)

### Frontend Implementation for US2

- [ ] T043 [P] Create `website/src/components/RAGChat/TextSelectionHandler.tsx` component: listen for text selection (mouseup/touchend), call window.getSelection(), normalize across browsers, validate min 10 chars
- [ ] T044 [P] Create `website/src/components/RAGChat/SelectionButton.tsx` component: positioned popup button "Ask about this" near selected text (use absolute positioning near selection), click to trigger query with selected_text
- [ ] T045 Update `website/src/components/RAGChat/index.tsx` to: detect text selection, show SelectionButton, capture selected_text when user clicks "Ask about this", pass to chat endpoint with mode="selected"
- [ ] T046 Add frontend validation in `website/src/components/RAGChat/TextSelectionHandler.tsx`: warn if selection <10 chars before sending to backend ("Please select at least 10 characters")
- [ ] T047 Update `website/src/hooks/useRAGChat.ts` to manage: selectedText state, mode state (full/selected), preserve selected_text when opening chat via "Ask about this"

### Integration for US2

- [ ] T048 Update `website/src/components/RAGChat/index.tsx` to show selected text context in chat: display highlighted passage at top of chat window with visual styling (gray background, italicized)
- [ ] T049 Cross-browser test for US2: test text selection capture on Chrome, Firefox, Safari, Edge (create test checklist in `website/src/__tests__/TextSelection.test.tsx`)
- [ ] T050 Mobile test for US2: test text selection on iOS/Android browsers, ensure SelectionButton is touch-friendly (document limitations in README if needed)

**Checkpoint**: User Story 2 complete - select-text mode works with zero-leakage. Can be tested independently.

---

## Phase 5: User Story 3 - Ingest Book Content from Docusaurus (Priority: P1)

**Goal**: Admin can run ingestion script to extract Docusaurus markdown, chunk semantically, generate embeddings, and populate Qdrant

**Independent Test**:
1. Run ingestion script pointing to Docusaurus build output
2. Verify all chapters extracted (check log output shows chapter count)
3. Verify chunks created with correct size (800-1200 chars, respects code blocks)
4. Verify Qdrant collection contains all vectors with metadata
5. Query Qdrant directly to verify chunk retrieval works

### Backend Implementation for US3

- [ ] T051 [P] Create `fastapi-rag-backend/app/services/ingestion/docusaurus_crawler.py` with DocusaurusCrawler class: scan docs/ directory, extract markdown files, remove YAML frontmatter, parse HTML (if build output), convert to plain text
- [ ] T052 [P] Create `fastapi-rag-backend/app/services/ingestion/chunker.py` with SemanticChunker class: chunk text into 800-1200 char segments with 150-200 char overlap, respect sentence boundaries, treat code blocks/tables as atomic units (no mid-block splits)
- [ ] T053 Create `fastapi-rag-backend/app/services/ingestion/pipeline.py` with IngestionPipeline class: orchestrate crawler → chunker → embeddings → Qdrant upsert, track progress, handle errors (depends on T051, T052, T022, T023)
- [ ] T054 Create `fastapi-rag-backend/app/api/routes/ingest.py` with POST /api/ingest endpoint: accept IngestRequest (source, book_id, docs_path, chunk_size, overlap), call pipeline, return IngestResponse with chunk count and ingestion time (depends on T053)
- [ ] T055 Create `fastapi-rag-backend/scripts/ingest_docusaurus.py` standalone script: run ingestion from CLI with argument parsing, progress bar, checkpoint support (can resume if interrupted), detailed logging (depends on T053)
- [ ] T056 Add metadata extraction in `fastapi-rag-backend/app/services/ingestion/docusaurus_crawler.py`: extract chapter title, section name, source URL, page number from markdown files and metadata
- [ ] T057 Add error handling in `fastapi-rag-backend/app/services/ingestion/pipeline.py`: log errors per chapter, continue on failures, final summary report (chunks created, failures, warnings)
- [ ] T058 Add validation in `fastapi-rag-backend/app/services/ingestion/pipeline.py`: warn if estimated vectors exceed Qdrant free tier limits (>100K vectors), estimate ingestion time before starting

### Integration for US3

- [ ] T059 Create `fastapi-rag-backend/tests/integration/test_ingest_pipeline.py` integration test: test full ingestion flow with sample markdown, verify chunks created, verify Qdrant has vectors, verify metadata is stored
- [ ] T060 Create ingestion guide in `fastapi-rag-backend/INGESTION_GUIDE.md`: document how to run ingestion script, expected output, troubleshooting (missing chapters, API rate limits, Qdrant connection issues)
- [ ] T061 Create `fastapi-rag-backend/scripts/setup_qdrant.py` helper script: provision Qdrant collection with correct schema, delete and recreate collection for re-ingestion (useful for testing)

**Checkpoint**: User Story 3 complete - ingestion pipeline works end-to-end. Content can be indexed and queried.

---

## Phase 6: User Story 4 - Stream Responses in Real-Time (Priority: P2)

**Goal**: Users see responses appearing character-by-character as they're generated, for better perceived performance

**Independent Test**:
1. Submit query to chatbot
2. Observe response text appearing gradually (not all at once)
3. Watch for 2-3 seconds to confirm streaming effect
4. Verify on slow network (throttle to 3G) that streaming works smoothly

### Backend Implementation for US4

- [ ] T062 Verify streaming in `fastapi-rag-backend/app/services/generation/rag_chat.py`: ensure generate_response_stream() yields chunks with small delays (50ms) for smooth streaming effect
- [ ] T063 Implement backpressure handling in `fastapi-rag-backend/app/api/routes/chat.py`: if client disconnects, stop generating and log gracefully

### Frontend Implementation for US4

- [ ] T064 Update `website/src/services/streamingClient.ts` to handle streaming animations: track chunk arrival time, display chunks with slight delay (50ms) for visual streaming effect
- [ ] T065 Update `website/src/components/RAGChat/ChatWindow.tsx` to render streaming message: use state to accumulate text, re-render on each chunk, show loading cursor while streaming

### Integration for US4

- [ ] T066 Test streaming on slow network: use Chrome DevTools throttle to 3G, verify no timeouts, smooth text appearance

**Checkpoint**: User Story 4 complete - streaming responses look and feel responsive.

---

## Phase 7: User Story 5 - Persist Chat History (Priority: P2)

**Goal**: Users see previous messages when reopening chat widget, with optional database persistence

**Independent Test**:
1. Submit 3-4 questions in chat
2. Close chat widget
3. Reopen chat widget (same session)
4. Verify previous messages appear
5. Close browser tab, reopen Docusaurus page, reopen chat
6. Verify messages persist (if localStorage or Neon enabled)

### Frontend Implementation for US5

- [ ] T067 [P] Update `website/src/hooks/useRAGChat.ts` to persist to localStorage: save chat history (queries + responses) to localStorage under key `rag-chat-history-{bookId}`
- [ ] T068 [P] Update `website/src/hooks/useRAGChat.ts` to load from localStorage on mount: retrieve chat history, restore to UI state
- [ ] T069 [P] Create `website/src/components/RAGChat/ClearHistoryButton.tsx` component: button in chat header, click to clear localStorage history and UI state, confirmation dialog
- [ ] T070 Update `website/src/components/RAGChat/index.tsx`: display "Clear History" button in chat footer, wire to clear action

### Backend Implementation for US5 (Optional - Neon Postgres)

- [ ] T071 Create `fastapi-rag-backend/app/db/postgres.py` with PostgresStore class: connect to Neon Postgres, save chat sessions, save individual queries (optional - only if advanced persistence needed)
- [ ] T072 Create `fastapi-rag-backend/app/api/routes/history.py` with GET /api/history endpoint: retrieve chat history from database if enabled (optional feature)

### Integration for US5

- [ ] T073 Test localStorage persistence: submit query, close/reopen chat, verify message appears; close browser, reopen site, verify history persists
- [ ] T074 Document localStorage behavior in `website/README.md`: explain that history is device-specific, not synced across devices

**Checkpoint**: User Story 5 complete - users can see previous conversations.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements affecting multiple stories, documentation, security, performance

### Documentation

- [ ] T075 [P] Create `fastapi-rag-backend/README.md` with: tech stack, setup instructions, local dev environment, API docs reference, deployment guide outline
- [ ] T076 [P] Create `fastapi-rag-backend/API.md` with: detailed endpoint descriptions, request/response examples, error codes, rate limiting info
- [ ] T077 [P] Create `website/CHATBOT_SETUP.md` with: how to integrate widget into Docusaurus, config options, troubleshooting
- [ ] T078 [P] Create `fastapi-rag-backend/DEPLOYMENT.md` with: Qdrant Cloud setup, Neon Postgres setup (optional), FastAPI deployment on Render/Fly.io, environment variables checklist

### Testing & Quality

- [ ] T079 [P] Create `fastapi-rag-backend/tests/unit/test_chunker.py`: unit tests for SemanticChunker with various text inputs (code blocks, tables, long lines, edge cases)
- [ ] T080 [P] Create `fastapi-rag-backend/tests/unit/test_validators.py`: unit tests for all validators (query length, selected_text min 10 chars, book_id format)
- [ ] T081 [P] Create `fastapi-rag-backend/tests/unit/test_openrouter_client.py`: unit tests with mocked OpenRouter API (test retry logic, error handling)
- [ ] T082 [P] Create `website/src/__tests__/RAGChat.test.tsx`: component tests for RAGChat widget rendering, state management, API error handling
- [ ] T083 Run backend unit tests: `pytest fastapi-rag-backend/tests/unit/` (all must pass)
- [ ] T084 Run frontend component tests: `npm test` in website/ (all must pass)
- [ ] T085 Create `fastapi-rag-backend/tests/integration/test_full_flow.py`: end-to-end test ingestion → query → stream response with real Qdrant/OpenRouter (mocked)

### Security & Performance

- [ ] T086 [P] Security review: verify no API keys hardcoded, all secrets in `.env`, HTTPS enforced in config for production, CORS properly restricted
- [ ] T087 [P] Performance profiling: measure chat endpoint latency (target <5s p95), measure embedding generation time (target <2s)
- [ ] T088 [P] Load testing: test concurrent users (target ≥10 simultaneous queries), verify Qdrant doesn't timeout
- [ ] T089 Implement rate limiting in `fastapi-rag-backend/app/middleware/rate_limit.py`: limit per IP or API key (OpenRouter rate limits), return 429 if exceeded

### Blind Testing & Accuracy

- [ ] T090 Create `fastapi-rag-backend/BLIND_TEST_QUERIES.md` with 50+ test questions covering all chapters (don't use during development)
- [ ] T091 Run blind test: execute 50+ queries, manually evaluate accuracy, measure p95 latency, document results in `fastapi-rag-backend/BLIND_TEST_RESULTS.md` (target ≥90% accuracy)
- [ ] T092 Analyze failures: categorize errors (chunking issues, retrieval gaps, hallucinations), propose fixes

### Deployment Preparation

- [ ] T093 [P] Create Docker support: `fastapi-rag-backend/Dockerfile` for containerized deployment
- [ ] T094 [P] Create `fastapi-rag-backend/docker-compose.yml` for local development (FastAPI + Qdrant mock)
- [ ] T095 Create deployment scripts: `fastapi-rag-backend/scripts/deploy_to_render.sh` (or Fly.io equivalent)
- [ ] T096 Create `website/build-and-test.sh`: build Docusaurus, verify widget loads, test API connectivity

### Final Integration

- [ ] T097 Full end-to-end test: ingest Physical AI textbook chapters, load Docusaurus site, test all user stories (US1-US5), verify accuracy, measure performance
- [ ] T098 Create `QUICKSTART.md` at repo root: 5-minute guide to clone, setup, run locally (backend + frontend + ingestion)
- [ ] T099 Documentation review: proofread all READMEs, API docs, guides for clarity and completeness
- [ ] T100 Code cleanup: run linters, fix style issues, remove debug logs, ensure no TODOs left unresolved

**Checkpoint**: All features complete, tested, documented, ready for production launch.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately ✅
- **Foundational (Phase 2)**: Depends on Setup completion - **BLOCKS all user stories**
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - **US1, US2, US3**: Must complete (P1) - form MVP
  - **US4, US5**: Optional (P2) - nice-to-have enhancements
  - **Execution**: Sequential or parallel by team capacity
- **Polish (Phase 8)**: Depends on desired user stories being complete

### User Story Execution Order

#### MVP Path (Minimum for launch)
1. Phase 1: Setup ✅
2. Phase 2: Foundational ✅
3. Phase 3: **US1** - Core Q&A feature (CRITICAL)
4. Phase 4: **US2** - Select-text mode (CRITICAL)
5. Phase 5: **US3** - Ingestion pipeline (CRITICAL)
6. Phase 8: Polish, documentation, testing

#### Full Feature Path (with enhancements)
1. Same as MVP, then:
7. Phase 6: **US4** - Streaming responses (enhances UX)
8. Phase 7: **US5** - Chat history (nice-to-have)
9. Phase 8: Polish

### Parallel Opportunities

**Within Phase 1 (Setup)**:
- All backend setup tasks marked [P] can run in parallel (T002-T005)
- All frontend setup tasks marked [P] can run in parallel (T006-T007)
- Common setup (T008-T009) can run in parallel

**Within Phase 2 (Foundational)**:
- All backend tasks marked [P] can run in parallel (T010-T013, T015-T016)
- All frontend tasks marked [P] can run in parallel (T018-T021)
- Once Phase 2 complete, can assign teams to different user stories

**Within User Stories**:
- Backend and frontend can be developed in parallel (different files, different repos)
- Example: Dev A works on US1 backend (T023-T028) while Dev B works on US1 frontend (T029-T036)

**Between User Stories (after Phase 2)**:
- Dev A: US1 backend + frontend (T023-T039)
- Dev B: US2 modifications (T040-T050)
- Dev C: US3 ingestion (T051-T061)

### Task Dependencies Within Stories

**User Story 1**:
- T023 (search) ← T017 (schema)
- T024 (RAG) ← T023 + T022
- T025 (endpoint) ← T024
- T029-T032 (components) ← no deps, parallel ✅
- T033 (main) ← T029-T032
- T034-T035 (streaming) ← T033
- T037 (integration) ← T033
- T039 (test) ← T025 + T033

**User Story 2**:
- T040 (matcher) ← T017
- T041 (RAG update) ← T040
- T042 (endpoint update) ← T041
- T043-T044 (components) ← no deps, parallel ✅
- T045-T047 (integration) ← T043-T044

**User Story 3**:
- T051-T052 (crawler + chunker) ← no deps, parallel ✅
- T053 (pipeline) ← T051 + T052 + T022 + T023
- T054 (endpoint) ← T053
- T055 (script) ← T053
- T056-T057 (enhancements) ← T053
- T059-T061 (testing) ← T053-T055

---

## Parallel Example: Complete Team (3 Developers)

### Week 1: Setup + Foundational (All Together)
```
Team completes Phase 1 (Setup) in parallel
Team completes Phase 2 (Foundational) in parallel
Checkpoint: Foundation ready
```

### Week 2-3: User Stories in Parallel
```
Developer 1: User Story 1 (US1 Backend + Frontend)
  T023-T039 (backend chat endpoint + frontend widgets)

Developer 2: User Story 2 (US2 Modifications)
  T040-T050 (select-text backend + frontend)

Developer 3: User Story 3 (US3 Ingestion)
  T051-T061 (ingestion pipeline + CLI script)

All three work in parallel, no file conflicts
Each story independently testable
```

### Week 4: Polish (All Together or Sequential)
```
One dev does documentation (T075-T078)
One dev does testing (T079-T085)
One dev does security/performance (T086-T089)
Or: Sequence if fewer resources
```

---

## Parallel Example: Single Developer (Sequential)

### Recommended Order for Time Efficiency
```
Phase 1: Setup (1-2 days)
Phase 2: Foundational (2-3 days)
Phase 3: US1 (5-7 days) - Backend + Frontend
Phase 4: US2 (3-4 days) - Incremental changes
Phase 5: US3 (4-5 days) - Ingestion pipeline
Phase 6: US4 (1-2 days) - Streaming (minimal)
Phase 7: US5 (1-2 days) - localStorage (simple)
Phase 8: Polish (5-7 days) - Testing, docs, deployment
Total: ~4-5 weeks
```

---

## MVP Definition

**Minimum Viable Product** = Complete US1, US2, US3 (Phase 3-5)

**Ready to launch when**:
- ✅ Phase 1 Setup complete
- ✅ Phase 2 Foundational complete
- ✅ Phase 3 US1 complete and independently tested
- ✅ Phase 4 US2 complete and integration tested with US1
- ✅ Phase 5 US3 complete with all chapters indexed
- ✅ Blind testing done (≥90% accuracy confirmed)
- ✅ Security review passed
- ✅ Documentation complete

**Do NOT include in MVP**:
- ❌ US4 (streaming) - nice-to-have
- ❌ US5 (history) - nice-to-have
- ❌ Advanced analytics
- ❌ Multi-language support

---

## Implementation Strategy

### For Solo Developer
1. Complete Setup + Foundational first (blocking)
2. Build US1 completely (backend → frontend → integrate)
3. Add US2 (incremental)
4. Add US3 (server-side)
5. Test and document
6. Deploy
7. Later: Add US4, US5 as time permits

### For Team (2-3 Developers)
1. All do Setup + Foundational together (faster knowledge sharing)
2. Once Foundational done, split into user stories:
   - Dev 1 leads US1 (critical path)
   - Dev 2 leads US2 (parallel)
   - Dev 3 leads US3 (parallel)
3. Dev 1 reviews code from Dev 2/3 as merge conflicts arise
4. Integrate and test together
5. Polish and deploy together

### Testing Strategy
- **Unit tests** (T079-T082): Write first, ensure fail before implementation ✅
- **Integration tests** (T085, T059): After features working independently
- **Blind testing** (T091): After all features integrated
- **Manual smoke test** (T097): Final before deployment

---

## Acceptance Criteria Checklist

### User Story 1 Ready When
- [ ] Floating button appears on all Docusaurus pages
- [ ] Chat window opens without errors
- [ ] Can submit query and receive response within 5s
- [ ] Response includes citations with chapter/section info
- [ ] Citations link to correct Docusaurus pages
- [ ] No external knowledge in responses (book-only)
- [ ] Error messages user-friendly for common cases

### User Story 2 Ready When
- [ ] Text selection detection works (Chrome, Firefox, Safari)
- [ ] "Ask about this" button appears near selection
- [ ] Selected text passed to backend (visible in chat context)
- [ ] Responses constrained to selected passage only
- [ ] "Not in selection" message when answer outside selection
- [ ] Min 10 chars validation works (frontend warning + backend error)
- [ ] Mobile text selection works (touch-friendly)

### User Story 3 Ready When
- [ ] Ingestion script runs without errors
- [ ] All chapters extracted and chunked
- [ ] Chunks respect code blocks, tables (no mid-block splits)
- [ ] Embeddings generated and upserted to Qdrant
- [ ] Metadata (chapter, section, URL) stored correctly
- [ ] Collection queryable (test search works)
- [ ] Progress logging shows status
- [ ] Handles errors gracefully (skip bad chapters, log, continue)

---

## Key Metrics to Track

- **Backend Setup Time**: T001-T009 (target 1-2 days)
- **Foundational Setup Time**: T010-T022 (target 2-3 days)
- **US1 Implementation Time**: T023-T039 (target 5-7 days)
- **US2 Implementation Time**: T040-T050 (target 3-4 days)
- **US3 Implementation Time**: T051-T061 (target 4-5 days)
- **Total Time to MVP**: Target 4-5 weeks for solo dev, 3-4 weeks for team of 2-3
- **Code Quality**: All tests passing before PR merge
- **Accuracy**: ≥90% on blind tests before launch
- **Latency**: p95 <5 seconds, p99 <8 seconds
- **Uptime**: Deploy with minimal downtime

---

## Notes

- Each task marked [P] can run in parallel (different files, no task dependencies)
- Each task marked with [Story] maps to user story, ensuring traceability
- Checkpoint commits after each user story completion
- Stop at any checkpoint to validate independently before continuing
- Avoid: crossing user story boundaries in same PR, merging incomplete stories
- Document as you code: comments, docstrings, README updates concurrent with implementation

---

**Next Steps**:
1. Review task list with team
2. Assign tasks/stories to developers
3. Begin Phase 1 Setup
4. After Phase 2 complete, begin user story work in parallel
5. Use checkpoints for validation before moving forward

**Total Task Count**: 100 tasks
**Tasks per User Story**: US1 (17 tasks), US2 (16 tasks), US3 (11 tasks), US4 (4 tasks), US5 (4 tasks), Polish (32 tasks)
**MVP Critical Tasks**: T001-T084 (covers Setup + Foundational + US1 + US2 + US3 + Core Tests)
