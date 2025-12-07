# Tasks: AI-Native Textbook with RAG Chatbot

**Feature**: `1-textbook-rag` | **Branch**: `1-textbook-rag` | **Date**: 2025-12-06 | **Status**: Ready for Implementation

**Overview**: Hierarchical task breakdown for 6-chapter Physical AI & Humanoid Robotics textbook with RAG chatbot. Organized by user story (P1, P2) with independent test criteria and dependencies. Estimated effort: 4-6 weeks for MVP (US1, US2, US4).

---

## Implementation Strategy

### MVP Scope (Weeks 1-4)

**What ships**: Readable textbook (6 chapters) + RAG chatbot (select-text → question)

**User Stories**: US1 (Reader Explores), US2 (Reader Asks Chatbot), US4 (Content Author Publishes)

**Out of scope**: Search (US3), Admin Dashboard (US5), Urdu/Personalization (P3)

### Phase Structure

| Phase | Title | Effort | Duration | Gating |
|-------|-------|--------|----------|--------|
| **Phase 1** | Project Setup | 1-2 days | Week 1 | Must complete before Phase 2 |
| **Phase 2** | Foundational Infrastructure | 3-4 days | Week 1-2 | Blocks all user story phases |
| **Phase 3** | US1 - Reader Explores Chapters | 3-4 days | Week 2 | Independent; P1 priority |
| **Phase 4** | US2 - Reader Asks Chatbot | 5-7 days | Week 2-3 | Depends on Phase 2 (RAG backend) |
| **Phase 5** | US4 - Content Author Publishes | 3-4 days | Week 3 | Depends on Phase 2 (CI/CD) |
| **Phase 6** | Testing & Deployment | 3-5 days | Week 4 | Final validation + deploy to GitHub Pages |

### Parallelization Opportunities

- **US1 & US4 can overlap**: Docusaurus setup (US1) and content authoring CI (US4) are independent after framework is ready
- **Chapter writing concurrent with backend dev**: Authors can write chapter Markdown in parallel while backend RAG system is built
- **RAG indexing decoupled from frontend**: Backend embedding/vector DB work doesn't block frontend chapter reading
- **Admin dashboard (US5) is optional**: Can be added post-MVP without blocking textbook launch

---

## Phase 1: Project Setup (Week 1)

**Goal**: Initialize repositories, configure tooling, establish deployment pipelines.

**Independent Test**: `npm run build` succeeds locally; `git push` triggers GitHub Actions without errors.

### Setup Tasks

- [ ] T001 Clone or create repository structure per plan.md (`website/` + `backend/` folders) at `D:\physical-ai-textbook`
- [ ] T002 Initialize Docusaurus 3.x project in `website/` with `npx create-docusaurus@latest website classic --typescript`
- [ ] T003 Initialize Python FastAPI backend in `backend/` with `poetry init` (or venv + requirements.txt)
- [ ] T004 Create `.gitignore` for Node.js (node_modules, build/) and Python (venv/, __pycache__, *.pyc, .env)
- [ ] T005 Create `.env.example` template in `backend/` with placeholders: `CHROMADB_PATH`, `OPENAI_API_KEY`, `RAG_API_URL`
- [ ] T006 Configure GitHub Actions workflow skeleton in `.github/workflows/docusaurus-build.yml` (build + deploy to GitHub Pages)
- [ ] T007 Configure GitHub Actions workflow for backend in `.github/workflows/rag-api.yml` (Python tests + lint)
- [ ] T008 Create `CONTRIBUTING.md` with: chapter authoring template, commit message conventions, PR checklist (learning objectives, code examples, links)
- [ ] T009 Create `README.md` at repo root with: project overview, quick start (local setup), deployment instructions, MIT license header
- [ ] T010 Add MIT license file (`LICENSE.md`) to repository root

**Acceptance Criteria**:
- ✅ Both `website/` and `backend/` folders exist with correct structure
- ✅ `npm run build` in `website/` completes without errors
- ✅ `poetry install` (or equivalent) in `backend/` completes without errors
- ✅ GitHub Pages deployment branch is configured (typically `gh-pages`)
- ✅ README contains quick-start instructions that work locally

---

## Phase 2: Foundational Infrastructure (Week 1-2)

**Goal**: Build RAG backend, vector database, embedding pipeline. These are blocking dependencies for all user story implementations.

**Independent Test**:
- FastAPI server starts: `uvicorn main:app --reload`
- ChromaDB loads and can index/query: `python -m pytest tests/unit/test_embedding.py`
- Sample chapter chunks and embeds: `python scripts/ingest_chapters.py --test`

### 2A: Data Model & Entity Setup

- [ ] T011 Create `backend/src/models/chunk.py` with Chunk dataclass: `id`, `text`, `chapter_id`, `section`, `page`, `token_count`, `metadata`
- [ ] T012 Create `backend/src/models/query.py` with ChatbotQuery dataclass: `query_text`, `selected_text` (context), `chapter_context`, `user_id` (optional), `timestamp`
- [ ] T013 Create `backend/src/models/response.py` with ChatbotResponse dataclass: `response_text`, `source_chunk_ids`, `citations` (list of {chunk_id, chapter, page}), `confidence_score`, `timestamp`
- [ ] T014 Create `backend/src/models/validation.py` with ValidationResult dataclass: `query`, `response`, `ground_truth`, `passed`, `feedback`

**Acceptance Criteria**:
- ✅ All models have type hints and docstrings
- ✅ Models are importable: `from src.models import Chunk, ChatbotQuery, ChatbotResponse`
- ✅ Serializable to JSON (for API responses)

### 2B: Vector Database & Embeddings Setup

- [ ] T015 Install ChromaDB: `pip install chromadb sentence-transformers` in `backend/requirements.txt`
- [ ] T016 Create `backend/src/services/embeddings.py` with EmbeddingService class: `load_model()`, `embed(text)`, `batch_embed(texts)` using `sentence-transformers/all-MiniLM-L6-v2`
- [ ] T017 Create `backend/src/services/vector_db.py` with ChromaDB wrapper class: `init_collection()`, `add_chunks()`, `search(query_embedding, top_k)`, `delete_by_chapter_id()`, `get_collection_stats()`
- [ ] T018 Create `backend/src/services/chunking.py` with semantic chunking: `chunk_text(text, chunk_size=200-500 tokens)` using tiktoken for token counting
- [ ] T019 Test embeddings locally: `python -c "from src.services.embeddings import EmbeddingService; svc = EmbeddingService(); print(svc.embed('test')[:5])"` (should output embedding vector)
- [ ] T020 Initialize ChromaDB persistent storage: Create `backend/data/embeddings/` directory with `__init__.py`

**Acceptance Criteria**:
- ✅ Embedding generation works: 384-dim vectors for any text
- ✅ Batch embedding <2 seconds for 60 chunks
- ✅ ChromaDB collection initializes and persists to disk
- ✅ Query returns top-5 results with similarity scores

### 2C: FastAPI Backend & Endpoints

- [ ] T021 Create `backend/main.py` with FastAPI app: `app = FastAPI(title="Physical AI RAG", version="1.0")`
- [ ] T022 [P] Create `backend/src/api/endpoints.py` with endpoint stubs:
  - `POST /chat` → receives ChatbotQuery, returns ChatbotResponse
  - `GET /health` → returns `{"status": "ok"}`
  - `POST /reindex` → admin endpoint to trigger re-indexing
  - `POST /validate` → admin endpoint to run validation suite
- [ ] T023 Create `backend/src/api/middleware.py` with: CORS (allow localhost + GitHub Pages domain), request logging, error handler (500 → "Internal error"), rate limiting stub
- [ ] T024 Implement `POST /chat` endpoint: receives query → generates embedding → searches ChromaDB → returns top chunks + response
- [ ] T025 Add request/response validation using Pydantic (ChatbotQuery, ChatbotResponse models)
- [ ] T026 Test API locally: `curl -X POST http://localhost:8000/chat -H "Content-Type: application/json" -d '{"query_text":"What is ROS 2?"}'`

**Acceptance Criteria**:
- ✅ FastAPI server starts on `http://localhost:8000`
- ✅ `/health` endpoint returns 200 OK
- ✅ `/chat` endpoint accepts query and returns response with source chunks
- ✅ CORS headers allow requests from `localhost:3000` (Docusaurus dev server)

### 2D: CI/CD Indexing Pipeline

- [ ] T027 Create `backend/scripts/ingest_chapters.py` with functions: `read_chapters_from_markdown(docs_path)`, `chunk_chapters()`, `embed_chunks()`, `add_to_chromadb()`
- [ ] T028 Create `backend/scripts/validate_rag.py` with validation suite: Load test queries (18+ total, 3+ per chapter), run chatbot, check accuracy (response sourced from book)
- [ ] T029 Create `.github/workflows/rag-indexing.yml` GitHub Actions workflow:
  - Trigger: On push to `main` if `docs/**` files changed
  - Jobs: Detect modified chapters → Delta re-index → Run validation → Report results
  - Include concurrency group `rag-indexing` with `cancel-in-progress: false` (queue-based)
- [ ] T030 Create `.github/workflows/link-checker.yml` for external link validation (breaks build on broken links per FR-008)
- [ ] T031 Create `.github/workflows/code-validation.yml` to validate code example syntax (Python, bash, YAML per FR-007)

**Acceptance Criteria**:
- ✅ `python backend/scripts/ingest_chapters.py --test` processes sample chapter without errors
- ✅ Validation suite runs 18+ test queries and reports pass/fail
- ✅ GitHub Actions workflows trigger on code push (manual test: push dummy change)
- ✅ Link checker fails build if broken link detected (test with intentional bad link)

---

## Phase 3: US1 - Reader Explores Textbook Chapters (Week 2)

**Goal**: Create readable textbook with 6 chapters, auto-generated sidebar, fast page loads, proper navigation.

**Priority**: P1 (core feature; delivers primary value)

**Independent Test**: Open browser to `http://localhost:3000/`, navigate chapters 1-6, verify sidebar updates, load time <2 seconds, links work.

**User Story**: Reader can open browser, view chapters, navigate between them, see learning objectives and code examples.

### 3A: Docusaurus Framework & Content Structure

- [ ] T032 [P] [US1] Configure `website/docusaurus.config.js`: Set site title, baseUrl, favicon, navbar links (Home, Docs, About)
- [ ] T033 [P] [US1] Configure i18n in `docusaurus.config.js`: Add Docusaurus i18n setup (prepare for Urdu, P3)
- [ ] T034 [P] [US1] Create chapter folder structure in `website/docs/`:
  - `01-introduction/` with `index.mdx`, `_category_.json`
  - `02-humanoid-basics/`
  - `03-ros2-fundamentals/`
  - `04-digital-twin-simulation/`
  - `05-vision-language-action/`
  - `06-capstone/`
- [ ] T035 [P] [US1] Create `_category_.json` template for each chapter with metadata: `label`, `position`, `link.type` (doc)
- [ ] T036 [US1] Configure `website/sidebars.js` to auto-generate sidebar from folder structure (Docusaurus built-in)

**Acceptance Criteria**:
- ✅ All 6 chapter folders exist in `website/docs/`
- ✅ `npm run start` in `website/` launches dev server
- ✅ Sidebar shows all 6 chapters with correct order
- ✅ Clicking chapter link navigates to that chapter

### 3B: Chapter Content Authoring (6 Chapters)

*Note: These can run in parallel; authors work concurrently on different chapters.*

- [ ] T037 [P] [US1] Author Chapter 1: Introduction to Physical AI (8-12 pages)
  - Include: Learning objectives, foundational concepts, why robotics+AI matter
  - Format: MDX with code examples (Python) showing basic robotics concepts
  - File: `website/docs/01-introduction/index.mdx`
  - Test: Chapter loads, learning objectives visible, no broken links

- [ ] T038 [P] [US1] Author Chapter 2: Basics of Humanoid Robotics (8-12 pages)
  - Include: Anatomy, actuators, sensors, kinematics basics
  - Format: MDX with diagrams, code examples (YAML configurations)
  - File: `website/docs/02-humanoid-basics/index.mdx`
  - Ref prior: Chapter 1 concepts
  - Test: Loads without errors, links to Chapter 1 work

- [ ] T039 [P] [US1] Author Chapter 3: ROS 2 Fundamentals (8-12 pages)
  - Include: Nodes, topics, services, launch files, practical examples
  - Format: MDX with Python/bash code examples, ROS 2 configurations
  - File: `website/docs/03-ros2-fundamentals/index.mdx`
  - Ref prior: Chapters 1-2 concepts
  - Test: Load, verify code syntax, links to prior chapters work

- [ ] T040 [P] [US1] Author Chapter 4: Digital Twin Simulation (8-12 pages)
  - Include: Gazebo intro, Isaac Sim basics, running simulations with humanoid models
  - Format: MDX with setup instructions, simulation configs (YAML)
  - File: `website/docs/04-digital-twin-simulation/index.mdx`
  - Ref prior: Chapters 1-3
  - Test: Load, syntax checks pass, links work

- [ ] T041 [P] [US1] Author Chapter 5: Vision-Language-Action Systems (8-12 pages)
  - Include: Vision input, language models for reasoning, action execution pipelines
  - Format: MDX with Python examples, pseudocode for pipelines
  - File: `website/docs/05-vision-language-action/index.mdx`
  - Ref prior: Chapters 1-4
  - Test: Load, code examples run without errors, links work

- [ ] T042 [P] [US1] Author Chapter 6: Capstone - Simple AI-Robot Pipeline (8-12 pages)
  - Include: Hands-on project integrating chapters 1-5, deliverable code, demo
  - Format: MDX with complete Python code, step-by-step instructions, exercises
  - File: `website/docs/06-capstone/index.mdx`
  - Ref prior: All chapters 1-5
  - Test: Full chapter, code examples executable, links to all chapters work

### 3C: Content Quality & Learning Objectives

- [ ] T043 [US1] Add learning objectives section to each chapter header (Bloom's taxonomy: remember, understand, apply, analyze)
- [ ] T044 [US1] Add exercises with solutions to chapters 2-6 (Chapter 1 intro only, no exercises)
- [ ] T045 [US1] Add code example outputs/comments to all code blocks (show expected results)
- [ ] T046 [US1] Validate all external links in chapters (manual check or CI automation via T030)
- [ ] T047 [US1] Create chapter summary diagrams (SVG or PNG) and embed in each chapter
- [ ] T048 [US1] Verify all chapter content aligns with declared learning objectives (spot check per PR review)

### 3D: Frontend Components (Select-Text & UI Hooks)

- [ ] T049 [US1] Create `website/src/components/ChatbotPanel.tsx` component:
  - Displays when user selects text + clicks "Ask AI"
  - Shows selected text as context
  - Input field for user question
  - Calls `/chat` endpoint on submit
  - Displays response with citations
  - Loading state, error handling

- [ ] T050 [US1] Create `website/src/components/PersonalizeButton.tsx` stub component (P3 future; button exists, no-op for MVP)
- [ ] T051 [US1] Create `website/src/components/UrduToggle.tsx` stub component (P3 future; button exists, no-op for MVP)
- [ ] T052 [US1] Integrate ChatbotPanel into Docusaurus layout: Inject component into doc pages via `src/theme/DocItem/Layout/index.tsx`
- [ ] T053 [US1] Test ChatbotPanel interaction locally: Select text → click button → panel opens (frontend only, no backend call yet)

**Acceptance Criteria**:
- ✅ All 6 chapters exist and load without errors
- ✅ Sidebar shows all chapters in correct order
- ✅ Each chapter displays learning objectives, content, exercises (2-6)
- ✅ Code examples have syntax highlighting and expected output comments
- ✅ Navigation links between chapters work
- ✅ Page load time <2 seconds
- ✅ ChatbotPanel component displays on chapter pages (ready for backend integration in US2)

---

## Phase 4: US2 - Reader Asks Chatbot About Content (Week 2-3)

**Goal**: Integrate RAG chatbot; enable select-text → question → response workflow.

**Priority**: P1 (core feature; interactive learning)

**Independent Test**:
1. Select text from Chapter 3
2. Click "Ask AI about this"
3. Type "What is a ROS 2 node?"
4. Verify response appears within 2 seconds
5. Click citation link → jump to source passage in chapter

**Depends on**: Phase 2 (RAG backend), Phase 3 (Docusaurus chapters)

**User Story**: Reader selects text, asks chatbot question, gets answer sourced from book, can jump to source.

### 4A: Backend RAG Implementation

- [ ] T054 Implement `/chat` endpoint fully in `backend/src/api/endpoints.py`:
  - Input: ChatbotQuery (query_text, selected_text context)
  - Process: Embed query → search ChromaDB → format response with citations
  - Output: ChatbotResponse (response_text, source_chunk_ids, citations, confidence_score)
  - Error handling: If no relevant chunks, return "I don't have information about this in the textbook"

- [ ] T055 Create `backend/src/services/retrieval.py` with RAG retrieval logic:
  - `search_chunks(query_embedding, top_k=5)` → returns top chunks with similarity scores
  - `format_response(chunks, query)` → combines chunks into coherent response
  - `generate_citations(chunks)` → creates citations with chapter, section, page references

- [ ] T056 Integrate LLM for response generation (optional; can use simple chunk combination for MVP):
  - Option A: Use free ChatGPT API (if available) to synthesize response from chunks
  - Option B: Concatenate top chunks + summarize (simpler, no external API)
  - **MVP approach**: Option B (concatenate top 3 chunks + minimal summary)

- [ ] T057 Add query/response logging in `backend/src/api/middleware.py`: Log query, response, source chunks, timestamp (no PII)

### 4B: Content Indexing

- [ ] T058 Ingest all 6 chapters into ChromaDB (after chapter authoring complete):
  - Read chapters from `website/docs/`
  - Chunk each chapter semantically (200-500 tokens per chunk)
  - Embed chunks using sentence-transformers
  - Store in ChromaDB with metadata (chapter_id, section, page)
  - Expected: ~60-120 total chunks across 6 chapters

- [ ] T059 Validate chunk quality: Verify each chapter has ≥3 distinct chunks that can answer different queries

### 4C: Frontend Integration

- [ ] T060 Connect ChatbotPanel to FastAPI backend:
  - On submit: Serialize selected text + query as ChatbotQuery JSON
  - POST to `http://localhost:8000/chat` (or production API URL)
  - Receive ChatbotResponse JSON
  - Display response + citations

- [ ] T061 Implement citation click handler: When user clicks citation link, scroll to relevant chunk in chapter (or open chapter if different chapter)

- [ ] T062 Add error handling: If API unavailable, show "Feature temporarily unavailable" message (per FR-005 spec)

- [ ] T063 Add loading state: Show spinner while awaiting response; cancel button to stop request

### 4D: RAG Accuracy Validation

- [ ] T064 Create validation test set: 18+ test queries (≥3 per chapter), covering different concepts
  - Example: "What is a ROS 2 node?", "How do humanoid actuators work?", "What is semantic chunking?"
  - File: `backend/tests/fixtures/rag_test_queries.json` (format: {chapter, query, expected_chunk_ids})

- [ ] T065 Implement validation suite: `python backend/scripts/validate_rag.py --verbose`
  - Run each test query through RAG pipeline
  - Check if response sources correct chunks (no hallucinations)
  - Report pass/fail per query
  - Target: ≥90% accuracy (18+ queries, pass ≥16)

- [ ] T066 Integrate validation into CI: Add validation check to `.github/workflows/rag-indexing.yml` (must pass before merge)

**Acceptance Criteria**:
- ✅ ChatbotPanel connects to backend
- ✅ Chatbot responds within 2 seconds (p95)
- ✅ Response contains citations with chapter/section/page
- ✅ Clicking citation navigates to source
- ✅ No hallucinations: All responses sourced from book content
- ✅ Validation suite runs; ≥90% accuracy on 18+ test queries
- ✅ Works offline gracefully: Shows "Feature temporarily unavailable" if API down

---

## Phase 5: US4 - Content Author Publishes Chapter (Week 3)

**Goal**: Enable seamless chapter publishing workflow with automated validation and indexing.

**Priority**: P1 (foundational for scaling content)

**Independent Test**:
1. Author creates new chapter file `docs/03-ros-fundamentals/index.mdx`
2. Pushes branch, opens PR
3. Verify: CI checks pass (markdown syntax, broken links, code examples, learning objectives)
4. Merge to main
5. Verify: Chapter appears on live site within 5 minutes, RAG indexed within <5 minutes

**Depends on**: Phase 2 (CI/CD pipelines), Phase 3 (Docusaurus setup)

**User Story**: Author writes chapter, commits, PR is validated, merged chapter auto-deploys + indexed.

### 5A: Content Authoring Workflow

- [ ] T067 Create chapter authoring template in `CONTRIBUTING.md`:
  - Template structure: Front matter (title, learning objectives), sections, exercises, code examples, references
  - Code example format: Include comments explaining expected output
  - External link format: Use Docusaurus routing (relative paths)
  - Learning objectives format: Use Bloom's taxonomy verbs

- [ ] T068 Create PR template in `.github/pull_request_template.md`:
  - Checklist: Learning objectives declared, code examples tested, links verified, chapter aligns with prior chapters
  - Auto-filled description: Summarize chapter goal, key concepts covered

### 5B: CI/CD Validation Gates (Blocking Merge)

- [ ] T069 Implement `.github/workflows/markdown-lint.yml`: Check markdown syntax, heading hierarchy, link format
  - Tool: `markdownlint` or similar
  - Fail build on: Invalid syntax, malformed headings, broken relative links

- [ ] T070 Implement `.github/workflows/code-validate.yml`: Test code examples locally
  - Execute Python/bash/YAML in code blocks
  - Verify expected output comments match actual output
  - Fail build on: Syntax errors, runtime errors, output mismatch

- [ ] T071 Implement `.github/workflows/learning-objectives.yml`: Verify learning objectives present and aligned
  - Parse chapter front matter
  - Check: Objectives use Bloom's verbs, content covers stated objectives
  - Fail build on: Missing objectives, no alignment

- [ ] T072 Ensure all CI checks are required for merge (GitHub branch protection rule):
  - Require: markdown-lint, code-validate, learning-objectives, docusaurus-build, rag-indexing

### 5C: Chapter Deployment & Indexing

- [ ] T073 Docusaurus auto-deploy to GitHub Pages: When PR merged to main, GitHub Actions builds and deploys static site to `gh-pages` branch
  - Expected: Live within 2-3 minutes

- [ ] T074 RAG automatic re-indexing: When PR merged, trigger `rag-indexing.yml` workflow
  - Detect modified chapters via Git diff
  - Delta re-index: Only modified chapters (fast, <30 sec for 1-2 chapters)
  - Validation: Run ≥3 test queries per modified chapter
  - Expected: Complete within 2-3 minutes

- [ ] T075 Implement blue-green collection swap (zero-downtime re-indexing):
  - Create new ChromaDB collection (green)
  - Index modified chapters into green
  - Run validation on green
  - Atomically swap alias: prod → green
  - Delete old collection
  - Prevents stale chunks; allows instant rollback if validation fails

### 5D: Author Communication & Feedback

- [ ] T076 Add GitHub Actions workflow comment on PR:
  - Auto-comment when CI passes: "✅ All checks passed. Chapter will be live within 5 minutes of merge."
  - Auto-comment when validation fails: "❌ RAG validation failed on X queries. See logs: [link]. Rollback initiated."

**Acceptance Criteria**:
- ✅ PR template appears on all new PRs
- ✅ Author creates valid chapter → all CI checks pass
- ✅ Merge → chapter visible on live site within 5 minutes
- ✅ Merge → chapter indexed in RAG within 5 minutes
- ✅ Invalid chapter (broken link, code error) → CI fails, merge blocked
- ✅ Failed validation → rollback happens automatically (old collection remains prod until fixed)

---

## Phase 6: Testing & Final Deployment (Week 4)

**Goal**: Comprehensive testing, performance validation, final deployment to production.

**Independent Test**: End-to-end user flows: signup → read chapter → ask chatbot → search (optional) → success.

### 6A: Performance Benchmarking

- [ ] T077 [P] Measure page load time: `npm run build` → measure Docusaurus build time
  - Target: <2 minutes locally, <4 minutes in CI ✓ (exceeds requirement per Phase 2)

- [ ] T078 [P] Measure chatbot latency: Run 100 requests to `/chat` endpoint, measure p95 latency
  - Target: <2 seconds p95 (embedding + search + response generation) ✓ (Phase 2 estimates: 30-80ms embedding + <50ms search + ~500ms LLM = well under 2s)

- [ ] T079 [P] Measure RAG re-indexing duration: Index all 6 chapters, measure total time
  - Target: <5 minutes (Phase 2 estimates: 30-60 sec) ✓

- [ ] T080 [P] Verify storage requirements: Measure ChromaDB size for 60-120 vectors
  - Target: <1 MB (Phase 2 estimates: ~90KB vectors + metadata) ✓

### 6B: Accessibility & WCAG Compliance

- [ ] T081 Run accessibility audit: Docusaurus site + ChatbotPanel component
  - Tool: Lighthouse, axe-core, or similar
  - Check: WCAG 2.1 AA compliance (color contrast, alt text for images, keyboard navigation, screen reader compatibility)

- [ ] T082 Test keyboard navigation: Tab through all chapters, sidebar, ChatbotPanel without mouse
  - Verify: All interactive elements (links, buttons, input fields) accessible via keyboard

- [ ] T083 Add alt text to all images/diagrams in chapters (done during authoring per T043-T048, verify here)

- [ ] T084 Test with screen reader (NVDA or similar): Verify chapter content, learning objectives, chatbot interaction narrated correctly

### 6C: Content & RAG Quality Assurance

- [ ] T085 Final RAG accuracy validation: Run validation suite on all 6 chapters
  - Target: ≥90% accuracy on 18+ test queries ✓

- [ ] T086 Manual smoke test: Browse all 6 chapters, verify content completeness, links work, code examples readable

- [ ] T087 Manual chatbot test: For each chapter, ask 2-3 sample questions, verify responses make sense, citations work

- [ ] T088 External link validation: Run link checker on all chapters (automated via T030, manual verification here)

- [ ] T089 Code example execution: Run all code examples locally, verify outputs match comments (spot check 5+ examples)

### 6D: Final Deployment & Production Readiness

- [ ] T090 Set up production deployment:
  - GitHub Pages: Configure custom domain (if applicable), HTTPS enabled
  - RAG API: Deploy FastAPI backend to Vercel/Railway free tier (or keep local for MVP)
  - Environment variables: Set up secrets in GitHub Actions (.env file, API keys)

- [ ] T091 Create production checklist:
  - ✅ All chapters deployed to GitHub Pages
  - ✅ Docusaurus build successful
  - ✅ RAG API responding
  - ✅ ChatbotPanel functional
  - ✅ Accessibility audit passed
  - ✅ RAG validation ≥90%
  - ✅ Performance targets met
  - ✅ MIT license visible

- [ ] T092 Create runbook for common tasks:
  - How to re-index chapters
  - How to diagnose chatbot failures
  - How to rollback to previous collection if needed
  - How to add a new chapter

- [ ] T093 Final documentation: Update `README.md` with:
  - Live deployment URL
  - Local setup instructions (updated, tested)
  - Feature list (what works, what's future work)
  - Contributing guidelines

### 6E: Post-Launch (Future Work, P3)

- [ ] T094 User Story 3 - Search Implementation (P2, optional for MVP):
  - Add search box to Docusaurus navbar
  - Implement full-text search using Algolia or ChromaDB semantic search
  - Return relevant chapters with snippets

- [ ] T095 User Story 5 - Admin Dashboard (P2, optional for MVP):
  - Admin-only page to view RAG validation results
  - Button to trigger manual re-indexing
  - View embedding quality metrics

- [ ] T096 Optional: Urdu Translation (P3, post-MVP):
  - Use Docusaurus i18n to add Urdu translations for ≥1 chapter
  - Use sentence-transformers for Urdu queries (if available)
  - Translate navigation, sidebar, learning objectives

- [ ] T097 Optional: Personalization Engine (P3, post-MVP):
  - "Personalize" button hooks (currently stubs)
  - Collect user learning preferences (beginner/advanced, speed, style)
  - Adapt content difficulty or add tips based on preferences

- [ ] T098 Optional: Better-Auth Integration (P3, post-MVP):
  - User signup/login via Better-Auth
  - Track user reading progress
  - Personalized dashboard (visited chapters, saved notes)

**Acceptance Criteria**:
- ✅ Docusaurus build time <4 minutes (CI)
- ✅ Chatbot latency <2 seconds p95
- ✅ RAG re-indexing <5 minutes
- ✅ All images have alt text; keyboard navigation works
- ✅ WCAG 2.1 AA compliance verified
- ✅ RAG validation ≥90% on 18+ queries
- ✅ All code examples execute without errors
- ✅ Production deployment successful; live URL accessible
- ✅ README updated with live URL and setup instructions
- ✅ No broken links in any chapter
- ✅ All PRs with CI checks passing merged without errors

---

## Task Dependency Graph

```
Phase 1: Project Setup (T001-T010)
    ↓
Phase 2: Foundational Infrastructure (T011-T031)
    ├─ 2A: Data Models (T011-T014) [Independent]
    ├─ 2B: Vector DB & Embeddings (T015-T020) [Depends on 2A]
    ├─ 2C: FastAPI Backend (T021-T026) [Depends on 2B]
    └─ 2D: CI/CD Pipelines (T027-T031) [Depends on 2C]
    ↓
Phase 3: US1 - Reader Explores (T032-T053) [Parallel with Phase 2 tail]
    ├─ 3A: Docusaurus Setup (T032-T036)
    ├─ 3B: Chapter Authoring (T037-T042) [Parallel, 6 concurrent authors]
    ├─ 3C: Quality & Learning Objectives (T043-T048) [Depends on 3B]
    └─ 3D: Frontend Components (T049-T053) [Depends on 3A]
    ↓
Phase 4: US2 - Reader Asks Chatbot (T054-T066) [Depends on Phase 2 + Phase 3]
    ├─ 4A: Backend RAG (T054-T057) [Depends on Phase 2]
    ├─ 4B: Content Indexing (T058-T059) [Depends on Phase 3 + 4A]
    ├─ 4C: Frontend Integration (T060-T063) [Depends on Phase 3 + 4A]
    └─ 4D: Validation (T064-T066) [Depends on 4A + 4B]
    ↓
Phase 5: US4 - Content Author Publishes (T067-T076) [Depends on Phase 2 + Phase 3]
    ├─ 5A: Authoring Workflow (T067-T068)
    ├─ 5B: CI Validation Gates (T069-T072) [Depends on 2D]
    ├─ 5C: Deployment & Indexing (T073-T075) [Depends on 2D + 4B]
    └─ 5D: Author Communication (T076)
    ↓
Phase 6: Testing & Deployment (T077-T093) [Depends on all previous]
    ├─ 6A: Performance (T077-T080)
    ├─ 6B: Accessibility (T081-T084)
    ├─ 6C: QA (T085-T089)
    └─ 6D: Production Deployment (T090-T093)
```

---

## Parallelization Strategy

### Week 1 (Parallel Setup & Foundation)

**Stream A: Frontend Setup**
- T001-T010: Project setup
- T032-T036: Docusaurus framework

**Stream B: Backend Setup**
- T011-T031: RAG infrastructure (data models, embeddings, FastAPI, CI/CD)

**Both streams can run independently; merge at end of Week 1**

### Week 2 (Content + RAG Integration)

**Stream A: Content Authoring (6 parallel authors)**
- T037-T042: Author chapters 1-6 concurrently
- T043-T048: Quality assurance per chapter as completed

**Stream B: RAG Backend**
- T054-T057: Implement `/chat` endpoint and retrieval logic
- T058-T059: Index chapters as they complete (rolling indexing)

**Stream C: Frontend Integration**
- T049-T053: ChatbotPanel component (can start in parallel)
- T060-T063: Connect to backend (start when 4A done)

### Week 3 (Validation + Publishing Workflow)

**Stream A: CI/CD & Publishing**
- T067-T076: Authoring workflow, CI gates, deployment automation

**Stream B: Testing & Validation**
- T064-T066: RAG accuracy validation (rolling as chapters index)

### Week 4 (Final QA & Launch)

**All Streams Converge**
- T077-T093: Performance, accessibility, final deployment

---

## Testing & Validation Checklist

### Manual Test Plan

**US1: Reader Explores Chapters**
- [ ] Open `http://localhost:3000/` in browser
- [ ] Click "Chapter 1: Introduction to Physical AI"
- [ ] Verify: Title visible, learning objectives clear, content readable, load time <2 sec
- [ ] Click sidebar item for "Chapter 2"
- [ ] Verify: Navigates to Chapter 2, sidebar highlights correctly
- [ ] Click internal link to Chapter 1 from Chapter 2
- [ ] Verify: Returns to Chapter 1, link works correctly

**US2: Reader Asks Chatbot**
- [ ] Navigate to Chapter 3 (ROS 2 Fundamentals)
- [ ] Select text: "ROS 2 nodes are..."
- [ ] Click "Ask AI about this" button
- [ ] Verify: ChatbotPanel opens with selected text visible
- [ ] Type question: "What is a ROS 2 node?"
- [ ] Verify: Response appears within 2 seconds
- [ ] Verify: Response mentions nodes, services, or related concepts (sourced from book)
- [ ] Click citation link in response
- [ ] Verify: Jumps to source passage in Chapter 3 or relevant chapter

**US4: Content Author Publishes**
- [ ] Create new branch: `git checkout -b test-chapter`
- [ ] Create test chapter file: `website/docs/07-test/index.mdx` (temporary, will delete)
- [ ] Add valid content with learning objectives, code example
- [ ] Commit: `git add . && git commit -m "test: add test chapter"`
- [ ] Push: `git push origin test-chapter`
- [ ] Open PR on GitHub
- [ ] Verify: GitHub Actions workflows trigger automatically
- [ ] Verify: All CI checks run (markdown-lint, code-validate, learning-objectives, docusaurus-build, rag-indexing)
- [ ] Verify: All checks pass or fail appropriately (test with intentional error to verify CI catches it)
- [ ] Merge PR (if checks pass)
- [ ] Verify: GitHub Pages deploy within 5 minutes (check live URL)
- [ ] Verify: Chapter indexed (run validation suite; query should match test chapter)

### Automated Test Plan (CI/CD)

**Build Tests** (GitHub Actions)
- [ ] Docusaurus build completes without errors: `npm run build` in `website/` <4 min
- [ ] FastAPI tests pass: `pytest tests/` in `backend/` all green
- [ ] Markdown lint passes: No syntax errors in chapters
- [ ] Code validation passes: Python/bash/YAML examples execute correctly
- [ ] Link checker passes: No broken external links
- [ ] Learning objectives present in all chapters

**RAG Accuracy Test** (GitHub Actions)
- [ ] Run validation suite: 18+ test queries across 6 chapters
- [ ] Verify: ≥90% accuracy (≥16/18 queries pass)
- [ ] Verify: No hallucinations (all responses sourced from book)

---

## MVP Scope Summary

**What ships in Week 4:**
- ✅ 6-chapter textbook deployed to GitHub Pages
- ✅ RAG chatbot (select-text → question → answer)
- ✅ CI/CD for chapter publishing
- ✅ RAG accuracy validation (≥90%)
- ✅ WCAG accessibility compliance
- ✅ MIT license

**What's future (P2, P3):**
- ⏳ US3: Search (full-text search, semantic ranking)
- ⏳ US5: Admin Dashboard (validation metrics, re-indexing UI)
- ⏳ Urdu translation (Docusaurus i18n)
- ⏳ Personalization engine (user preferences, adaptive content)
- ⏳ Better-Auth integration (user signup, progress tracking)

**Estimated MVP Effort**: 4-6 weeks with 2-3 core developers (1 frontend, 1 backend, 1 content author)

---

## Notes & Risks

### Notes on User Input Discrepancy

The user input requested 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) with 10+ chapters. However, the current spec.md defines 6 chapters (Introduction, Humanoid Basics, ROS 2, Digital Twin, Vision-Language-Action, Capstone). This tasks.md follows the spec.md structure. **If 4 modules with 10+ chapters is the actual target, spec.md should be updated** before proceeding with implementation.

### Risk: Scope Creep

The `/sp.tasks` input requested many bonus features (Personalization, Better-Auth, Urdu, Subagents). These have been moved to Phase 6 (future work, P3) to keep MVP focused. Confirm with stakeholders that 6 chapters + RAG chatbot is acceptable for initial launch.

### Risk: RAG Accuracy

Target is ≥90% accuracy on validation set. If achieved accuracy is lower during Phase 4D, mitigation options:
- Improve chunking strategy (larger chunks to reduce context loss)
- Increase top-k results (use top 10 instead of top 5)
- Upgrade embedding model: all-mpnet-base-v2 (higher MTEB score but slower)
- Add manual validation & correction before indexing

### Risk: Timeline

4-6 week estimate assumes dedicated team and no major blockers. Risks:
- Authors slow on chapter writing → extend Phase 3
- RAG accuracy issues → extend Phase 4D
- Accessibility failures → extend Phase 6B
- Deployment issues → extend Phase 6D

Mitigation: Identify critical path (Phase 1 → Phase 2 → Phase 3 + 4 in parallel) and add buffer time.

---

## References & Appendices

### A. RAG Accuracy Validation Queries Template

```json
{
  "validation_set": [
    {
      "chapter": 1,
      "query": "What is physical AI?",
      "expected_chunks": ["ch1_intro_1", "ch1_intro_2"],
      "expected_accuracy": true
    },
    {
      "chapter": 2,
      "query": "Name three types of actuators in humanoid robots",
      "expected_chunks": ["ch2_actuators_1", "ch2_actuators_2"],
      "expected_accuracy": true
    },
    ...
  ]
}
```

### B. Source Tracking Template (Per Module)

```
docs/03-ros2-fundamentals/SOURCES.md

# Sources for Chapter 3: ROS 2 Fundamentals

1. ROS 2 Documentation (Official)
   - URL: https://docs.ros.org/en/humble/
   - Version: Humble (LTS)
   - Sections referenced: Nodes, Topics, Services, Packages

2. ROS 2 Concepts (Robotics Stack Exchange)
   - URL: https://robotics.stackexchange.com/questions/tagged/ros2
   - Sections referenced: Common patterns, best practices

3. ROS 2 Design Document
   - URL: https://design.ros2.org/
   - Sections referenced: Middleware, DDS integration

4. "Programming Robots with ROS" (O'Reilly)
   - Authors: Quigley, Gerkey, Smart
   - Sections referenced: Launch files, parameters

5. ROS 2 Tutorials (Official)
   - URL: https://docs.ros.org/en/humble/Tutorials.html
   - Sections referenced: Creating packages, writing nodes
```

### C. Learning Objectives Checklist

Bloom's Taxonomy Verbs (per Constitution):
- **Remember**: list, name, recall, identify
- **Understand**: explain, describe, interpret, classify
- **Apply**: use, demonstrate, solve, modify
- **Analyze**: distinguish, differentiate, categorize, compare
- **Synthesize**: combine, design, compile, propose
- **Evaluate**: assess, critique, judge, recommend

Example for Chapter 3:
```
## Learning Objectives

After completing this chapter, you will be able to:
1. **Understand**: Explain the concept of ROS 2 nodes and their role in modular robotics
2. **Apply**: Create a simple ROS 2 node using rospy and publish to a topic
3. **Analyze**: Distinguish between topics (pub-sub) and services (req-rep) patterns
4. **Evaluate**: Assess when to use services vs topics in a robotics system design
```

---

**Generated**: 2025-12-06 | **Status**: Ready for Implementation | **Next Step**: Begin Phase 1 (Project Setup)
