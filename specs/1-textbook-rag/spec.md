# Feature Specification: AI-Native Textbook with RAG Chatbot

**Feature Branch**: `1-textbook-rag`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: Build short, clean, professional AI-native textbook with RAG chatbot for Physical AI & Humanoid Robotics course

## Clarifications

### Session 2025-12-06

- Q1: Selected text handling → A: Selected text is used as query enrichment context (meta-data), NOT as a trigger for new chunk creation. RAG chunks remain fixed at 200–500 tokens.
- Q2: RAG API failure mode → A: When RAG API unavailable, disable select-text and search gracefully; show "Feature temporarily unavailable" message. Static textbook reading remains operational.
- Q3: Concurrent re-indexing & chatbot queries → A: During re-indexing, block incoming chatbot queries with "Chatbot maintenance in progress" message. Re-indexing is <5 min; no stale chunks served.
- Q4: CI merge gate strictness → A: All checks block merge: broken links, markdown syntax, code execution, and learning objectives. Ensures MVP quality gate.
- Q5: Chapter structure granularity → A: 6 chapters + named sections (internal subsections within each chapter file, not separate topic folders). Maintains chapter cohesion.

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Reader Explores Textbook Chapters (Priority: P1)

A student opens the textbook in a browser and navigates through chapters to learn foundational concepts in Physical AI and Humanoid Robotics. The reader expects a clean, distraction-free interface with fast page loads and reliable navigation.

**Why this priority**: Core feature that delivers primary value—students reading structured, high-quality educational content. Without this, no other features matter.

**Independent Test**: Can be fully tested by opening browser, navigating chapters 1–6, and verifying content loads, sidebar works, and links to prior chapters function. Delivers complete learning experience for a single chapter.

**Acceptance Scenarios**:

1. **Given** a reader visits the textbook homepage, **When** they click "Chapter 1: Introduction to Physical AI", **Then** the chapter loads in under 2 seconds and displays learning objectives, content, and exercises.
2. **Given** a reader is on Chapter 2, **When** they click a link to Chapter 1, **Then** they navigate to Chapter 1 and the navigation sidebar highlights the correct chapter.
3. **Given** a reader is on any chapter, **When** they view the sidebar, **Then** they see all 6 chapter titles with auto-generated hierarchy (no manual edits).
4. **Given** a reader scrolls through a chapter with code examples, **When** they view an example, **Then** the code is syntax-highlighted and includes expected output comments.

---

### User Story 2 - Reader Asks Chatbot About Content (Priority: P1)

A student reads a specific passage or concept, selects text on the page, and asks the RAG chatbot to explain or clarify it. The chatbot responds with information strictly from the book, no hallucinations or external knowledge. **Selected text is passed as query enrichment context, not as a new indexed chunk—RAG retrieval always uses the fixed 200–500 token chunks.**

**Why this priority**: Enables interactive learning and immediate feedback; core differentiator of AI-native textbook. Requires tight integration with content indexing and retrieval.

**Independent Test**: Can be fully tested by selecting text from a chapter, triggering chatbot, asking a question, and verifying response derives only from book content. Delivers immediate value without requiring other user stories.

**Acceptance Scenarios**:

1. **Given** a reader selects text from Chapter 3 (ROS 2 Fundamentals), **When** they click "Ask AI about this", **Then** a chat panel opens with the selected text as context.
2. **Given** a chat panel is open with context from Chapter 3, **When** the reader types "What is a ROS 2 node?", **Then** the chatbot responds within 2 seconds with an explanation sourced only from book content (no external synthesis).
3. **Given** the chatbot answers a question, **When** the reader reads the response, **Then** they can click a citation link to jump to the relevant book passage.
4. **Given** a reader asks a question that cannot be answered from book content, **When** the chatbot processes the query, **Then** it responds "I don't have information about this in the textbook" rather than hallucinating.

---

### User Story 3 - Reader Discovers Content via Search (Priority: P2)

A student uses a search box to find specific topics, concepts, or keywords across all chapters. Search results show relevance ranking and link directly to relevant sections.

**Why this priority**: Improves discoverability and student agency; reduces friction when students need specific information. Depends on successful RAG infrastructure (US2) but can be independently tested once RAG backend is ready.

**Independent Test**: Can be fully tested by entering search queries (e.g., "actuators", "ROS topics"), verifying results include relevant chapters, and clicking results to verify navigation works.

**Acceptance Scenarios**:

1. **Given** a reader enters "humanoid kinematics" in the search box, **When** they press Enter, **Then** within 2 seconds they see up to 5 relevant passages from Chapter 2 ranked by relevance.
2. **Given** search results are displayed, **When** the reader clicks on a result, **Then** they navigate to the correct chapter and scroll to the relevant section.
3. **Given** a reader enters a query with no matching content, **When** search executes, **Then** the interface displays "No results found" gracefully.

---

### User Story 4 - Content Author Publishes Chapter (Priority: P1)

An author writes a chapter in Markdown, commits it to Git, and submits a PR. After approval, the chapter is automatically indexed by the RAG pipeline and deployed to the textbook website.

**Why this priority**: Enables content delivery and RAG integration. Without this workflow, new chapters cannot be published or indexed. Foundation for scaling content creation.

**Independent Test**: Can be fully tested by creating a chapter file, committing, opening a PR, and verifying PR status checks pass (link validation, code syntax, RAG re-indexing).

**Acceptance Scenarios**:

1. **Given** an author creates a new chapter file in `docs/03-ros-fundamentals/index.md`, **When** they push the branch and open a PR, **Then** CI automatically validates markdown syntax, checks for broken links, and verifies learning objectives are declared.
2. **Given** a PR is opened with a valid chapter, **When** the PR is merged to master, **Then** within 5 minutes the chapter appears on the live textbook website.
3. **Given** a chapter is merged, **When** the RAG indexing pipeline runs, **Then** the chapter's content is chunked semantically and indexed in Qdrant, ready for chatbot queries.
4. **Given** an author makes a significant chapter rewrite (>50% content change), **When** the PR is merged, **Then** the RAG pipeline purges old chunks and re-indexes the entire chapter.

---

### User Story 5 - Admin Monitors RAG Accuracy (Priority: P2)

An admin or content lead runs validation tests to ensure the RAG chatbot only answers from book content and doesn't hallucinate. They can re-trigger indexing, view embedding quality metrics, and address failures.

**Why this priority**: Ensures core constraint (accuracy-first) is maintained. Depends on successful RAG deployment (US2, US4) but can be tested independently via admin dashboard/CLI.

**Independent Test**: Can be fully tested by running validation suite (≥3 queries per chapter), reviewing results, and verifying all responses derive only from book text.

**Acceptance Scenarios**:

1. **Given** an admin accesses the RAG dashboard, **When** they click "Run Validation Suite", **Then** within 2 minutes the system tests ≥3 distinct queries per chapter and reports pass/fail status.
2. **Given** validation results show a failure (chatbot hallucinated), **When** the admin investigates, **Then** they can view the problematic query, response, and source chunks to diagnose the issue.
3. **Given** an embedding model is updated, **When** the admin triggers "Re-index All Chapters", **Then** within 10 minutes all 6 chapters are re-indexed and validation suite passes.

---

### User Story 6 - Reader Uses Urdu Interface (Optional; Priority: P3)

A student reading in Urdu can switch the textbook language to Urdu and read translated chapters. The chatbot responds in Urdu when queried in Urdu.

**Why this priority**: Optional feature; expands accessibility. Does NOT block textbook launch. Can be added post-MVP if resources available.

**Independent Test**: Can be fully tested by switching language to Urdu, verifying at least one complete chapter is translated, and testing chatbot responses in Urdu.

**Acceptance Scenarios**:

1. **Given** a reader visits the textbook, **When** they click the language selector and choose Urdu, **Then** the sidebar, navigation, and at least one complete chapter (e.g., Chapter 1) display in Urdu.
2. **Given** a reader is viewing Urdu content, **When** they select text and ask the chatbot a question in Urdu, **Then** the chatbot responds in Urdu within 2 seconds.

---

### Edge Cases

- What happens when a reader's network is slow and chapter loads take >5 seconds? (Graceful spinner; no error message)
- How does the system handle code examples that reference external tools (ROS 2, Gazebo) not installed locally? (Examples include clear prerequisites and links to installation guides)
- What if a student asks the chatbot something completely unrelated to the textbook (e.g., "What is the capital of France")? (Chatbot responds: "This question is outside the scope of the textbook. I can only help with content from the Physical AI & Humanoid Robotics chapters.")
- What if the RAG indexing fails for a chapter after merge? (CI build status shows RED; PR author and maintainers are notified via GitHub; chapter is NOT deployed until issue is resolved)
- What if two authors edit the same chapter simultaneously and merge conflicting PRs? (Git conflict resolution required; second merge fails CI with merge conflict message)
- What if the RAG API (Qdrant or embedding service) becomes unavailable? (Select-text and search features are disabled gracefully with "Feature temporarily unavailable" message; textbook reading remains fully operational)
- What if a student tries to use the chatbot while re-indexing is in progress? (System shows "Chatbot maintenance in progress, back online in ~5 minutes"; query is not processed; no stale chunks served)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST serve a Docusaurus-based textbook with 6 chapters covering: Introduction to Physical AI, Basics of Humanoid Robotics, ROS 2 Fundamentals, Digital Twin Simulation, Vision-Language-Action Systems, and Capstone. Each chapter MAY contain named internal sections/subsections (e.g., Chapter 3 → "Nodes", "Topics", "Services") rendered as headings within the chapter file.
- **FR-002**: System MUST render all content as MDX (Markdown + JSX) and display chapters with auto-generated sidebar navigation (no manual sidebar edits). Internal sections appear as sub-headings in the sidebar hierarchy.
- **FR-003**: System MUST include a select-text → ask-chatbot feature allowing readers to highlight any passage and query the RAG chatbot for clarification.
- **FR-004**: System MUST deploy the textbook as static HTML to GitHub Pages with build completion in <2 minutes.
- **FR-005**: System MUST integrate a RAG chatbot (Qdrant vector DB + free embedding model) that answers ONLY from book content; chatbot MUST NOT synthesize or hallucinate external knowledge. **When RAG API is unavailable, select-text and search features MUST be disabled gracefully with "Feature temporarily unavailable" message; static textbook reading remains operational.**
- **FR-005a**: **Selected text passed to chatbot is query enrichment context (meta-data), not a new indexed chunk. RAG retrieval always uses fixed 200–500 token chunks.**
- **FR-006**: System MUST chunk book content semantically (200–500 tokens per chunk) and re-index after chapter merges via CI pipeline. **During re-indexing, incoming chatbot queries MUST be blocked with "Chatbot maintenance in progress" message. Re-indexing window is <5 minutes; no stale chunks served.**
- **FR-007**: System MUST validate code examples locally before commit; all examples MUST execute without errors and include expected output comments. **Code execution validation, markdown syntax checks, broken link validation, AND learning objectives verification all MUST BLOCK PR merge (required CI checks).**
- **FR-008**: System MUST fail CI builds on broken external links; links MUST be verified and updated before deployment.
- **FR-009**: System MUST support optional Urdu translation using Docusaurus i18n workflow; translations MUST be complete for at least one chapter before release.
- **FR-010**: System MUST include a learning objectives declaration in every chapter; content MUST align with declared objectives per PR review.
- **FR-011**: System MUST provide an admin dashboard or CLI to trigger RAG re-indexing, run validation tests (≥3 queries per chapter), and view embedding quality metrics.
- **FR-012**: System MUST log RAG chatbot queries and responses for monitoring accuracy and debugging; logs MUST NOT contain sensitive student data.

### Key Entities

- **Chapter**: A unit of learning content (Markdown file) with learning objectives, 2–4 code examples, exercises, ~8–12 pages. Attributes: title, number (1–6), learning objectives, content body, associated code files, links to prior chapters.
- **Content Chunk**: A semantic segment of a chapter (200–500 tokens) indexed in Qdrant for RAG retrieval. Attributes: chunk ID, text, chapter reference, embedding vector, source line numbers.
- **Chatbot Query**: A student question about book content, potentially with selected text context. Attributes: query text, selected text context (enrichment meta-data, NOT indexed), chapter context (if provided), student ID (optional), timestamp, retrieved chunks (200–500 token chunks, fixed indexing).
- **Chatbot Response**: System answer sourced from book content. Attributes: response text, source chunk IDs, citations with links to book passages, confidence score, timestamp.
- **Build Artifact**: Output from Docusaurus build (static HTML + JS). Attributes: build timestamp, Git commit SHA, status (success/fail), error logs (if failed).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Textbook deploys successfully to GitHub Pages with all 6 chapters publicly accessible; readers can navigate between chapters and view content within 2 seconds per page load.
- **SC-002**: RAG chatbot answers ≥95% of test queries (≥3 per chapter, 18+ total queries) correctly using only book content; <5% of responses hallucinate or cite sources outside the book.
- **SC-003**: Docusaurus build completes in <2 minutes locally and <4 minutes in CI; no build failures due to markdown, link, or syntax errors.
- **SC-004**: All code examples in chapters execute without errors locally; authors provide proof of testing (script output) before PR merge.
- **SC-005**: Readers can select text and open chatbot within 1 click; chatbot response appears within 2 seconds (p95 latency).
- **SC-006**: Search functionality returns relevant results within 2 seconds (p95) for any chapter-related query; <10% false negatives (relevant passages not returned).
- **SC-007**: If Urdu translation is added, at least 1 complete chapter is available in Urdu before MVP release; chatbot responds in Urdu when prompted in Urdu.
- **SC-008**: Content authors can commit, PR, and deploy a new chapter in <1 hour (excluding review time) following the documented authoring workflow.
- **SC-009**: RAG validation suite (≥3 queries per chapter) runs in <5 minutes and provides clear pass/fail report; failures are traceable to specific chunks and can be debugged.
- **SC-010**: Zero unintentional downtime for the textbook during steady state (build, deploy, and chatbot API running continuously without manual intervention).

## Assumptions

- **Technical Stack**: Docusaurus 3.x, FastAPI or Node.js for RAG backend, Qdrant for vector DB, free embedding model (e.g., text-embedding-3-small), Neon PostgreSQL optional for metadata, GitHub Pages for static hosting, Vercel optional for RAG API.
- **Chapter Content Baseline**: Authors will provide ~8–12 pages per chapter (estimated 4,000–6,000 words each); 6 chapters total estimated at 50–60 pages.
- **Code Examples**: All examples are Python, bash, or YAML; no GPU-heavy code; examples run locally in <5 seconds.
- **User Audience**: Students with basic familiarity with robotics and AI concepts; no prior ROS 2 or Gazebo experience assumed.
- **Deployment Model**: Textbook is public (no authentication); RAG chatbot API is rate-limited but publicly accessible; optional admin dashboard requires GitHub login.
- **Data Retention**: Chatbot query logs retained for 90 days (debugging, accuracy monitoring); no personal student data collected unless explicitly opted in.
- **RAG Accuracy Constraint**: Chatbot MUST answer ONLY from book text; if uncertain, MUST say "I don't know" rather than guess or synthesize external knowledge.

## Out of Scope

- Mobile app (web-only initially)
- Personalized learning paths / adaptive content (listed as "optional" but not in MVP)
- Video content or interactive simulations (text + code examples only)
- Real-time collaboration or live chat with instructors (asynchronous chatbot only)
- Integration with learning management systems (LMS) or grade tracking
- Advanced analytics on student reading patterns (basic logs only)

## Dependencies & Integration Points

- **Git + GitHub**: Source of truth for chapter files and PR workflow
- **GitHub Actions**: CI pipeline for validation, build, and deployment
- **Docusaurus 3.x**: Framework for textbook rendering
- **Qdrant Vector DB**: Stores embeddings; retrieval backend for RAG
- **Free Embedding API**: (e.g., OpenAI text-embedding-3-small, Ollama) provides vectors for chunks
- **GitHub Pages**: Hosts static textbook website
- **Vercel (optional)**: Hosts RAG API backend
- **ROS 2, Gazebo, Isaac Sim**: External tools referenced in content; NOT hosted by textbook platform (authors link to official docs)
