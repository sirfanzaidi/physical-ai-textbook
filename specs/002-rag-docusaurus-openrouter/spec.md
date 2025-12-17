# Feature Specification: RAG Chatbot Integration with Docusaurus & OpenRouter

**Feature Branch**: `002-rag-docusaurus-openrouter`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Integrate a Retrieval-Augmented Generation (RAG) chatbot into the existing Docusaurus book using OpenRouter API for embeddings and chat completions"

## User Scenarios & Testing

### User Story 1 - Ask Questions About Book Content Anywhere (Priority: P1)

End users reading the Physical AI & Humanoid Robotics textbook on the Docusaurus site can click a floating chatbot button and ask questions about the book content from any page.

**Why this priority**: This is the core value proposition. Without this, users have no way to interact with the chatbot. It's the foundational feature for all other functionality.

**Independent Test**: Can be fully tested by:
1. Visiting any Docusaurus page
2. Locating and clicking the floating chatbot widget
3. Typing a question about visible content
4. Receiving an answer with source citations
5. Verifying response is grounded in book content

**Acceptance Scenarios**:

1. **Given** user is on any Docusaurus page, **When** user clicks the floating chatbot button, **Then** chat widget opens smoothly without page navigation or disruption
2. **Given** chat widget is open, **When** user types a question about the book content, **Then** chatbot responds within 5 seconds with an answer grounded in the book
3. **Given** chatbot responds, **When** response is generated, **Then** response includes source citations showing which chapter/section/page the information came from
4. **Given** user asks a question about information not in the book, **When** chatbot processes the query, **Then** chatbot responds "I don't have that information in the textbook" rather than using external knowledge

---

### User Story 2 - Ask About Selected Text (Priority: P1)

End users can highlight any text in the Docusaurus book and click an "Ask about this" button to ask targeted questions about just that passage.

**Why this priority**: Select-text mode is a core differentiator and provides precise context-aware querying. High priority because it enables the zero-leakage RAG feature mentioned in requirements.

**Independent Test**: Can be fully tested by:
1. Reading a section of the textbook
2. Highlighting a specific text passage
3. Clicking "Ask about this" button
4. Submitting a query
5. Verifying the response only uses information from the highlighted passage

**Acceptance Scenarios**:

1. **Given** user has highlighted text on a Docusaurus page, **When** user clicks "Ask about this", **Then** chat widget opens with selected text pre-populated as context
2. **Given** selected text is provided, **When** user submits a question, **Then** chatbot generates response ONLY from the selected passage (no information from rest of book)
3. **Given** selected passage doesn't contain the answer, **When** user queries, **Then** chatbot responds "This information is not in the selected passage" rather than searching the full book
4. **Given** selected text is too short (less than 10 characters), **When** user tries to query, **Then** frontend warns user to select more text, or backend rejects with clear error message

---

### User Story 3 - Ingest Book Content from Docusaurus (Priority: P1)

An administrator runs an ingestion script that crawls the built Docusaurus site, extracts all markdown content, chunks it semantically, generates embeddings via OpenRouter, and stores everything in Qdrant Cloud.

**Why this priority**: Without indexing, the chatbot has no content to search. This is the first setup task and blocks all other features.

**Independent Test**: Can be fully tested by:
1. Running ingestion script pointing to Docusaurus build output
2. Script extracts all markdown files and converts to text
3. Text is chunked semantically (800–1200 characters with overlap)
4. Embeddings are generated via OpenRouter Qwen embedding model
5. Vectors and metadata are upserted to Qdrant Cloud
6. Verify all chapters appear as collections or are properly indexed
7. Verify no data loss during ingestion

**Acceptance Scenarios**:

1. **Given** Docusaurus build is complete, **When** ingestion script runs, **Then** all chapters/docs are extracted and converted to text chunks
2. **Given** markdown is extracted, **When** chunks are generated, **Then** chunks are 800–1200 characters, preserve sentence boundaries, and respect code block/table atomicity (no splitting mid-block)
3. **Given** chunks are ready, **When** embeddings are generated, **Then** OpenRouter Qwen embedding model is used to create 1024-dimensional vectors (or appropriate dimension for chosen model)
4. **Given** embeddings are generated, **When** data is upserted to Qdrant, **Then** vectors, metadata (chapter, section, page, URL), and source links are stored with zero data loss
5. **Given** ingestion completes, **When** metadata is stored, **Then** Qdrant collection contains all book content and is ready for queries

---

### User Story 4 - Chat Streams Real-Time Responses (Priority: P2)

User sees chatbot responses streaming in real-time, with text appearing character-by-character for better perceived performance and engagement.

**Why this priority**: Improves user experience but is not required for MVP. Can be implemented after basic Q&A works.

**Independent Test**: Can be fully tested by:
1. Submitting a query to chatbot
2. Observing response text appearing gradually
3. Verifying streaming doesn't skip content or cut off mid-word
4. Testing on slow network conditions

**Acceptance Scenarios**:

1. **Given** user submits a query, **When** chatbot generates response, **Then** response text appears character-by-character on screen (not all at once)
2. **Given** response is streaming, **When** network is slow, **Then** text continues to stream reliably without timeouts or data loss
3. **Given** streaming is complete, **When** citations are available, **Then** citations appear after response text

---

### User Story 5 - Persist Chat History (Priority: P2)

Users can see their previous messages and continue conversations across sessions (stored in browser localStorage or optional Neon database).

**Why this priority**: Nice-to-have feature for better UX but not critical for MVP. Can be implemented after core Q&A.

**Independent Test**: Can be fully tested by:
1. Submitting multiple questions in one session
2. Closing chat widget
3. Reopening chat widget
4. Verifying previous messages appear
5. Continuing conversation

**Acceptance Scenarios**:

1. **Given** user submits questions in chat, **When** chat session is active, **Then** all messages are visible in chat history
2. **Given** user closes chat widget, **When** user reopens widget later, **Then** previous messages reappear (from browser storage or database)
3. **Given** user clicks "Clear History", **When** action is confirmed, **Then** all previous messages are deleted

---

### Edge Cases

- **Slow Ingestion**: What if the Docusaurus site has hundreds of pages and ingestion takes hours?
  - **Requirement**: Ingestion script should provide progress logging, estimate time remaining, and support resuming from checkpoints if interrupted.

- **Embedding Model Unavailability**: What if OpenRouter API is temporarily down during ingestion or queries?
  - **Requirement**: System should retry with exponential backoff, log detailed errors, and provide user-friendly error messages ("Chatbot service temporarily unavailable").

- **Large Book Content**: What if ingestion produces millions of chunks exceeding Qdrant Free Tier limits?
  - **Requirement**: System should warn before ingestion, provide guidance on data management, or support pagination/archival strategies.

- **Out-of-Sync Content**: What if Docusaurus site is updated but old vectors remain in Qdrant?
  - **Requirement**: Admin should have ability to purge and re-index specific chapters or entire book without manual cleanup.

- **Cross-Browser Text Selection**: How does select-text feature handle different browsers' text selection APIs?
  - **Requirement**: JavaScript widget must normalize text selection across browsers (Chrome, Firefox, Safari, Edge) using standard DOM APIs.

- **Mobile Text Selection**: How does select-text work on mobile devices?
  - **Requirement**: Mobile-friendly text selection (touch-based) must be supported; fallback to full-book mode if selection fails.

## Requirements

### Functional Requirements

- **FR-001**: System MUST ingest all markdown content from Docusaurus build output (docs/, blog/, or static files) and extract text, removing HTML and frontmatter
- **FR-002**: System MUST chunk ingested text into semantic segments of 800–1200 characters with 150–200 character overlap, respecting sentence boundaries and atomic units (code blocks, tables, equations)
- **FR-003**: System MUST generate embeddings using OpenRouter API (Qwen embedding model: prefer qwen/qwen3-embedding-4b or qwen/qwen3-embedding-0.6b for efficiency)
- **FR-004**: System MUST store embeddings and metadata (chapter, section, URL, page) in Qdrant Cloud Free Tier with proper indexing for semantic search
- **FR-005**: System MUST provide a `/api/chat-stream` endpoint accepting `{query, book_id, mode, selected_text}` and returning streaming JSON-lines response
- **FR-006**: System MUST generate responses using OpenRouter API with a strong LLM (recommended: Qwen3 or Claude model, configurable via env var)
- **FR-007**: System MUST answer queries ONLY from indexed Docusaurus content; no external knowledge synthesis or hallucinations
- **FR-008**: System MUST support select-text mode: accept highlighted text as context and constrain retrieval to overlapping chunks only (zero leakage)
- **FR-008a** (Select-Text Validation): System MUST enforce minimum **10 characters** for selected text. Backend MUST reject (HTTP 400) selected_text <10 characters with error "Selected text must be at least 10 characters"
- **FR-009**: System MUST provide floating widget (React or Vanilla JS) embedded in Docusaurus pages with smooth UI integration and no console errors
- **FR-010**: Floating widget MUST NOT break Docusaurus navigation, styling, or performance; CORS headers MUST be properly configured
- **FR-011**: System MUST respond to queries within 5 seconds (p95 latency)
- **FR-012**: System MUST store API keys (OpenRouter, Qdrant) in environment variables (`.env`); never hardcode secrets
- **FR-013**: System MUST provide comprehensive error handling with user-friendly messages (e.g., "Chatbot service unavailable; please try again")
- **FR-014**: System MUST log all queries and responses for debugging and quality assurance (without exposing PII or sensitive book excerpts)
- **FR-015**: System MUST enforce HTTPS for all API endpoints in production
- **FR-016**: System MUST support streaming responses (Server-Sent Events or JSON-lines format) for real-time text display
- **FR-017**: System MUST optionally persist chat history to Neon Serverless Postgres or browser localStorage for session continuity
- **FR-018**: Chatbot accuracy MUST be ≥90% on blind test queries (50+ unseen questions validated before production)

### Key Entities

- **Chapter/Doc**: Represents a Docusaurus page or chapter. Attributes: title, slug, content (raw markdown), chapter_number, section_path, source_url
- **Chunk**: Represents a semantic segment. Attributes: chunk_id, chapter_id, content (text), character_count, token_count, embedding_vector, source_url, chapter_name, section_name
- **Query**: Represents a user question. Attributes: query_id, query_text, book_id, mode (full/selected), selected_text (if applicable), timestamp, response_text, citations, latency_ms
- **Session**: (Optional) Represents a user's chat history. Attributes: session_id, queries (list), created_at, updated_at

## Success Criteria

### Measurable Outcomes

- **SC-001**: Chatbot accurately answers ≥90% of blind test queries (50+ unseen questions covering all chapters)
- **SC-002**: Select-text mode retrieves ONLY from highlighted passages with zero leakage (validated by queries outside selected text returning "not in selection" messages)
- **SC-003**: API response time (p95) is <5 seconds for typical queries (50–200 character questions)
- **SC-004**: Floating widget loads and initializes without JavaScript errors on all Docusaurus pages
- **SC-005**: Chatbot citations correctly link to source chapters/sections in Docusaurus (100% of citations are valid, clickable links)
- **SC-006**: Ingestion script successfully processes entire Docusaurus site (all chapters/docs) with zero data loss or corruption
- **SC-007**: System supports at least 10 concurrent users querying chatbot simultaneously without performance degradation
- **SC-008**: Code passes security review: no hardcoded credentials, HTTPS enforced, CORS properly configured, API keys in `.env`
- **SC-009**: ≥80% of qualitative user feedback indicates chatbot responses are helpful and accurate (collected via in-app survey or user testing)
- **SC-010**: Ingestion script completes for full Docusaurus site in <30 minutes with detailed progress logging

## Assumptions

- **Docusaurus Build Output**: Markdown files are available in a standard Docusaurus build directory structure (e.g., `docs/`, `blog/`)
- **OpenRouter API Availability**: OpenRouter is stable and provides reliable embeddings and LLM API endpoints during development and production
- **Qdrant Cloud Stability**: Qdrant Cloud Free Tier is sufficient for MVP (estimated <100K vectors based on book size); no self-hosted Qdrant initially
- **Embedding Dimension**: OpenRouter Qwen embedding model produces vectors compatible with Qdrant (typical: 1024-dimensional or as specified in API response)
- **User Context**: End users can formulate coherent questions about book content and understand chatbot limitations (book-only knowledge)
- **Select-Text Implementation**: Browser supports standard text selection APIs (window.getSelection, document.createRange); widget JavaScript normalizes across browsers
- **No Real-Time Updates**: Book content is static during a session. Updates require manual re-indexing; live sync is out of scope
- **Optional Postgres**: Neon Postgres is optional for MVP; chat history can be stored in browser localStorage first

## Out of Scope

- **Multi-Language Support**: Initial version targets English; localization deferred
- **Advanced Analytics/Dashboard**: Beyond basic query logging and accuracy metrics
- **Personalization**: User profiles, saved preferences, or ML-based recommendations
- **Enterprise Scaling**: Initial deployment for single/small-group usage (not millions of users)
- **Multiple LLM Backends**: MVP uses single OpenRouter LLM; switching backends not supported in first release
- **Voice Input/Output**: Audio chatbot features deferred
- **Custom Styling**: Chatbot uses default CSS; extensive theming deferred
- **Integration with Third-Party Books**: Single book (Physical AI textbook) focused; multi-book support deferred
