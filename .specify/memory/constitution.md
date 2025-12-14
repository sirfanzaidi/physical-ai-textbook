<!--
  SYNC IMPACT REPORT
  ==================
  Version: 2.0.0 (MAJOR bump)
  Bump Rationale: Complete scope shift from textbook authoring to published-book RAG chatbot. New core principles (functionality, integration, documentation). New technology stack (Cohere API, Qdrant Cloud Free, Neon Postgres, FastAPI). New success criteria (90%+ accuracy, user-selected text support). Textbook content sections removed; RAG-centric constraints added.

  Modified Principles:
    - I. Functionality and Usability (replaces Content-First Design)
    - II. Integration and Scalability (new)
    - III. Documentation (new; previously implicit)
    - IV–IX. Cohere API, Budget, Security, Accuracy, Stack, Constraints (all new)

  Removed Sections:
    - 6-Chapter Structure (textbook-specific)
    - Progressive Disclosure, Accessibility & i18n, Simplicity & Minimalism (textbook principles)
    - Development Workflow (textbook-specific)
    - Documentation Quality Standards (replaced with unified approach)

  Added Sections:
    - API Usage & Budget (Cohere-specific)
    - Data Handling (500-page books, chunking strategy)
    - RAG Accuracy & Select-Text Feature (core chatbot requirement)
    - Success Criteria (chatbot-centric metrics)

  Templates to Review:
    - spec-template.md (now for chatbot features, not textbook chapters) ⚠ pending
    - plan-template.md (tech stack updated) ⚠ pending
    - tasks-template.md (new task categories: RAG setup, embedding, select-text) ⚠ pending

  Follow-up TODOs: None (clear migration path for new RAG chatbot project)
-->

# Integrated RAG Chatbot Development — Project Constitution

## Core Principles

### I. Functionality and Usability
The chatbot MUST accurately retrieve and generate responses based solely on the book's content, including user-selected text. User experience MUST be intuitive with:
- Consistent response quality regardless of query phrasing
- Clear attribution of answers to book sections
- Support for user-selected text as chat context (zero hallucination from external content)
- Response time <5 seconds (p95) for user-facing queries
Rationale: A chatbot that hallucinates or confuses content scope breaks user trust and educational value.

### II. Integration and Scalability
The chatbot MUST embed seamlessly into the web-based book viewer (iframe or JavaScript API). Architecture MUST support:
- End-to-end flow testing (user selects text → chat opens → response generated)
- User simulation tests for load validation
- Horizontal scaling via serverless infrastructure (no persistent server requirements)
- Stateless API endpoints for easy deployment and failover
Rationale: Embeddability and automated testing enable rapid iteration and production readiness.

### III. Documentation Excellence
All code and systems MUST be thoroughly documented:
- Inline comments explaining non-obvious logic and Cohere API usage
- README with full setup instructions (API keys, environment, local dev, deployment)
- API documentation auto-generated via FastAPI Swagger UI
- Architecture diagrams for RAG pipeline and select-text feature
Rationale: Clear documentation accelerates onboarding, enables knowledge transfer, and reduces support burden.

### IV. API Usage & Budget Discipline
Cohere API is the exclusive embedding and generation provider for RAG:
- NO OpenAI, Claude API, or other paid LLMs for RAG operations
- Claude (via Claude Code) ONLY for development assistance and reasoning
- Cohere API key stored securely in `.env`; NEVER hardcoded or committed
- Budget tracking: Monitor Cohere usage monthly; optimize chunking to minimize API calls
- Free-tier alternatives for supporting infrastructure (Qdrant Cloud Free, Neon Serverless free plan)
Rationale: Predictable costs, reproducibility, vendor focus.

### V. Accuracy and Relevance (NON-NEGOTIABLE)
RAG responses MUST be grounded exclusively in the book's content:
- Chatbot MUST answer ONLY from indexed book text—no external knowledge synthesis
- Select-text mode MUST retrieve ONLY from user-highlighted passages (zero leakage from rest of book)
- Semantic chunking strategy: 300–500 token chunks with clear conceptual boundaries
- Post-indexing validation: Every chunk MUST answer ≥3 distinct student queries from its chapter
- Blind test target: ≥90% accuracy on unseen queries before production release
- Failure mode: If chatbot generates content not in book, feature MUST fail review before merge
Rationale: Accuracy is foundational to user trust and educational integrity.

### VI. Security and Privacy
User data and book content MUST be protected:
- Database credentials stored in environment variables (no hardcoded secrets)
- API keys for Cohere and cloud services managed via `.env` and CI/CD secrets
- User queries logged securely (if logged at all) with no PII exposure
- Book content restricted to authorized access only (if applicable)
- HTTPS enforced for all API endpoints
Rationale: Compliance and user confidence.

### VII. Innovation with Tools
Development MUST leverage SpecifyKit Plus and Claude for efficiency and quality:
- SpecifyKit Plus used for structured specification, planning, and task generation
- Claude (via Claude Code) for enhanced reasoning, code generation, and refactoring
- Automated PHR (Prompt History Record) creation for traceability and learning
- ADR (Architectural Decision Record) for significant design choices
Rationale: Structured processes and AI-assisted reasoning produce higher-quality outcomes faster.

### VIII. Technology Stack Constraints
Deployment MUST use the prescribed stack to ensure compatibility and minimize costs:
- **Backend**: FastAPI (Python 3.11+) for API endpoints
- **Vector DB**: Qdrant Cloud Free Tier (managed; no self-hosted overhead)
- **Metadata DB**: Neon Serverless PostgreSQL (free tier) for user sessions and chat history
- **Embeddings**: Cohere API (exclusive; no local models)
- **Generation**: Cohere API (exclusive for RAG generation)
- **Deployment**: Vercel (free tier) for API, GitHub Pages or Netlify for static book viewer
- **Testing**: pytest for unit tests, automated RAG accuracy tests
Rationale: Free tiers + managed services = zero operational overhead; Cohere focus = predictable API costs.

## Data Handling & Constraints

- **Book Scope**: Support books up to 500 pages; larger books require explicit planning
- **Chunking Strategy**: Semantic chunks 300–500 tokens; boundaries MUST NOT split learning concepts, code blocks, or logical arguments
- **Indexing Process**:
  - Extract chapters/sections from PDF or markdown
  - Split into semantic chunks with clear section markers
  - Embed using Cohere Embed API (exclusive)
  - Store in Qdrant (embeddings + metadata including page, section, chapter)
  - Validate chunks post-indexing (spot checks with sample queries)
- **Select-Text Feature**:
  - User highlights text in book viewer
  - Chat opens with highlighted text as context
  - Query answered ONLY from that passage (no full-book retrieval)
  - Implementation: Pass selected text + query to RAG pipeline with constrained retrieval
- **Data Retention**: Book content indexed indefinitely; user queries/sessions purged after 90 days (or per legal requirement)

## RAG Accuracy & Select-Text Feature

- **Retrieval Source**: Chatbot answers ONLY from indexed book content; NO external knowledge synthesis
- **Select-Text Mode**: User highlights text → opens chat → chatbot answers ONLY from that passage (zero leakage from rest of book)
- **Validation Strategy**:
  - Post-indexing: Run ≥3 queries per chapter to verify chunks are discoverable and coherent
  - Pre-launch: Blind test with 50+ unseen queries; target ≥90% accuracy (answer is correct and grounded in book)
  - Production: Log queries and manually sample monthly for quality assurance
- **Failure Mode**: If chatbot generates information not found in the book, MUST fail review before merge to production

## Technology Architecture Overview

```
User Web Client (Iframe/JS API)
    ↓
FastAPI Backend (Vercel Free Tier)
    ├─→ Cohere Embed API (for indexing)
    ├─→ Cohere Generate API (for response generation)
    ├─→ Qdrant Cloud Free Tier (vector search)
    └─→ Neon Serverless PostgreSQL (optional: session/history)
```

## Development Workflow

### Setup & Indexing (Phase 0)
- Extract book content (PDF or markdown) into sections
- Validate section quality (no OCR errors, consistent formatting)
- Configure Cohere API access and Qdrant connection
- Implement chunking pipeline with configurable token limits

### RAG Implementation (Phase 1)
- Implement embedding pipeline (Cohere Embed API)
- Implement retrieval logic (Qdrant semantic search)
- Implement generation pipeline (Cohere Generate API)
- Wire FastAPI endpoints for `/query` and `/select-text-query`

### Select-Text Feature (Phase 2)
- Extend chat UI to accept highlighted text context
- Modify retrieval logic to constrain results to selected passage
- Add tests ensuring zero leakage to non-selected content

### Validation & Deployment (Phase 3)
- Run post-indexing validation (queries per chapter)
- Execute blind accuracy test (50+ unseen queries)
- Deploy to Vercel and integrate with book viewer
- Monitor production accuracy (monthly manual sampling)

### Continuous Improvement
- Log metrics: query count, response latency, user satisfaction (if tracked)
- Monitor Cohere API usage and costs
- Quarterly accuracy reviews; re-index if results degrade

## Success Criteria

- ✅ **Accuracy**: Chatbot correctly answers ≥90% of queries based on book content in blind tests
- ✅ **Isolation**: Select-text mode retrieves ONLY from highlighted passages; zero leakage from rest of book
- ✅ **Deployment**: Fully functional demo deployable to production (Vercel + GitHub Pages) with zero errors
- ✅ **Usability**: Response time <5 seconds (p95); users report coherent, helpful answers in qualitative feedback
- ✅ **Code Quality**: Codebase is clean, well-documented, passes security review; adheres to all principles above

## Governance

- **Authority**: This constitution supersedes all design documents and informal practices for this RAG chatbot project.
- **Compliance Verification**: Every PR affecting RAG logic, API integration, or data handling MUST validate against Core Principles (mandatory checkboxes in PR template).
- **Amendments**: Changes to principles, success criteria, or technology stack MUST be proposed in separate PRs with detailed justification. Approval requires consensus from technical lead and domain expert.
- **Version Bumping**:
  - **MAJOR**: Principle removal, Cohere API replaced, success criteria removed, scope boundary change (e.g., 500→1000 pages), accuracy target removed
  - **MINOR**: New principle added, new section, material expansion of guidance (e.g., new validation strategy)
  - **PATCH**: Clarifications, wording, typos, non-breaking refinements

**Version**: 2.0.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14
