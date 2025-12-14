---
id: 3
title: RAG Chatbook Constitution v2.0
stage: constitution
date: 2025-12-14
surface: agent
model: claude-haiku-4-5-20251001
feature: rag-chatbot
branch: main
user: user
command: /sp.constitution (new project)
labels: ["constitution", "rag-chatbot", "governance", "cohere-api", "scope-shift"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```
/sp.constitution

Project: Integrated RAG Chatbot Development for a Published Book Using Cohere API, SpecifyKit Plus, and Claude

Core principles:
- Functionality and usability: Ensure the chatbot accurately retrieves and generates responses based on the book's content, including user-selected text, for an intuitive user experience.
- Integration and scalability: Seamlessly embed the chatbot for end-to-end flow, and user simulation tests.
- Documentation: Inline code comments, README with setup instructions, and API documentation via FastAPI's Swagger UI.

Constraints:
- API usage: Exclusively use Cohere's API key for RAG components; no OpenAI or other paid LLMs except Claude for development assistance.
- Budget: Stick to free tiers (Qdrant Cloud Free, Neon Serverless free plan) to avoid costs.
- Timeline: Assume a development sprint of 2-4 weeks, with milestones for setup, RAG implementation, embedding, and testing.
- Platform: Embeddable in a web-based book viewer (e.g., via iframe or JavaScript integration).
- Data limits: Handle books up to 500 pages; optimize for efficient chunking and vector storage within free tier limits.
- Accuracy and relevance: Prioritize precise retrieval-augmented generation using Cohere's API to minimize hallucinations and ensure responses are grounded in the provided context.
- Security and privacy: Protect user data and book content through secure database practices and API key management.
- Innovation with tools: Utilize SpecifyKit Plus for structured development and Claude for enhanced reasoning and code generation to create a high-quality, efficient chatbot.

Key standards:
- All responses must be based solely on the book's content or user-selected text when specified.
- Use Cohere's API for embeddings and generation, replacing any OpenAI dependencies.
- Backend built with FastAPI for API endpoints, Neon Serverless Postgres for metadata storage, and Qdrant Cloud Free Tier for vector database.
- Development process guided by SpecifyKit Plus for specification-driven design.

Success criteria:
- Chatbot accurately answers 90%+ of queries based on book content in blind tests.
- Supports user-selected text mode with zero leakage from non-selected content.
- Fully deployable and embeddable demo with no errors in production-like environment.
- Positive feedback on usability (e.g., response time < 5 seconds, coherent answers).
- Codebase passes review for cleanliness, security, and adherence to principles.
```

## Response snapshot

Created new RAG Chatbot Project Constitution v2.0.0 (MAJOR version bump from v1.1.0 of textbook). Complete scope shift from Physical AI textbook authoring to published-book RAG chatbot development.

**Major Changes:**

**Scope Shift (1.1.0 → 2.0.0):**
- OLD: Physical AI textbook (6 chapters, ~50-60 pages, Docusaurus content platform)
- NEW: Published-book RAG chatbot (up to 500 pages, Cohere API, select-text feature)

**Core Principles Replaced (8 → 8):**
1. Functionality and Usability (replaces Content-First Design; focuses on chatbot accuracy + UX)
2. Integration and Scalability (NEW; enables iframe embedding + end-to-end testing)
3. Documentation Excellence (NEW; detailed API + architecture docs)
4. API Usage & Budget Discipline (NEW; Cohere-exclusive, no OpenAI/Claude for RAG)
5. Accuracy and Relevance (NON-NEGOTIABLE; 90%+ blind test target, zero hallucination)
6. Security and Privacy (NEW; credentials in .env, HTTPS, data retention policy)
7. Innovation with Tools (NEW; SpecifyKit + Claude for structured development)
8. Technology Stack Constraints (NEW; FastAPI, Qdrant Cloud Free, Neon Serverless, Vercel)

**Sections Removed (textbook-specific):**
- 6-Chapter Structure (textbook authoring)
- Progressive Disclosure, Accessibility & i18n, Simplicity & Minimalism (textbook pedagogical principles)
- Development Workflow (textbook content review cycle)
- Documentation Quality Standards (textbook-specific)

**Sections Added (RAG-centric):**
- Data Handling & Constraints (500-page book limit, 300-500 token chunks, select-text feature, data retention)
- RAG Accuracy & Select-Text Feature (validation strategy, blind tests, failure modes)
- Technology Architecture Overview (ASCII diagram: User Client → FastAPI → Cohere + Qdrant + Neon)
- Development Workflow (4 phases: Setup & Indexing, RAG Implementation, Select-Text Feature, Validation & Deployment)

**Version**: 1.1.0 → 2.0.0
**Bump Type**: MAJOR (principle scope shift, removed textbook-specific sections, new RAG technology stack, new success criteria)
**Ratification Date**: 2025-12-14 (today)
**Governance**: Updated version bumping rules (MAJOR: Cohere API replaced, 500→1000 pages, accuracy target removed)

## Outcome

- ✅ Impact: Establishes unambiguous scope for RAG chatbot project (Cohere API exclusive, 90%+ accuracy target, select-text zero-leakage constraint). Codifies technology stack (FastAPI, Qdrant Cloud, Neon, Vercel). Enables parallel feature development (RAG core, select-text feature, validation).
- 🧪 Tests: No tests applicable (governance document)
- 📁 Files: `.specify/memory/constitution.md` (v1.1.0 → v2.0.0; 8 principles, 10 major sections; scope shift from textbook to chatbot)
- 🔁 Next prompts: `/sp.specify` to define chatbot user stories (user query, select-text, admin indexing); `/sp.plan` to architect Cohere integration + Qdrant + FastAPI backend
- 🧠 Reflection: Constitution now anchors RAG chatbot project (not textbook). Cohere API exclusivity, 90% accuracy gate, and select-text zero-leakage constraint are non-negotiable. Free-tier infrastructure (Qdrant Cloud, Neon) minimizes costs. SpecifyKit + Claude structured process ensures quality.

## Evaluation notes (flywheel)

- Failure modes observed: None (complete, coherent migration from textbook to chatbot)
- Graders run and results (PASS/FAIL): PASS (all placeholders resolved; 8 principles + 10 sections; version 2.0.0 matches header; dates ISO; MAJOR bump justified; governance updated; no unresolved constraints)
- Prompt variant (if applicable): null
- Next experiment: Run `/sp.specify` to produce RAG chatbot user stories (query chatbot, select-text query, admin book indexing, accuracy validation) and acceptance scenarios
