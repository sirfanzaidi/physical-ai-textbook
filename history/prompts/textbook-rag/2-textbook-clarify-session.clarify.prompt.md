---
id: 2
title: Textbook Clarify Session 5 QA
stage: clarify
date: 2025-12-06
surface: agent
model: claude-haiku-4-5-20251001
feature: textbook-generation
branch: 1-textbook-rag
user: user
command: /sp.clarify
labels: ["clarify", "textbook-generation", "rag-chatbot", "requirements-validation"]
links:
  spec: specs/1-textbook-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-textbook-rag/spec.md (updated with Clarifications section)
tests:
  - none
---

## Prompt

```
"Identify all unclear points in the requirements related to the 13 textbook topics,
the chatbot features, and system integration. Resolve all ambiguities."
```

## Response snapshot

Conducted structured ambiguity scan of AI-Native Textbook specification and resolved 5 critical clarification questions through interactive questioning:

**Q1: Semantic Chunking Granularity & Select-Text Integration**
- Answer: D - Query Enrichment Only
- Clarification: Selected text is passed as query enrichment context (meta-data), NOT as a trigger for new chunk creation. RAG chunks remain fixed at 200–500 tokens. Avoids ad-hoc chunking complexity; keeps RAG strategy stable.

**Q2: RAG API Failure & Graceful Degradation**
- Answer: C - Disable Gracefully
- Clarification: When RAG API unavailable, disable select-text and search gracefully with "Feature temporarily unavailable" message. Static textbook reading remains operational. Aligns with accuracy-first principle (don't serve uncertain data).

**Q3: Concurrent Indexing & Chatbot Query Handling**
- Answer: A - Block Queries
- Clarification: During re-indexing, block incoming chatbot queries with "Chatbot maintenance in progress" message. Re-indexing <5 min. Prevents stale chunks; simplest to implement and debug.

**Q4: CI Merge Gate Requirements**
- Answer: B - All Required
- Clarification: All CI checks block merge: broken links, markdown syntax, code execution, AND learning objectives. Ensures MVP quality gate; faster to fix pre-deploy.

**Q5: Chapter Structure & Topic Granularity**
- Answer: B - 6 Chapters + Named Sections
- Clarification: 6 chapters with named internal sections (e.g., Chapter 3 → "Nodes", "Topics", "Services" as sub-headings). Not 13 separate topic folders; maintains chapter cohesion. Resolved ambiguity about "13 topics" mentioned in user request.

**Spec Updates Applied**:
- Added Clarifications section (Session 2025-12-06)
- Updated FR-001/FR-002 for chapter+sections structure
- Updated FR-005 for RAG failure handling and query enrichment clarification
- Added FR-005a for selected text as enrichment context
- Updated FR-006 for re-indexing blocking behavior
- Updated FR-007 for CI merge gate strictness
- Updated Chatbot Query entity to clarify selected text as enrichment meta-data
- Added 2 new edge cases: RAG API unavailability, chatbot query during re-indexing

**Total Ambiguities Resolved**: 5 critical questions (of 5 allowed)
**Coverage After Clarification**: All high-impact ambiguities now Clear

## Outcome

- ✅ Impact: Specification is now unambiguous and implementation-ready. All clarifications directly impact architecture (RAG pipeline, CI gates, failure handling), data modeling (chunk strategy, query context), and task decomposition (re-indexing blocking, graceful degradation).
- 🧪 Tests: No formal tests; clarifications validated via structured taxonomy scan and interactive Q&A
- 📁 Files:
  - `specs/1-textbook-rag/spec.md` (updated with Clarifications section + 5 integrated answers)
  - No new files created
- 🔁 Next prompts: `/sp.plan` to create architecture design, data models, RAG pipeline, CI/CD strategy, and task dependency map
- 🧠 Reflection: Clarification session successfully identified and resolved all critical ambiguities by mapping user intent ("13 topics") to actual structure (6 chapters + sections). Questions were prioritized by impact and interdependencies (Q1 affects data model, Q2/Q3 affect system design, Q4 affects CI/CD, Q5 affects structure). All answers are recorded in spec for reference during planning and implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None; all questions were answerable within 5-question quota
- Graders run and results (PASS/FAIL): PASS
  - Ambiguity scan identified 5 Partial/Missing categories (chunk strategy, RAG failure, concurrent ops, CI gates, chapter structure)
  - All 5 clarifications directly address high-impact areas (>1 section/requirement updated per clarification)
  - Spec now has zero unresolved ambiguities in Partial/Missing categories
  - Terminology consistency validated (e.g., "chunk" consistently 200–500 tokens; "selected text" consistently enrichment context)
- Prompt variant (if applicable): null (user request resolved on first pass)
- Next experiment: `/sp.plan` to design system architecture given these clarified requirements

---

## Summary of Clarifications

| Q | Topic | Answer | Key Update |
|---|-------|--------|-----------|
| Q1 | Selected text handling | D - Enrichment only | FR-005a: Selected text is query meta-data, not new chunks |
| Q2 | RAG API failure | C - Disable gracefully | FR-005: Select-text/search disabled when API down; textbook readable |
| Q3 | Concurrent re-indexing | A - Block queries | FR-006: Queries blocked during <5 min re-indexing window |
| Q4 | CI merge gates | B - All required | FR-007: Links, syntax, code, objectives all block merge |
| Q5 | Chapter structure | B - 6 chapters + sections | FR-001/FR-002: Named sections as sub-headings, not separate folders |
