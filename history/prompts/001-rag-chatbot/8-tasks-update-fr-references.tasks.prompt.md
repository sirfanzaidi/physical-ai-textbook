---
id: 8
title: Tasks update - FR-001a and FR-007a references
stage: tasks
date: 2025-12-14
surface: agent
model: claude-haiku-4-5-20251001
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: system
command: manual refinement
labels: ["tasks", "requirements", "atomic-units", "select-text-validation"]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/tasks.md
tests:
  - null
---

## Prompt

Update tasks.md to add explicit references to new functional requirements FR-001a (Atomic Units in Chunking) and FR-007a (Select-Text Validation - Minimum 10 Characters) in relevant task descriptions.

## Response snapshot

Updated tasks.md across 7 locations to integrate FR-001a and FR-007a references:

**1. Header update** (lines 7-9):
- Updated prerequisites to note "plan.md (UPDATED with FR-001a & FR-007a), spec.md (UPDATED with FR-001a & FR-007a)"
- Added "data-model.md (COMPLETE)"
- Added new section: "**New Requirements**: FR-001a (Atomic Units in Chunking), FR-007a (Select-Text Validation - Minimum 10 Characters)"

**2. T019 (Foundational Phase)** - Line 63:
- Added "**(FR-001a)**" to chunker.py deliverable description
- Notes: "with semantic chunking logic (300-500 tokens, 200 overlap, recursive splitting) **with atomic-unit preservation (FR-001a)**"

**3. T029 (Unit Test for Chunking)** - Line 86:
- Added FR-001a validation requirement to unit test description
- Notes: "**including atomic-unit validation (FR-001a)**: verify code blocks, tables, equations are never split mid-unit"

**4. T046 (Backend Validation)** - Line 117:
- Added FR-007a explicit specification with HTTP error codes and messages
- Notes: "**(FR-007a)**: minimum 10 characters, return HTTP 400 with message 'Selected text must be at least 10 characters' if <10 chars, and 'Selected text is required for select-text mode' if empty/null"

**5. T047 (Unit Test for Select-Text)** - Line 118:
- Added FR-007a validation testing requirement
- Notes: "**and FR-007a validation**: test empty selection, 1-9 char rejection, 10+ char acceptance with specific HTTP 400 error messages"

**6. T048 (Integration Test)** - Line 119:
- Added FR-007a edge case testing requirement
- Notes: "**FR-007a edge cases**: test empty selection, 5-char selection (expect HTTP 400), 10-char selection (expect success), etc."

**7. T050 (Frontend Select-Text)** - Line 121:
- Added FR-007a frontend validation requirement
- Notes: "**(FR-007a frontend validation)**: reject empty selections, warn if 1-9 characters, allow ≥10 characters, handle HTTP 400 errors from backend"

## Outcome

- ✅ **Impact**: Task list now fully aligned with updated spec and plan. Each task description contains explicit references to FR-001a or FR-007a. Developers can read task descriptions and immediately understand requirement linkage. Test tasks have explicit acceptance criteria tied to requirements (e.g., "test empty selection, 1-9 char rejection, 10+ char acceptance").

- 🧪 **Tests**: Test task descriptions (T029, T047, T048, T050) now have explicit requirement-linked acceptance criteria. Easy for QA to validate that tests cover the requirement intent.

- 📁 **Files**: `specs/001-rag-chatbot/tasks.md` updated with 7 edits (header, T019, T029, T046, T047, T048, T050)

- 🔁 **Next prompts**: (1) Create test-data-policy.md (MEDIUM severity U1) if user wants to address blind eval dataset isolation; (2) Create frontend E2E tests (MEDIUM severity C1) for select-text UI→API integration; (3) Proceed to implementation when ready

- 🧠 **Reflection**: Tasks are now comprehensive and implementable. Each task is traceable to a requirement. Test tasks have measurable acceptance criteria. Teams can pick up tasks with full context on what requirement they're implementing/testing.

## Evaluation notes (flywheel)

- **Failure modes observed**: None; updates were additive and backward-compatible. No breaking changes.
- **Graders run and results**: Task traceability check: PASS. All FR-001a tasks (T019, T029) reference atomic units. All FR-007a tasks (T046, T047, T048, T050) reference select-text validation. Test tasks have explicit acceptance criteria.
- **Prompt variant**: N/A (standard task refinement)
- **Next experiment**: When implementation begins, developer should confirm that test cases match requirement intent by reading task description + linked requirement (spec.md). Example: "T047 asks to test empty selection, 1-9 char rejection, 10+ char acceptance" — developer reads T047 → links to FR-007a in spec.md → confirms test intent.

## Traceability Matrix Created

| Requirement | Phase | Task(s) | Type |
|-------------|-------|---------|------|
| FR-001a (Atomic Units) | Phase 2 | T019 | Implementation |
| FR-001a (Atomic Units) | Phase 3 | T029 | Unit Test |
| FR-007a (Select-Text Validation) | Phase 4 | T046 | Backend Implementation |
| FR-007a (Select-Text Validation) | Phase 4 | T047 | Unit Test |
| FR-007a (Select-Text Validation) | Phase 4 | T048 | Integration Test |
| FR-007a (Select-Text Validation) | Phase 4 | T050 | Frontend Implementation |

All requirements now have explicit implementation + testing coverage in task list.
