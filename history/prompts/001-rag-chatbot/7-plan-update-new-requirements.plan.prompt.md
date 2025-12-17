---
id: 7
title: Plan update - reference new requirements
stage: plan
date: 2025-12-14
surface: agent
model: claude-haiku-4-5-20251001
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: system
command: manual refinement
labels: ["plan", "requirements", "atomic-units", "select-text-validation"]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/plan.md
tests:
  - null
---

## Prompt

Update plan.md to reference new functional requirements FR-001a (Atomic Units) and FR-007a (Select-Text Validation) that were added to spec.md to address HIGH severity issues.

## Response snapshot

Updated plan.md across 6 locations to integrate new requirements:

**1. Header update** (line 4):
- Changed from "15 functional requirements" to "17 functional requirements (15 + 2 new: FR-001a atomic units, FR-007a select-text validation)"

**2. Phase 2 (Book Ingestion - Lines 186-213)**:
- Added "with **atomic-unit preservation** (FR-001a)" to chunker deliverable
- Expanded Key Activity #2 with explicit atomic-unit rules: code blocks, tables, equations, lists as atomic units; expand boundaries; post-indexing validation
- Added success criteria: "No code blocks or tables split mid-unit (FR-001a validation)"

**3. Phase 4 (FastAPI Backend - Lines 271-294)**:
- Added "**Implement select-text validation (FR-007a)**" subsection with:
  - Minimum 10 character requirement
  - Specific HTTP 400 error messages ("Selected text must be at least 10 characters", "Selected text is required for select-text mode")
- Updated input validation to highlight "**Selected text minimum 10 characters (FR-007a)**"
- Updated integration tests note to "include select-text edge cases"

**4. Phase 5 (Frontend Widget - Lines 324-332)**:
- Renamed activity #2 to "Implement select-text feature with validation (FR-007a)"
- Added explicit validation steps: reject empty, warn 1-9 chars, allow ≥10 chars
- Added error handling: "Handle backend error responses (HTTP 400 for <10 chars)"

**5. Technical Decision #2 (Chunking Strategy - Line 466)**:
- Added "atomic-unit preservation - FR-001a" to section title
- Added new subsection explaining FR-001a: code blocks/tables/equations MUST NEVER be split; expand boundary if needed; post-indexing validation

**6. Technical Decision #5 (Select-Text Isolation - Line 482)**:
- Renamed to "Select-Text Isolation & Validation (FR-007a)"
- Added FR-007a explanation: backend enforces 10-character minimum, rejects <10 with HTTP 400
- Updated rationale: 10-char minimum prevents noise and ensures meaningful context

## Outcome

- ✅ **Impact**: Plan now fully aligned with updated spec. Developers have clear references to FR-001a and FR-007a throughout all phases. Atomic-unit preservation logic is explicit in Phase 2. Select-text validation is explicit in Phase 4 (backend) and Phase 5 (frontend). Technical decisions explain rationale for both new requirements.

- 🧪 **Tests**: No test tasks changed; existing test tasks (T029, T046-T047) now have clearer requirements via explicit FR references

- 📁 **Files**: `specs/001-rag-chatbot/plan.md` updated with 6 strategic edits (header, 4 phase updates, 2 technical decision updates)

- 🔁 **Next prompts**: (1) Update tasks.md to add FR-001a references to T019/T029 (chunking tasks); (2) Update tasks.md to add FR-007a references to T046/T047 (select-text validation tasks); (3) Create test-data-policy.md (MEDIUM severity U1) if needed

- 🧠 **Reflection**: Plan is now cohesive and implementable. All phases have clear linkage to requirements. Technical decisions justify the architectural choices (atomic units, 10-char minimum). Frontend developers can implement with confidence on validation rules.

## Evaluation notes (flywheel)

- **Failure modes observed**: None; updates were purely additive (no breaking changes, no deletions, backward-compatible)
- **Graders run and results**: Plan integrity check: PASS. All phases reference relevant requirements. All technical decisions explain rationale. No circular dependencies. Phases still independent and parallelizable.
- **Prompt variant**: N/A (standard plan refinement)
- **Next experiment**: If user approves, update tasks.md to add FR-001a and FR-007a references to specific task descriptions, ensuring task descriptions quote the requirements directly.
