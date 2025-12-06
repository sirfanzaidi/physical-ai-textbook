---
id: 2
title: Refined Constitution v1.1
stage: constitution
date: 2025-12-06
surface: agent
model: claude-haiku-4-5-20251001
feature: textbook-generation
branch: master
user: user
command: /sp.constitution (amendment)
labels: ["constitution", "textbook-generation", "governance", "scope-refinement"]
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

Project Name: Physical AI & Humanoid Robotics — Essentials

Purpose:
Create a short, clean, professional AI-Native textbook based on the Physical AI & Humanoid Robotics course.
The book must serve as a fast, simple, high-quality learning resource built with a modern Docusaurus UI
and a fully integrated free-tier RAG chatbot.

Scope:
- 6 short chapters:
  1. Introduction to Physical AI
  2. Basics of Humanoid Robotics
  3. ROS 2 Fundamentals
  4. Digital Twin Simulation (Gazebo + Isaac)
  5. Vision-Language-Action Systems
  6. Capstone: Simple AI-Robot Pipeline
- Clean UI
- Free-tier friendly
- Lightweight embeddings

Core Principles:
- Simplicity
- Accuracy
- Minimalism
- Fast builds
- Free-tier architecture
- RAG answers ONLY from book text

Key Features:
- Docusaurus textbook
- RAG chatbot (Qdrant + Neon + FastAPI)
- Select-text → Ask AI
- Optional Urdu / Personalize features

Constraints:
- No heavy GPU usage
- Minimal embeddings

Success Criteria:
- Build success
- Accurate chatbot
- Clean UI
- Smooth GitHub Pages deployment

Generate full constitution.
```

## Response snapshot

Updated Physical AI & Humanoid Robotics — Essentials Textbook Constitution v1.1.0 (MINOR version bump). Key changes:

**Modified Principles:**
1. Content-First Design → emphasizes accuracy over breadth; MUST NOT sacrifice accuracy for features
2. Added principle VIII: Simplicity & Minimalism (6 chapters max, <60 pages, <30s local build, no heavy GPU, YAGNI)
3. RAG principle clarified: accuracy-first, ONLY from book text, no hallucinations

**New Sections:**
- **6-Chapter Structure**: Prescribed chapters with mandatory elements (learning objectives, 2–4 code examples, exercises, 8–12 pages each, prior chapter links)
- **RAG Accuracy & Select-Text Feature**: Chunk strategy (200–500 tokens), validation rules (≥3 queries per chapter answered from book), select-text feature gates
- **Technology Stack**: Explicit choice of FastAPI or Node.js; free embeddings only; Qdrant + Neon + GitHub Pages + Vercel
- **Development Workflow**: Branch naming, review cycle (content accuracy → RAG/structure), CI gates (link checks, syntax validation, RAG accuracy tests)
- **Documentation Quality Standards**: Added scope constraint (50–60 pages max), Bloom's taxonomy verbs, SVG diagrams, no outdated references

**Governance Updated:**
- Version bumping rules clarified (MAJOR: chapter removal, RAG constraint removed; MINOR: new principle/section; PATCH: wording)
- PR compliance: every PR MUST validate against Core Principles

**Version**: 1.0.0 → 1.1.0
**Bump Type**: MINOR (added principles VIII, new sections, refined constraints)
**Ratification Date**: 2025-12-06 (today)

## Outcome

- ✅ Impact: Establishes unambiguous scope, emphasizes RAG accuracy and content quality constraints, sets build/performance gates, enables parallel chapter authoring
- 🧪 Tests: No tests applicable (governance document)
- 📁 Files: `.specify/memory/constitution.md` (v1.0.0 → v1.1.0; added 8 sections; refined all principles)
- 🔁 Next prompts: `/sp.specify` to generate feature spec with user stories; `/sp.plan` to architect RAG backend and CI pipeline
- 🧠 Reflection: Constitution now aligns project scope (short, focused, accurate), codifies RAG requirements (accuracy-first, only-from-book), and establishes clear quality gates for content and chatbot

## Evaluation notes (flywheel)

- Failure modes observed: None (well-structured amendment)
- Graders run and results (PASS/FAIL): PASS (all placeholders resolved; 8 principles + 6 sections; version 1.1.0 matches header; dates ISO; no YAGNI violations)
- Prompt variant (if applicable): null
- Next experiment: Run `/sp.specify` to produce detailed user stories (textbook reader, chatbot user, admin) and acceptance criteria
