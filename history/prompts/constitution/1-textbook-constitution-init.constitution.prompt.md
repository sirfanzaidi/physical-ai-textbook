---
id: 1
title: Textbook Constitution Init
stage: constitution
date: 2025-12-06
surface: agent
model: claude-haiku-4-5-20251001
feature: textbook-generation
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "textbook-generation", "governance"]
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

Feature: textbook-generation

Objective:
Define a complete, unambiguous specification for building the AI-native textbook with RAG chatbot.

Book Structure:
1. Introduction to Physical AI
2. Basics of Humanoid Robotics
3. ROS 2 Fundamentals
4. Digital Twin Simulation (Gazebo + Isaac)
5. Vision-Language-Action Systems
6. Capstone

Technical Requirements:
- Docusaurus
- Auto sidebar
- RAG backend (Qdrant + Neon)
- Free-tier embeddings

Optional:
- Urdu translation
- Personalize chapter

Output:
Full specification.
```

## Response snapshot

Created Physical AI Textbook Constitution v1.0.0 with 7 core principles governing content design, platform standardization, and quality assurance:

**Core Principles:**
1. Content-First Design — Learning objectives precede technical decisions
2. Progressive Disclosure — Systematic chapter progression with no knowledge gaps
3. Searchability via RAG — All content must be semantically indexed and queryable
4. Platform Standardization: Docusaurus — MDX content, auto-sidebar, relative links
5. Minimal, Free-Tier Stack — Free embeddings, Qdrant cloud/self-hosted, Neon PostgreSQL
6. Accessibility & Internationalization — i18n workflow for optional translations
7. Test Coverage for Content Quality — Automated validation of links, code, objectives, RAG chunks

**Additional Sections:**
- Technology Stack: Docusaurus 3.x, Qdrant, Neon, Vercel/GitHub Pages, GitHub Actions
- Development Workflow: Branch naming (`docs/<chapter>`), PR review cycles (educator + platform), RAG re-indexing rules
- Documentation Quality Standards: Code examples must execute, external links verified in CI, Bloom's taxonomy for objectives, SVG diagrams with alt text

**Governance:**
- Constitution supersedes all prior design docs
- Every PR must validate against principles via checklist
- Amendments require consensus from technical and content leads
- Version 1.0.0 (ratified 2025-12-06)

## Outcome

- ✅ Impact: Establishes governance framework for all content and RAG integration work; unambiguous requirements for authors and reviewers
- 🧪 Tests: No tests applicable (governance document)
- 📁 Files: `.specify/memory/constitution.md` (updated with 7 principles + 3 sections)
- 🔁 Next prompts: `/sp.specify` to create detailed feature specification; `/sp.plan` to architect data model and RAG pipeline
- 🧠 Reflection: Constitution aligns with free-tier constraints, emphasizes content quality, and establishes clear review workflow for both educational and technical leads

## Evaluation notes (flywheel)

- Failure modes observed: None (initial version)
- Graders run and results (PASS/FAIL): PASS (all placeholders resolved; no unresolved {{}} tokens; version 1.0.0 matches header)
- Prompt variant (if applicable): null
- Next experiment: Run `/sp.specify` to produce detailed learning outcomes and API contracts for RAG backend
