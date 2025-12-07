<!--
  SYNC IMPACT REPORT
  ==================
  Version: 1.1.0 (MINOR bump)
  Bump Rationale: Refined project scope (short, focused textbook); added Simplicity & Minimalism principle; clarified 6-chapter structure; emphasized RAG accuracy constraint
  Modified Principles:
    - Renamed "Content-First Design" → emphasizes accuracy over breadth
    - Added new principle VIII: Simplicity & Minimalism (fast builds, lightweight, no heavy GPU usage)
  Added Sections: 6-Chapter Structure, RAG Accuracy Constraint
  Removed Sections: None (kept all prior sections)
  Templates Updated: spec-template.md (user story scenarios), tasks-template.md (build/deployment tasks) ⚠ pending
  Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics — Essentials Textbook Constitution

## Core Principles

### I. Content-First Design: Accuracy Over Breadth
Every textbook section must prioritize accuracy and depth within tight scope boundaries. Educational clarity MUST be precise and verifiable. Content decisions MUST precede code decisions. Specifications start with learning objectives, concrete examples, and diagrams before architecture or API design. MUST NOT sacrifice accuracy for feature count.

### II. Progressive Disclosure: Linear Knowledge Path
Chapters build systematically from foundational concepts to advanced topics. No chapter assumes knowledge not covered in prior chapters. Each section includes:
- Learning objectives (MUST state what learner will understand)
- Concrete examples (MUST include robotics/AI use cases where applicable)
- Exercises with solutions (REQUIRED for chapters 2+)
- References to prior chapters where applicable

### III. Searchability via RAG: Accuracy-First (NON-NEGOTIABLE)
All content MUST be indexable and retrievable through the Qdrant + embedding pipeline. RAG chatbot MUST answer ONLY from book text—no hallucinations or external knowledge synthesis. MUST use semantic chunks with clear boundaries for RAG. Every section MUST support semantic search with at least 3 distinct query patterns. Embedding quality MUST be validated post-indexing.

### IV. Platform Standardization: Docusaurus
Content MUST be authored in MDX (Markdown + JSX). Docusaurus auto-sidebar generation MUST be primary navigation method (no manual sidebar). Links MUST use Docusaurus routing (relative paths, no absolute URLs). Breaking changes to structure MUST be accompanied by redirect rules in `docusaurus.config.js`. Deployment MUST support GitHub Pages static hosting.

### V. Minimal, Free-Tier Stack
All production dependencies MUST use free or non-proprietary tiers:
- Embeddings: Free models only (e.g., `text-embedding-3-small` via free quota, or open-source alternatives like Ollama)
- Database: Qdrant cloud free tier or self-hosted; Neon PostgreSQL free tier
- RAG Backend: FastAPI (Python) or lightweight Node.js framework
- No proprietary AI services beyond embedding APIs
- Rationale: Accessibility for students; reproducibility of deployment; long-term sustainability

### VI. Accessibility and Internationalization (Optional Path)
When localization added (e.g., Urdu translation), MUST use Docusaurus i18n workflow. Translations MUST be complete for at least one chapter before release. No partial translations without documented completion plan. Optional feature—does NOT block textbook launch.

### VII. Test Coverage for Content Quality
Content quality MUST be validated via automated checks:
- Broken link checks (MUST fail CI on detected links)
- Code example syntax validation (MUST run examples locally before merge)
- Learning objective alignment (MUST verify chapter content matches declared objectives)
- RAG chunk quality (MUST verify semantic coherence and accuracy of indexed content)

### VIII. Simplicity & Minimalism (NON-NEGOTIABLE)
Textbook MUST be short, fast, and simple:
- Maximum 6 chapters, ~50-60 pages total
- No heavy GPU usage in examples or deployment
- Lightweight embedding model (no large LLMs for book generation)
- Fast build times (<30 seconds locally, <2 min CI)
- Clean UI: single-column layout, minimal customization, Docusaurus defaults preferred
- No over-engineering: YAGNI (You Aren't Gonna Need It)
- Rationale: Student accessibility; deployment simplicity; focus on learning, not polish

## 6-Chapter Structure

All content MUST conform to this structure:

1. **Introduction to Physical AI** — Why robotics + AI matter; foundational concepts; learning objectives for entire book
2. **Basics of Humanoid Robotics** — Robot anatomy, actuators, sensors, kinematics (simplified)
3. **ROS 2 Fundamentals** — Nodes, topics, services, launch files; practical setup and examples
4. **Digital Twin Simulation** — Gazebo intro, Isaac Sim basics, running simulations with humanoid models
5. **Vision-Language-Action Systems** — Vision as input, language models for reasoning, action execution pipelines
6. **Capstone: Simple AI-Robot Pipeline** — Hands-on project integrating chapters 1–5; deliverable code and demo

Each chapter MUST include:
- Clear learning objectives (what student can do after reading)
- 2–4 runnable code examples (Python, bash, YAML)
- Exercises with solutions
- ~8–12 pages (text + diagrams)
- Links to prior chapters where applicable

## RAG Accuracy & Select-Text Feature

- **Retrieval Source**: RAG chatbot MUST answer ONLY from the 6 chapters above; NO external knowledge synthesis
- **Select-Text Feature**: User highlights text in book → opens chat prompt with context; chatbot explains only from book
- **Chunk Strategy**: Semantic chunks MUST be 200–500 tokens; boundaries MUST NOT split learning concepts
- **Validation**: Post-indexing, verify all chunks answer ≥3 distinct student queries from each chapter
- **Failure Mode**: If chatbot generates information not in book, MUST fail review before merge

## Technology Stack

- **Framework**: Docusaurus 3.x (latest stable)
- **Content Format**: MDX (Markdown + JSX components)
- **RAG Backend**: FastAPI (Python) or lightweight Node.js + TypeScript
- **Vector Database**: Qdrant (cloud free tier or self-hosted)
- **Embeddings**: Free model only (`text-embedding-3-small` or Ollama/open-source)
- **Database**: Neon PostgreSQL (free tier) for session/user data (optional; may use Qdrant metadata alone)
- **Deployment**: Static textbook on GitHub Pages; RAG API on Vercel free tier or self-hosted
- **Version Control**: Git + GitHub
- **CI/CD**: GitHub Actions (build validation, link checks, syntax validation)
- **Testing**: Docusaurus link checker, code example syntax validators, custom RAG accuracy tests

## Development Workflow

### Authoring & Content Review
- All content lives in `docs/` with Docusaurus-compatible structure (one folder per chapter)
- Branch naming: `docs/<chapter-number>-<title>` for content work (e.g., `docs/02-humanoid-basics`)
- Commit messages MUST include chapter and learning objective
- Code examples MUST be tested locally and committed with expected output
- Content PRs MUST include:
  - Learning objectives clearly stated
  - Chapter alignment matrix (how content covers objectives)
  - Proof of example testing (scripts pass, output visible)
- Review cycle: **Content accuracy review** (domain expert) → **RAG/structure review** (platform maintainer)

### RAG Integration & Deployment
- After content PR merge, chunks MUST be indexed via manual or CI-triggered pipeline
- Stale content MUST be purged if chapter rewritten (>50% diff)
- Embedding model changes MUST trigger full re-indexing
- Select-text feature MUST be tested with ≥5 sample queries per chapter before release
- Deployment gates: Build success, link check pass, RAG accuracy pass

### Continuous Validation
- CI MUST fail on broken external links
- CI MUST validate code example syntax (Python, YAML, bash)
- Docusaurus build MUST complete in <2 min
- RAG API response time MUST be <2 sec (p95)

## Documentation Quality Standards

- **Code Examples**: MUST execute without errors; MUST include expected output comment or docstring
- **External Links**: MUST be verified in CI; MUST update or remove deprecated links
- **Learning Objectives**: MUST use measurable verbs (Bloom's: remember, understand, apply, analyze, synthesize, evaluate)
- **References**: MUST cite ROS 2, Gazebo, Isaac Sim official docs with version info; NO outdated references
- **Figures/Diagrams**: MUST include alt text; SVG or high-res PNG preferred; MUST be versioned in Git
- **Scope**: MUST NOT exceed 50–60 pages total across all 6 chapters; MUST prioritize depth over breadth

## Governance

- **Authority**: This constitution supersedes all design documents and informal practices.
- **Compliance Verification**: Every PR affecting content or RAG MUST validate against Core Principles (checkboxes in PR template).
- **Amendments**: Changes to principles or chapter structure MUST be proposed in separate PRs with justification. Approval requires consensus from technical lead and subject-matter expert.
- **Version Bumping**:
  - **MAJOR**: Principle removal, chapter removal/major restructure, RAG accuracy constraint removed
  - **MINOR**: New principle, new section, material expansion of guidance (e.g., new quality standard)
  - **PATCH**: Clarifications, wording, typos, non-breaking refinements

**Version**: 1.1.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
