# Physical AI & Humanoid Robotics: Essentials Textbook

An open-source, AI-native educational textbook for learning Physical AI and humanoid robotics. Features interactive RAG chatbot for real-time Q&A, semantic search, and multi-language support.

**Status**: 🎉 Content Complete - Live at https://sirfanzaidi.github.io/physical-ai-textbook/

---

## Quick Start

### Prerequisites
- Node.js 18+ (for Docusaurus)
- Python 3.11+ (for FastAPI backend)
- Git

### Frontend (Docusaurus Textbook)

```bash
cd website
npm install
npm run start  # Dev server: http://localhost:3000
npm run build  # Production build
```

### Backend (RAG Chatbot API)

```bash
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
python -m uvicorn main:app --reload  # Dev server: http://localhost:8000
```

---

## Project Structure

```
physical-ai-textbook/
├── website/              # Docusaurus frontend (static site)
├── backend/             # FastAPI RAG backend
├── .github/workflows/   # GitHub Actions CI/CD
├── specs/               # Feature specifications & plans
├── history/adr/         # Architectural Decision Records
└── CONTRIBUTING.md      # Chapter authoring guide
```

---

## Features

### Core (MVP - Week 4)
- ✅ **6-chapter textbook** on Physical AI & Humanoid Robotics
- ✅ **RAG chatbot** with select-text → question → answer
- ✅ **Semantic search** across chapters
- ✅ **Auto-validation** CI/CD (code, links, learning objectives)
- ✅ **Zero-downtime updates** with blue-green collection swap
- ✅ **WCAG 2.1 AA** accessibility

### Future (Post-MVP)
- ⏳ Advanced search with relevance ranking
- ⏳ Admin dashboard for RAG monitoring
- ⏳ Urdu translation (Docusaurus i18n ready)
- ⏳ Personalization engine
- ⏳ User authentication via Better-Auth

---

## Technology Stack

| Component | Technology | Tier |
|-----------|-----------|------|
| **Frontend** | Docusaurus 3.x + React + MDX | Free (GitHub Pages) |
| **Backend** | FastAPI (Python 3.11) | Free (Vercel/Railway) |
| **Vector DB** | ChromaDB (embedded) | Free (local) |
| **Embeddings** | sentence-transformers/all-MiniLM-L6-v2 | Free (open-source) |
| **CI/CD** | GitHub Actions | Free |
| **Hosting** | GitHub Pages + Vercel/Railway | Free |

---

## Chapter Structure

### 6 Chapters (50-60 pages total)

1. **Introduction to Physical AI** - Foundational concepts, why robotics + AI matter
2. **Basics of Humanoid Robotics** - Anatomy, actuators, sensors, kinematics
3. **ROS 2 Fundamentals** - Nodes, topics, services, launch files
4. **Digital Twin Simulation** - Gazebo, Isaac Sim, running simulations
5. **Vision-Language-Action Systems** - Vision, language models, action pipelines
6. **Capstone: Simple AI-Robot Pipeline** - Hands-on project, end-to-end integration

### Quality Standards

Each chapter includes:
- **Learning objectives** (Bloom's taxonomy verbs)
- **2-4 code examples** (Python, bash, YAML)
- **Exercises with solutions** (chapters 2-6)
- **5+ credible references** (official docs, papers, books)
- **8-12 pages** of content

---

## Contributing

### For Chapter Authors

See `CONTRIBUTING.md` for:
- Chapter MDX template
- Quality checklist (10 items)
- Commit message conventions
- PR process & CI validation gates
- Code example standards
- Reference standards

**Quick start**:
```bash
git checkout -b docs/03-ros2-fundamentals
# Write chapter in website/docs/03-ros2-fundamentals/index.mdx
git add website/docs/03-ros2-fundamentals/
git commit -m "[chapter] 3: ROS 2 Fundamentals - Add nodes and topics section"
git push origin docs/03-ros2-fundamentals
# Open PR; CI validation runs automatically
```

### For Developers

See `specs/1-textbook-rag/`:
- `spec.md` - Feature requirements
- `plan.md` - Architecture & tech stack
- `tasks.md` - 98 actionable tasks (6 phases)

**Phase 1 setup** (current):
- Initialize Docusaurus & FastAPI
- Setup GitHub Actions workflows
- Create configuration files

**Phase 2-3** (next):
- RAG backend implementation (embeddings, ChromaDB, FastAPI endpoints)
- Chapter authoring (6 concurrent writers)

---

## RAG Chatbot Accuracy

Target: **≥90% accuracy** on 18+ test queries per chapter

- Chatbot answers **ONLY from book content** (no hallucinations)
- Selected text is query context, not indexed content
- Semantic chunks: 200-500 tokens each
- Validation suite: 3+ queries per chapter before deployment

---

## Deployment

### GitHub Pages (Static Textbook)

```bash
cd website
npm run build
# Auto-deployed to gh-pages branch via GitHub Actions
```

**Live URL**: https://your-github-username.github.io/physical-ai-textbook/

### RAG API Backend

Deploy FastAPI to Vercel, Railway, or self-hosted:

```bash
cd backend
# Configure deployment platform
# (See .github/workflows/rag-api.yml for CI/CD)
```

### CI/CD Validation Gates (Auto-Run on PR)

All checks **block merge** on failure:
- ✅ Markdown syntax validation
- ✅ Code example execution & output verification
- ✅ Learning objectives presence & alignment
- ✅ External link validation
- ✅ Docusaurus build success (<4 min)
- ✅ RAG indexing & accuracy validation (≥90%)

---

## Architecture Decisions

Three key architectural decisions documented as ADRs:

1. **ADR-001**: Vector Database Selection → **ChromaDB** (embedded, zero external dependencies)
2. **ADR-002**: Embedding Model Selection → **sentence-transformers/all-MiniLM-L6-v2** (free, fast, battle-tested)
3. **ADR-003**: Concurrent Indexing → **Blue-green collection swap** (zero-downtime, atomic updates)

See `history/adr/` for full rationale, alternatives considered, and validation checkpoints.

---

## Timeline

| Phase | Duration | Tasks | Output |
|-------|----------|-------|--------|
| **Phase 1** | Week 1 | Setup (T001-T010) | Project initialized |
| **Phase 2** | Week 1-2 | Foundation (T011-T031) | RAG backend ready |
| **Phase 3** | Week 2 | Chapters (T032-T053) | 6 chapters authored |
| **Phase 4** | Week 2-3 | Chatbot (T054-T066) | RAG integrated, validated |
| **Phase 5** | Week 3 | Publishing (T067-T076) | CI/CD workflow live |
| **Phase 6** | Week 4 | Testing (T077-T093) | QA, accessibility, deploy |

**Estimated MVP Launch**: 4-6 weeks with 2-3 developers

---

## License

MIT License - See `LICENSE.md` for details

This textbook is free and open-source. You're welcome to:
- ✅ Use, modify, and distribute
- ✅ Use in educational settings (universities, bootcamps, self-study)
- ✅ Create derivatives (translations, adaptations, specialized editions)

**Attribution required**: Please link back to this repository.

---

## Support & Questions

- 📖 **Documentation**: See `CONTRIBUTING.md` (authors) and `specs/1-textbook-rag/` (developers)
- 🏗️ **Architecture**: Review `history/adr/` for design decisions
- 🐛 **Issues**: Open GitHub issues with `[question]` or `[bug]` labels
- 💬 **Discussions**: Use GitHub Discussions for architecture questions

---

## Project Status

| Phase | Status | Completion |
|-------|--------|-----------|
| **Phase 0: Research** | ✅ Complete | 100% |
| **Phase 1: Setup** | ✅ Complete | 100% |
| **Phase 2: Foundation** | ✅ Complete | 100% |
| **Phase 3: Chapters** | ✅ Complete | 100% |
| **Phase 4: Chatbot** | ✅ Complete | 100% |
| **Phase 5: Publishing** | ✅ Complete | 100% |
| **Phase 6: Testing** | 🟢 In Progress | 80% |

### ✅ What's Complete:

**Infrastructure (Phase 1)**
- ✅ Docusaurus website initialized and configured
- ✅ FastAPI backend setup with project structure
- ✅ GitHub Actions workflows (Docusaurus build + RAG indexing)
- ✅ GitHub Pages deployment configured

**RAG Backend (Phase 2)**
- ✅ Pydantic data models for all API contracts
- ✅ EmbeddingService (sentence-transformers integration)
- ✅ VectorDBService (ChromaDB wrapper)
- ✅ ChunkingService (semantic paragraph-based)
- ✅ FastAPI endpoints (/health, /chat, /reindex, /validate)
- ✅ CI/CD scripts (ingest.py, validate.py)

**Content (Phase 3)**
- ✅ **Chapter 1**: Introduction to Physical AI (~2,500 words)
- ✅ **Chapter 2**: Humanoid Robotics (~3,200 words)
- ✅ **Chapter 3**: ROS2 Fundamentals (~3,500 words)
- ✅ **Chapter 4**: Digital Twin (~3,400 words)
- ✅ **Chapter 5**: Vision-Language-Action Systems (~3,600 words)
- ✅ **Chapter 6**: Capstone Project (~4,100 words)
- ✅ Total: ~20,000 words with exercises, case studies, and references

**Deployment (Phase 5)**
- ✅ Live textbook site: https://sirfanzaidi.github.io/physical-ai-textbook/
- ✅ Automated GitHub Pages deployment
- ✅ RAG indexing workflow configured

### ✅ Phase 4: Chatbot Integration Complete
- ✅ Backend API complete
- ✅ Frontend chatbot UI implemented with message history
- ✅ LLM response generation integrated
- ✅ Beautiful homepage redesign with Chapter Showcase component

### ✅ Phase 5: Publishing Complete
- ✅ 5 professional blog posts on robotics & AI topics
- ✅ Responsive design across all devices
- ✅ SEO optimization with metadata

### 🟢 In Progress: Testing & Optimization (Phase 6)
- ✅ Ingestion script validated with all chapters
- ✅ Path discovery fixed for Docusaurus structure
- ⏳ End-to-end RAG query testing (in progress)
- ⏳ Accuracy validation with test queries (in progress)

---

**Ready to contribute?** Start with `CONTRIBUTING.md` and `specs/1-textbook-rag/spec.md`.

🚀 **Live Textbook**: https://sirfanzaidi.github.io/physical-ai-textbook/

---

*Last updated: 2025-12-08 | Status: MVP Complete - 6 Chapters, Chatbot UI, 5 Blog Posts, Homepage Redesign*
