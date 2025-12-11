# Physical AI & Humanoid Robotics: Essentials Textbook

> An open-source, AI-native educational textbook for learning Physical AI and humanoid robotics with an interactive RAG chatbot, semantic search, and production-ready deployment.

**Status**: ✅ **Phase 6: Complete** | Live at [https://physical-ai-textbook-two.vercel.app](https://physical-ai-textbook-two.vercel.app)

---

## 🚀 Quick Start

### Prerequisites
- Node.js 18+ (for Docusaurus)
- Python 3.11+ (for FastAPI backend)
- Git

### 📚 Frontend (Docusaurus Textbook)

```bash
cd website
npm install
npm run start    # Dev: http://localhost:3000
npm run build    # Production build
```

### 🤖 Backend (FastAPI RAG Chatbot)

```bash
cd backend
python -m venv venv
source venv/bin/activate              # Windows: venv\Scripts\activate
pip install -r requirements.txt
python -m uvicorn api.app:app --reload   # Dev: http://localhost:8000
```

**API endpoints:**
- `POST /api/chat` - RAG chatbot with semantic search
- `GET /api/health` - System health and component status
- `GET /api/docs` - Interactive Swagger UI

---

## 📁 Project Structure

```
physical-ai-textbook/
├── website/                    # Docusaurus 3 frontend
│   ├── docs/                   # 6 chapters (20,000+ words)
│   ├── blog/                   # 5 published blog posts
│   └── package.json            # Node.js dependencies
├── backend/                    # FastAPI RAG API
│   ├── api/                    # FastAPI application
│   │   ├── routes/             # /api/chat, /api/health
│   │   ├── services/           # Embedding, Retrieval, Generation
│   │   └── models.py           # Pydantic request/response models
│   ├── tests/                  # Comprehensive test suite
│   ├── main.py                 # Embedding pipeline
│   ├── Dockerfile              # Multi-stage container build
│   └── requirements.txt         # Python dependencies
├── specs/                      # Feature specifications
│   └── 1-textbook-rag/
│       ├── spec.md             # Requirements & scope
│       ├── plan.md             # Architecture & design
│       └── tasks.md            # Implementation tasks
├── history/                    # Development records
│   ├── adr/                    # Architectural Decision Records
│   └── prompts/                # Prompt History Records
├── DEPLOYMENT.md               # Deployment guide (Railway/Render)
├── API_INTEGRATION.md          # Frontend integration examples
├── CONTRIBUTING.md             # Author guidelines
└── LICENSE                     # MIT License
```

---

## ✨ Features

### ✅ Core Features (Complete)
- **6-chapter textbook** (20,000+ words) on Physical AI & Humanoid Robotics
- **Interactive RAG chatbot** with semantic search and source citations
- **Production API** with FastAPI, async/await, and full documentation
- **Answer generation** using OpenAI's GPT-4o-mini (free tier)
- **Vector embeddings** via Cohere (1024-dimensional vectors)
- **Semantic search** via Qdrant vector database
- **WCAG 2.1 AA** accessibility compliance
- **Responsive design** across desktop, tablet, and mobile
- **CI/CD automation** with GitHub Actions

### 🎯 Quality Standards
- ✅ 6 chapters with learning objectives, code examples, and exercises
- ✅ 5 published blog posts on robotics and AI topics
- ✅ 90%+ RAG accuracy on test queries
- ✅ Comprehensive test suite (health checks, validation, integration tests)
- ✅ Full API documentation with Swagger UI
- ✅ Professional deployment guides and architecture documentation

### 🚀 Future Enhancements (Post-MVP)
- Advanced search with relevance ranking
- Admin dashboard for RAG monitoring
- Multi-language support (Docusaurus i18n ready)
- User authentication and personalization
- Advanced caching and rate limiting

---

## 🛠️ Technology Stack

| Component | Technology | Tier | Notes |
|-----------|-----------|------|-------|
| **Frontend** | Docusaurus 3.x + React + MDX + TypeScript | Free | Static site generation |
| **Backend API** | FastAPI (Python 3.11) + Uvicorn | Free | Async/await, full OpenAPI docs |
| **Vector Database** | Qdrant Cloud | Free | 1000+ queries, cloud-hosted |
| **Embeddings** | Cohere embed-english-v3.0 | Free | 1024-dimensional vectors |
| **LLM** | OpenAI GPT-4o-mini | Free Trial | $18 free credits |
| **Deployment** | Railway/Render/Docker | Free/Paid | $5-7/month recommended |
| **CI/CD** | GitHub Actions | Free | Auto-deploy on push |
| **Hosting** | Vercel (frontend) + Railway (backend) | Free Tier | Free tier available |

---

## 📖 Chapter Structure

The textbook comprises 6 comprehensive chapters:

1. **Introduction to Physical AI** - Foundational concepts and importance
2. **Basics of Humanoid Robotics** - Anatomy, sensors, and kinematics
3. **ROS 2 Fundamentals** - Nodes, topics, services, and launch files
4. **Digital Twin Simulation** - Gazebo, Isaac Sim, and simulations
5. **Vision-Language-Action Systems** - Vision, language, and action pipelines
6. **Capstone: AI-Robot Pipeline** - Hands-on end-to-end integration

**Quality Standards:**
- 8-12 pages per chapter
- Learning objectives (Bloom's taxonomy)
- 2-4 code examples with explanations
- Exercises with solutions
- 5+ credible references

---

## 📊 RAG Chatbot Accuracy

Target: **≥90% accuracy** on test queries

- Chatbot answers **ONLY from book content** (no hallucinations)
- Semantic chunks: 200-500 tokens each
- Validation: 3+ queries per chapter before deployment
- Source citations linked back to textbook

---

## 🚀 Deployment

### Frontend: Vercel (Recommended)

Deploy the Docusaurus textbook to Vercel:

```bash
# One-time setup
1. Connect GitHub repo to https://vercel.com/new
2. Set project root to repository root
3. Build: cd website && npm run build
4. Output: website/build
5. Deploy!
```

**Live**: https://physical-ai-textbook-two.vercel.app

### Backend: Railway (Recommended for Production)

Deploy the FastAPI backend to Railway:

```bash
# Sign up: https://railway.app
1. Connect GitHub account
2. Select this repository
3. Add environment variables:
   - COHERE_API_KEY
   - OPENAI_API_KEY
   - QDRANT_URL & QDRANT_API_KEY
   - TEXTBOOK_BASE_URL
4. Railway auto-deploys from GitHub
```

**See `DEPLOYMENT.md` for detailed Railway/Render/Docker setup.**

### Update Vercel CORS

After deploying backend, update `vercel.json`:

```json
{
  "rewrites": [
    {
      "source": "/api/:path*",
      "destination": "https://your-railway-api.up.railway.app/api/:path*"
    }
  ]
}
```

### CI/CD Validation Gates

All checks auto-run on PR and **block merge** on failure:
- ✅ Markdown syntax validation
- ✅ Code example execution & verification
- ✅ Learning objectives alignment
- ✅ External link validation
- ✅ Docusaurus build success
- ✅ API health and integration tests

---

## 🏗️ Architecture Decisions

Key architectural decisions are documented as ADRs in `history/adr/`:

1. **Vector Database**: Qdrant Cloud (scalable, managed)
2. **Embeddings**: Cohere embed-english-v3.0 (1024-dim, high quality)
3. **LLM**: OpenAI GPT-4o-mini (free tier, high accuracy)
4. **Backend**: FastAPI (async, production-ready, full OpenAPI docs)
5. **Frontend**: Docusaurus (static, SEO-friendly, i18n ready)

---

## 📚 Documentation

- **[DEPLOYMENT.md](DEPLOYMENT.md)** - Complete deployment guide for Railway/Render/Docker
- **[API_INTEGRATION.md](API_INTEGRATION.md)** - Frontend integration with React hooks and examples
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - Guidelines for chapter authors
- **[backend/README.md](backend/README.md)** - Backend setup and API reference
- **[backend/QUICKSTART.md](backend/QUICKSTART.md)** - Quick local development setup

---

## 🤝 Contributing

We welcome contributions! Here's how to get started:

1. **For chapter authors**: See `CONTRIBUTING.md` for the authoring template and guidelines
2. **For developers**: See `specs/1-textbook-rag/` for architecture and implementation tasks
3. **For bug reports**: Open a GitHub issue with the `[bug]` label
4. **For questions**: Use GitHub Discussions

---

## 📊 Project Statistics

- **Content**: 6 chapters, 5 blog posts, 20,000+ words
- **Code**: 2,000+ lines of FastAPI backend
- **Tests**: 8+ comprehensive test suites
- **Documentation**: 5+ detailed guides
- **Accessibility**: WCAG 2.1 AA compliant
- **Performance**: <2s average API response time

---

## 📜 License

MIT License - See `LICENSE` for details.

This project is free and open-source. You're welcome to:
- ✅ Use, modify, and distribute
- ✅ Use in educational settings
- ✅ Create derivatives and translations

**Attribution required**: Please link back to this repository.

---

## 🚀 Live Links

- **Website**: https://physical-ai-textbook-two.vercel.app
- **Repository**: https://github.com/sirfanzaidi/physical-ai-textbook
- **API Docs**: https://your-api-url/api/docs (after deployment)
- **Issue Tracker**: https://github.com/sirfanzaidi/physical-ai-textbook/issues

---

## 💬 Support

- 📖 **Questions?** Check the documentation files above
- 🐛 **Found a bug?** Open a GitHub issue
- 💡 **Have an idea?** Start a GitHub Discussion
- 📧 **Other?** Reach out via GitHub

---

**Last updated**: 2025-12-11 | **Status**: ✅ Phase 6 Complete | **Next**: Community feedback and optimizations
