# ADR-002: Embedding Model Selection for RAG Queries

**Status**: Accepted
**Date**: 2025-12-06
**Decision Maker**: Architecture Team
**Impact**: RAG Backend Embeddings, Performance, Cost

---

## Context

The RAG chatbot requires an embedding model to convert user queries and chapter chunks into vector space for semantic similarity search. Constraints:

1. **Free tier only**: No API costs or external service dependencies (Constitution V)
2. **Fast**: <100ms per query (leaves budget for vector search + LLM response in <2s total)
3. **Accurate**: Adequate quality for educational robotics/AI content (not high-dimensional scientific research)
4. **Scalable**: Batch processing capability for initial indexing (~60 chunks, <2 seconds)
5. **Lightweight**: Runs on CPU (no GPU required; free tier servers lack GPU)

## Problem Statement

Four embedding solutions were evaluated:

1. **OpenAI text-embedding-3-small** (API)
2. **Ollama** (open-source, local)
3. **sentence-transformers/all-MiniLM-L6-v2** (open-source, local) ← **SELECTED**
4. **FAISS** (library, requires custom integration)

Each has distinct tradeoffs in cost, latency, quality, and deployment complexity.

---

## Decision

**Selected: sentence-transformers/all-MiniLM-L6-v2**

This model will be deployed locally in the FastAPI backend using the `sentence-transformers` Python library.

### Rationale

1. **Truly free**: Open-source; no API costs, rate limits, or external dependencies
2. **Fast embeddings**: 30-80ms per query (leaves 1.92+ seconds for vector search + LLM)
3. **Lightweight**: 384 dimensions (minimal storage: ~90KB for entire textbook)
4. **Batch-friendly**: Process 60 chunks in <2 seconds
5. **Battle-tested**: Most popular sentence transformer; 15K+ production models built on this framework
6. **Domain-robust**: Performs well on robotics/AI educational text
7. **CPU-friendly**: Runs on any hardware (no GPU required)
8. **Aligns with Constitution**: Minimal, free-tier stack; no proprietary services

### Implementation

```python
# backend/src/services/embeddings.py
from sentence_transformers import SentenceTransformer

# Load model (80MB, one-time download ~2 min)
model = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')

# Batch embed chapters during indexing
chapter_chunks = [...]  # ~60-120 chunks
embeddings = model.encode(
    chapter_chunks,
    batch_size=32,
    show_progress_bar=True
)
# Expected time: <2 seconds for full textbook

# Query-time embedding
query = "What is a ROS 2 node?"
query_embedding = model.encode(query)
# Expected time: 30-80ms
```

### Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| **Dimensions** | 384 | Compact; minimal storage |
| **Latency** | 30-80ms | Well within 2s p95 budget |
| **Batch throughput** | 100+ chunks/sec | <2 sec for full textbook indexing |
| **MTEB score** | 56 | Adequate for educational content (OpenAI is 71) |
| **Model size** | 80 MB | Tiny; downloads in ~2 minutes |
| **CPU/GPU** | CPU recommended | Works on any hardware |

---

## Alternatives Considered & Rejected

### Alternative 1: OpenAI text-embedding-3-small

**Evaluation**:
- ✅ High quality (MTEB 71, better than all-MiniLM)
- ✅ Official OpenAI support
- ✅ Easy integration with LangChain, etc.
- ❌ **Free tier too restrictive**: 3 requests/minute (would require 20+ minutes for batch indexing)
- ❌ **Not truly free for production**: $0.02 per 1M tokens (costs accumulate)
- ❌ **External API dependency**: Network latency, rate limits
- ❌ **Violates Constitution V**: Proprietary service

**Why rejected**: Free tier rate limiting makes batch indexing impractical (60 chunks would take 20+ minutes at 3 req/min). Not suitable for production despite higher quality. Constitution mandates free-tier-only stack.

### Alternative 2: Ollama (nomic-embed-text)

**Evaluation**:
- ✅ Completely free, open-source
- ✅ No external dependencies
- ✅ Strong MTEB score (57, similar to all-MiniLM)
- ⚠️ **Slower**: 100-200ms per query (depending on CPU; crosses latency budget)
- ⚠️ **Larger model**: ~400MB download (vs 80MB for all-MiniLM)
- ⚠️ **More complex setup**: Requires Ollama service running separately

**Why rejected**: Latency 100-200ms approaches budget ceiling (leaves only 1.8s for search + LLM). all-MiniLM's 30-80ms is more comfortable. Simpler deployment favored for MVP.

### Alternative 3: FAISS (DIY Embedding + Storage)

**Evaluation**:
- ✅ Fastest query performance (<5ms)
- ❌ **Not a turnkey solution**: FAISS is a library, not a model
- ❌ Requires choosing a separate embedding model anyway
- ❌ Requires custom persistence layer
- ❌ Requires custom metadata storage
- ❌ Excessive integration work for minimal gain

**Why rejected**: FAISS solves the "search" problem, not the "embedding" problem. Still need to choose an embedding model (which would be sentence-transformers or OpenAI). FAISS adds complexity without benefit for textbook scale.

### Alternative 4: Custom Fine-Tuned Model

**Evaluation**:
- ✅ Potentially best quality for robotics domain
- ❌ Requires labeled training data (thousands of pairs)
- ❌ Requires machine learning expertise
- ❌ 8-12 weeks development time (outside MVP scope)
- ❌ Not justified for 6 chapters

**Why rejected**: Out of scope for MVP. all-MiniLM is sufficient for educational robotics content (MTEB 56 is respectable). Fine-tuning can be explored post-MVP if accuracy needs improvement.

---

## Consequences

### Positive

1. **Fast embeddings**: 30-80ms leaves comfortable budget for 2s p95 target
2. **Truly free**: No cost, no rate limits, no external dependencies
3. **Simple deployment**: Single library; no separate service
4. **Excellent for MVP**: Perfect fit for textbook scale
5. **Clear fallback**: If accuracy <95%, upgrade to all-mpnet-base-v2 (higher quality, still free)
6. **Production-ready**: Battle-tested on 15K+ models

### Negative (Minor)

1. **MTEB score 56 vs 71 (OpenAI)**: Slightly lower quality, adequate for educational content
2. **Requires local compute**: Can't use serverless API (but acceptable; FastAPI backend handles it)

### Mitigation

- **Validate during Phase 4D**: Run 18+ test queries; verify ≥90% accuracy
- **Fallback documented**: If accuracy <95%, upgrade to all-mpnet-base-v2 (768 dims, MTEB 62, still free)
- **Monitor quality**: Track RAG validation results post-launch; adjust if needed

---

## Fallback Plan

If validation (Phase 4D) reveals accuracy <90%:

```python
# Option A: Upgrade to all-mpnet-base-v2
model = SentenceTransformer('sentence-transformers/all-mpnet-base-v2')
# Trade-off: 768 dims (2x storage), 50-120ms latency (still within budget)

# Option B: Increase top-k results
# Instead of top-5, use top-10 chunks
results = collection.query(
    query_texts=["..."],
    n_results=10  # up from 5
)

# Option C: Improve chunking strategy
# Use larger chunks (500-800 tokens) to preserve context
```

---

## Validation Checkpoints

- [ ] Model downloaded and loaded locally (T015)
- [ ] Batch embedding tested: 60 chunks in <2 seconds (T015)
- [ ] Query embedding latency measured: <100ms (T016)
- [ ] Integration with ChromaDB verified (T017)
- [ ] RAG validation suite runs: 18+ queries tested (Phase 4D)
- [ ] Accuracy ≥90% confirmed before deployment (Phase 6)

---

## Related Decisions

- **ADR-001**: Vector Database Selection (ChromaDB)
- **ADR-003**: Concurrent Indexing & Blue-Green Swap

---

## References

- [Sentence Transformers Documentation](https://www.sbert.net/)
- [all-MiniLM-L6-v2 HuggingFace](https://huggingface.co/sentence-transformers/all-MiniLM-L6-v2)
- [MTEB Leaderboard](https://huggingface.co/spaces/mteb/leaderboard)
- [Embedding Model Comparison 2025](https://supermemory.ai/blog/best-open-source-embedding-models-benchmarked-and-ranked/)

---

**Decision Record ID**: ADR-002
**Review Status**: Approved by Architecture Team
**Last Updated**: 2025-12-06
