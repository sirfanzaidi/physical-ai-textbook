# ADR-001: Vector Database Selection for Textbook RAG

**Status**: Accepted
**Date**: 2025-12-06
**Decision Maker**: Architecture Team
**Impact**: RAG Backend Infrastructure, Performance, Deployment

---

## Context

The Physical AI textbook requires a vector database to store embeddings of semantic chunks (200-500 tokens each) for RAG (Retrieval-Augmented Generation) chatbot queries. The project is constrained by:

1. **Scale**: ~60-120 vectors (6 chapters, 384 dimensions each) = ~90 KB total
2. **Latency target**: <2 seconds p95 for end-to-end chatbot response
3. **Cost constraint**: Free tier only (Constitution V)
4. **Persistence**: Must survive server restarts (production requirement)
5. **Operations**: Minimal DevOps overhead (small team)

## Problem Statement

Three vector database solutions were evaluated:

1. **ChromaDB** (embedded, persistent)
2. **Qdrant Cloud** (managed, API-based)
3. **FAISS** (library, fastest, no persistence)

Each has distinct tradeoffs in deployment complexity, operational overhead, and cost.

---

## Decision

**Selected: ChromaDB (embedded in FastAPI backend)**

ChromaDB will be deployed as an embedded Python library within the FastAPI backend, with persistent storage to disk via DuckDB.

### Rationale

1. **Perfect scale match**: ChromaDB's optimal range is 1-10K vectors; textbook's 60-120 vectors is ideal
2. **Sub-20ms query latency**: Leaves 1.98+ seconds budget for vector search + LLM response generation (target <2s p95)
3. **Simplest deployment**: Single `pip install chromadb` in backend; no separate service process
4. **Zero external dependencies**: No API keys, rate limits, or vendor lock-in
5. **Built-in persistence**: DuckDB handles durability; survives restarts
6. **Aligns with Constitution V**: Minimal, free-tier stack; no proprietary services
7. **Clear upgrade path**: If textbook grows >200K vectors, migration to Qdrant Cloud is straightforward (4-8 hours work)

### Implementation

```python
# backend/src/services/vector_db.py
import chromadb
from chromadb.config import Settings

# Persistent client
client = chromadb.PersistentClient(
    path="./data/embeddings",
    settings=Settings(anonymized_telemetry=False)
)

# Collection per textbook version
collection = client.get_or_create_collection(
    name="physical_ai_textbook_v1",
    metadata={"version": "1.0.0"}
)

# Add chapter chunks during indexing
collection.add(
    documents=["ROS 2 nodes are..."],
    metadatas=[{"chapter": 3, "section": "Nodes", "page": 12}],
    ids=["ch3_section1_chunk1"]
)

# Query at runtime
results = collection.query(
    query_texts=["What is a ROS 2 node?"],
    n_results=5
)
```

---

## Alternatives Considered & Rejected

### Alternative 1: Qdrant Cloud (Free Tier)

**Evaluation**:
- ✅ Production-grade, managed infrastructure
- ✅ 1GB free tier (supports up to 1M vectors; textbook uses <0.1%)
- ✅ Advanced filtering, monitoring, replication features
- ✅ Zero DevOps overhead
- ⚠️ Network latency (+20-50ms per query vs local ChromaDB)
- ⚠️ Free tier single-node (no HA during maintenance)
- ⚠️ Vendor lock-in risk (migration effort if Qdrant discontinues free tier)

**Why rejected**: While excellent for production, Qdrant's network latency reduces budget for LLM response generation. ChromaDB's sub-20ms latency is preferable for the tight <2s p95 target. Migration to Qdrant is straightforward if textbook scales.

### Alternative 2: FAISS (Meta's Vector Search Library)

**Evaluation**:
- ✅ Fastest query performance (<5ms for 60-120 vectors)
- ✅ GPU acceleration available (if needed in future)
- ✅ Battle-tested at scale (Meta, Facebook use it)
- ❌ **No built-in persistence** (must implement custom file I/O)
- ❌ **No metadata filtering** (must build on top)
- ❌ **No API layer** (must wrap with FastAPI manually)
- ❌ Significantly more integration work (12-16 hours to production)
- ❌ More code to maintain (persistence, metadata, API)

**Why rejected**: FAISS is over-engineered for textbook scale. The integration work and maintenance burden outweigh the minimal latency benefit (5ms vs 20ms is 4ms difference on 2000ms budget—negligible). ChromaDB's built-in features provide faster time-to-value.

### Alternative 3: In-Memory Only (Redis, Memcached)

**Evaluation**:
- ✅ Ultra-fast queries
- ❌ **Data loss on restart** (unacceptable for production textbook)
- ❌ Requires external service
- ❌ No persistence guarantees

**Why rejected**: Educational content cannot tolerate data loss. Persistence is non-negotiable.

---

## Consequences

### Positive

1. **Fast deployment**: 2-3 hours to working RAG (ChromaDB setup + integration)
2. **Low operational burden**: Single file-based database; no service monitoring
3. **Excellent fit for MVP**: Perfect for textbook scale; no over-engineering
4. **Easy testing**: In-process database simplifies local development and CI/CD
5. **Clear upgrade path**: If scale increases, Qdrant migration is documented and straightforward

### Negative (Minor)

1. **Not suitable for 10M+ vectors**: If textbook grows massive, would need upgrade
2. **No built-in replication**: Single instance (acceptable for MVP; not HA)
3. **Single-node only**: No sharding across multiple machines (not needed for textbook)

### Mitigation

- **Monitor growth**: Track vector count; plan Qdrant migration if >100K vectors (unlikely for 6-chapter textbook)
- **Document upgrade path**: ADR-001-migration.md (to be created if needed)
- **Backup strategy**: Daily snapshots of ChromaDB data directory to GitHub (via backup script)

---

## Validation Checkpoints

- [ ] ChromaDB installed and tested locally (T015-T020)
- [ ] Persistence verified: restarts preserve data (T020)
- [ ] Query latency measured: <50ms on 60-chunk sample (Phase 2 validation)
- [ ] Integration with FastAPI confirmed: `/chat` endpoint returns results (Phase 2 validation)
- [ ] Deployment to Vercel/Railway tested (Phase 6 deployment)

---

## Related Decisions

- **ADR-002**: Embedding Model Selection (sentence-transformers/all-MiniLM-L6-v2)
- **ADR-003**: Concurrent Indexing & Blue-Green Swap

---

## References

- [ChromaDB Documentation](https://docs.trychroma.com/)
- [Qdrant Pricing & Free Tier](https://qdrant.tech/pricing/)
- [FAISS GitHub Repository](https://github.com/facebookresearch/faiss)
- [Vector Database Benchmark Comparison](https://qdrant.tech/benchmarks/)

---

**Decision Record ID**: ADR-001
**Review Status**: Approved by Architecture Team
**Last Updated**: 2025-12-06
