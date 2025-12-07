# ADR-003: Concurrent Indexing & Blue-Green Collection Swap for Zero-Downtime Updates

**Status**: Accepted
**Date**: 2025-12-06
**Decision Maker**: Architecture Team
**Impact**: RAG Backend CI/CD, Availability, Reliability

---

## Context

The textbook system must support concurrent chapter authoring: Multiple authors write chapters and merge PRs within minutes. Each merge triggers RAG re-indexing to add chapters to the vector database. Requirements:

1. **No stale chunks**: Chatbot always serves current content (not outdated embeddings)
2. **No downtime**: Readers should not see "Chatbot offline" messages during re-indexing
3. **No data loss**: In-flight queries should not be dropped
4. **Fast re-indexing**: <5 minutes per chapter update (from FR-006)
5. **Atomic updates**: Either old or new content is served; never partial/inconsistent state

## Problem Statement

Four strategies were evaluated for managing concurrent chapter updates:

1. **Hard-block chatbot** during re-indexing (simple, poor UX)
2. **Blue-green collection swap** (zero-downtime, atomic) ← **SELECTED**
3. **Delta indexing only** (fast but incomplete validation)
4. **Tombstone soft-delete** (complex, adds storage overhead)

---

## Decision

**Selected: Blue-Green Collection Swap + GitHub Actions Concurrency Control**

This strategy uses Docusaurus/ChromaDB collection aliases to atomically swap between old and new collections, with GitHub Actions concurrency groups to serialize re-indexing jobs.

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  GitHub Actions CI/CD Pipeline                               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  1. PR Merge to main (docs/** changed)                │  │
│  │  2. Concurrency group: "rag-indexing" (queued)       │  │
│  │  3. Git diff → detect modified chapters              │  │
│  │  4. Create new collection (green)                    │  │
│  │  5. Index modified chapters + unchanged copy        │  │
│  │  6. Validation suite runs (18+ queries)              │  │
│  │  7. Atomically swap: alias "prod" → green           │  │
│  │  8. Delete old collection (blue)                     │  │
│  │  9. Chatbot immediately serves new content           │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘

Result:
- ✅ Zero downtime (old collection stays live until new one passes validation)
- ✅ No stale chunks (validation before swap)
- ✅ Atomic transition (alias swap is all-or-nothing)
- ✅ Instant rollback (if validation fails, keep blue)
```

### Implementation

**GitHub Actions Concurrency**:
```yaml
# .github/workflows/rag-indexing.yml
concurrency:
  group: rag-indexing
  cancel-in-progress: false  # CRITICAL: Queue, don't cancel
```

**Blue-Green Swap Logic**:
```python
# backend/scripts/reindex.py

# Step 1: Create new collection (green)
new_collection = chromadb_client.create_collection(
    name=f"textbook_v{timestamp}",  # e.g., textbook_v20251206_143022
    metadata={"status": "pending_validation", "created_at": timestamp}
)

# Step 2: Index chapters (delta or full)
modified_chapters = detect_changes_from_git_diff()
for chapter in modified_chapters:
    chunks = chunk_chapter(chapter)
    embeddings = embed_batch(chunks)
    new_collection.add(embeddings)

# Step 3: Run validation (18+ queries per chapter)
validation_results = run_validation_suite(new_collection)
if not validation_results.passed():
    raise ValidationError("RAG accuracy <90%")

# Step 4: Atomic alias swap
chromadb_client.update_collection_aliases([
    chromadb.DeleteAlias("prod"),
    chromadb.CreateAlias("prod", new_collection.name)
])

# Step 5: Clean up old collection
for collection in chromadb_client.list_collections():
    if collection.name != "prod":
        chromadb_client.delete_collection(collection.name)
```

### Timeline for Concurrent PRs

```
10:00 AM: PR 1 merge (Chapter 1)
         → Job 1 starts: Create new_v1 → Index Ch1 → Validate → Swap → Done (3 min)
         → Chatbot serves new content immediately

10:15 AM: PR 2 merge (Chapter 2)
         → Job 2 queued (waits for Job 1)
         → Job 1 finishes; Job 2 starts
         → Job 2: Create new_v2 → Index Ch2 → Validate → Swap → Done (3 min)
         → Chatbot serves updated content

10:30 AM: PR 3 merge (Chapter 3)
         → Job 3 queued (waits for Job 2)
         → Similar timeline
```

---

## Alternatives Considered & Rejected

### Alternative 1: Hard-Block Chatbot During Re-indexing

**Approach**: Set maintenance flag; return "Chatbot offline" during re-indexing

```python
# middleware.py
if redis_client.exists("indexing_in_progress"):
    return {"error": "Chatbot maintenance in progress; back online in ~5 minutes"}
```

**Evaluation**:
- ✅ Simple to implement (1-2 hours)
- ⚠️ **Poor user experience**: 5-min downtime per merge
- ⚠️ **Cumulative impact**: If 3 PRs merge in 15 min, chatbot offline 15 min total
- ⚠️ **Unacceptable for learning tool**: Students interrupted mid-lesson

**Why rejected**: Educational tool requires high availability. Hard-blocking chatbot defeats purpose of interactive learning.

### Alternative 2: Delta Indexing Only (No Blue-Green)

**Approach**: Update collection in-place; only re-index modified chapters

```python
# Fast for single-chapter updates
old_chunks = collection.query(where={"chapter": 3})
collection.delete(ids=old_chunks.ids)  # Delete old Ch3
collection.add(new_chunks)  # Add new Ch3
```

**Evaluation**:
- ✅ Fast for 1-2 chapter updates (7-13 sec vs 30-60 sec full)
- ❌ **Incomplete validation**: Old chapters never re-validated
- ❌ **Risky for embedding model upgrades**: Old vectors coexist with new ones
- ❌ **No rollback**: If validation fails, data is partially corrupted
- ❌ **Stale chunks possible**: Incorrect deletions could leave orphaned chunks

**Why rejected**: While faster for incremental updates, delta lacks safety guarantees. Cannot rollback if validation fails. Blue-green swap provides safer, more predictable behavior despite slightly longer total time.

### Alternative 3: Tombstone Soft-Delete Strategy

**Approach**: Mark old chunks as deleted; retrieval filters them out

```python
collection.update(
    ids=old_chunk_ids,
    metadatas=[{"deleted": True}] * len(old_chunk_ids)
)

# Retrieve only non-deleted chunks
results = collection.query(
    where={"deleted": {"$ne": True}}
)
```

**Evaluation**:
- ✅ Simpler than blue-green (no collection recreation)
- ⚠️ **Adds storage overhead**: Dead data persists until hard delete
- ⚠️ **Query complexity**: Every query must filter deleted items
- ⚠️ **Fragmentation over time**: Database bloats with soft-deleted data
- ⚠️ **Less clear state**: Which version is current?

**Why rejected**: Adds unnecessary complexity and storage overhead. Blue-green swap is cleaner: one collection is always "current" (via alias), old data is completely removed.

---

## Consequences

### Positive

1. **Zero downtime**: Chatbot continues serving old content during indexing; swaps atomically when ready
2. **No stale chunks**: Validation runs before swap; guaranteed current data
3. **Instant rollback**: If validation fails, keep old collection; no data loss
4. **Atomic semantics**: Alias swap is all-or-nothing; no partial states
5. **Clear observability**: Can track collection age via metadata (`created_at` timestamp)
6. **Concurrent PR handling**: GitHub Actions concurrency groups serialize jobs safely

### Negative (Minor)

1. **Temporary 2x storage**: During indexing, both collections exist in memory (~180 KB for textbook scale, negligible)
2. **Slightly longer total time**: Full re-index (30-60 sec) vs delta (7-13 sec) per PR
   - Mitigation: Run weekly full re-index; daily delta for individual chapters (future optimization)

### Performance Impact

| Operation | Time |
|-----------|------|
| Create collection + index chapters | 20-30 sec |
| Validation suite (18+ queries) | 10-20 sec |
| Alias swap | <100 ms |
| **Total per PR** | **30-60 sec** |
| **Total for 3 concurrent PRs** | **~2 minutes** (queued) |

**Budget impact**: Still well under 5-min target (FR-006)

---

## GitHub Actions Concurrency Control

Critical: Use concurrency groups to serialize indexing jobs

```yaml
concurrency:
  group: rag-indexing
  cancel-in-progress: false  # Queue, don't cancel (important!)
```

**Without this**: Two merges within seconds → both jobs run concurrently → race condition → corrupted database

**With this**: Job 1 runs → Job 2 queued → Job 1 finishes → Job 2 runs automatically

---

## Validation Checkpoints

- [ ] Blue-green collection swap implemented and tested (T031)
- [ ] Atomic alias operation verified (manual test with mock data)
- [ ] GitHub Actions concurrency group configured and tested (push 2 PRs simultaneously; verify serialization)
- [ ] Validation suite runs before swap (Phase 4D)
- [ ] Rollback tested: Validation intentionally fails → old collection remains active
- [ ] Monitor collection versions in production (track metadata `created_at`)

---

## Monitoring & Debugging

**Observability**:
- Log collection swap operations with timestamps
- Track validation pass/fail rates per chapter
- Alert if validation <90% (requires manual investigation + fix)

**Debugging**:
```bash
# List collections and ages
python backend/scripts/debug_collections.py

# Manual rollback (if needed)
python backend/scripts/rollback_to_collection.py --collection textbook_v20251206_090000
```

---

## Future Optimizations (Post-MVP)

1. **Delta indexing + blue-green**: Create new collection with only modified chapters + copy unchanged chunks (faster)
2. **Async validation**: Start validation before swap completes; queue swap operation
3. **Multi-region replication**: Backup collections to secondary storage (not needed for MVP)
4. **Chunking optimization**: Use larger chunks (500-800 tokens) to reduce collection size

---

## Related Decisions

- **ADR-001**: Vector Database Selection (ChromaDB supports collections/aliases)
- **ADR-002**: Embedding Model Selection (affects indexing speed)

---

## References

- [Blue-Green Deployments Pattern](https://martinfowler.com/bliki/BlueGreenDeployment.html)
- [GitHub Actions Concurrency](https://docs.github.com/en/actions/using-jobs/using-concurrency)
- [ChromaDB Collections & Aliases](https://docs.trychroma.com/usage/collection)
- [Database Concurrency Patterns](https://en.wikipedia.org/wiki/Concurrency_control)

---

**Decision Record ID**: ADR-003
**Review Status**: Approved by Architecture Team
**Last Updated**: 2025-12-06
