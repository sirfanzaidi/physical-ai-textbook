# Data Model: Integrated RAG Chatbot for Published Books

**Document**: Design specification for entity relationships, database schemas, and storage layer architecture
**Date**: 2025-12-14
**Version**: 1.0
**Audience**: Backend developers, database architects, API contract owners

---

## Overview

This document defines the complete data model for the RAG chatbot system. The architecture uses a **dual-storage pattern**:

1. **Qdrant Cloud** (vector embeddings): Fast semantic search on chunked book content
2. **Neon PostgreSQL** (relational metadata): Source attribution, chunk lineage, query history, and analytics

This separation ensures optimal performance (vector operations in Qdrant) while maintaining queryable lineage and audit trails (PostgreSQL).

---

## Core Entities

### Entity 1: Book

Represents a published book being indexed into the RAG system.

**Purpose**: Track book metadata, versioning, and lifecycle (upload, indexing, re-indexing, deletion)

**Primary Storage**: Neon PostgreSQL (metadata table: `books`)

**Relational**: Foreign key referenced by Chunks, Queries, Sessions

```sql
-- Neon PostgreSQL: books table
CREATE TABLE books (
    book_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    title VARCHAR(255) NOT NULL,
    author VARCHAR(255),
    isbn VARCHAR(20),
    total_pages INTEGER NOT NULL,
    upload_date TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    indexed_date TIMESTAMP,
    file_path VARCHAR(500),  -- Path in cloud storage (if applicable)
    file_hash VARCHAR(64),   -- SHA-256 of original file for deduplication
    language VARCHAR(10) DEFAULT 'en',
    file_size_bytes BIGINT,
    status VARCHAR(50) DEFAULT 'pending' CHECK (status IN ('pending', 'indexing', 'indexed', 'failed', 'archived')),
    error_message TEXT,      -- Populated if status = 'failed'
    chunk_count INTEGER,     -- Total semantic chunks created
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    -- Indexes
    CONSTRAINT fk_book_chunks FOREIGN KEY (book_id) REFERENCES chunks(book_id) ON DELETE CASCADE
);

CREATE INDEX idx_books_status ON books(status);
CREATE INDEX idx_books_title ON books(title);
CREATE INDEX idx_books_file_hash ON books(file_hash);
```

**Fields**:
| Field | Type | Constraint | Purpose |
|-------|------|-----------|---------|
| book_id | UUID | PK | Unique identifier |
| title | VARCHAR(255) | NOT NULL | Book title |
| author | VARCHAR(255) | nullable | Author name |
| isbn | VARCHAR(20) | nullable | ISBN-13 or ISBN-10 |
| total_pages | INTEGER | NOT NULL | Total page count |
| upload_date | TIMESTAMP | NOT NULL | When file uploaded |
| indexed_date | TIMESTAMP | nullable | When indexing completed |
| file_path | VARCHAR(500) | nullable | Cloud storage path (S3, GCS, etc.) |
| file_hash | VARCHAR(64) | nullable, UNIQUE | SHA-256 for deduplication |
| language | VARCHAR(10) | DEFAULT 'en' | Language code (en, es, fr, etc.) |
| file_size_bytes | BIGINT | nullable | Original file size |
| status | VARCHAR(50) | CHECK | pending \| indexing \| indexed \| failed \| archived |
| error_message | TEXT | nullable | Error details if status=failed |
| chunk_count | INTEGER | nullable | Total chunks created |
| created_at | TIMESTAMP | NOT NULL | Record creation time |
| updated_at | TIMESTAMP | NOT NULL | Last modification time |

**Example**:
```json
{
  "book_id": "550e8400-e29b-41d4-a716-446655440000",
  "title": "Introduction to Machine Learning",
  "author": "Alice Smith",
  "isbn": "978-1-491-92230-3",
  "total_pages": 450,
  "upload_date": "2025-12-14T10:30:00Z",
  "indexed_date": "2025-12-14T11:15:00Z",
  "file_hash": "a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6",
  "language": "en",
  "file_size_bytes": 12500000,
  "status": "indexed",
  "chunk_count": 3847,
  "created_at": "2025-12-14T10:30:00Z",
  "updated_at": "2025-12-14T11:15:00Z"
}
```

---

### Entity 2: Chunk

Represents a semantic segment of a book (300–500 tokens, preserving conceptual boundaries).

**Purpose**: Store indexed text segments with embeddings and metadata for retrieval

**Primary Storage**:
- **Embeddings + Vector**: Qdrant Cloud (collection: `book_vectors`)
- **Metadata + Lineage**: Neon PostgreSQL (table: `chunks_metadata`)

**Dual-Storage Pattern Rationale**:
- Qdrant stores embeddings for vector similarity search (optimized for ANN queries)
- Neon stores metadata (page, section, chapter, text_hash) for re-indexing, attribution, and analytics (optimized for relational queries)

#### Qdrant Collection Schema

```python
# Qdrant Cloud collection: book_vectors
# dimension: 1024 (Cohere embed-v4.0 output dimension)
# metric: cosine (similarity-based retrieval)

collection_config = {
    "name": "book_vectors",
    "vectors": {
        "size": 1024,
        "distance": "Cosine"
    },
    "payload_schema": {
        "chunk_id": {
            "type": "keyword",
            "index": True  # For exact match in select-text mode
        },
        "book_id": {
            "type": "keyword",
            "index": True  # For filtering by book
        },
        "text": {
            "type": "text",
            "index": False  # Store but don't index (large field)
        },
        "page_number": {
            "type": "integer",
            "index": True  # For page-based filtering
        },
        "section_name": {
            "type": "keyword",
            "index": True  # For section attribution
        },
        "chapter_name": {
            "type": "keyword",
            "index": True  # For chapter attribution
        },
        "text_hash": {
            "type": "keyword",
            "index": True  # For deduplication and select-text matching
        },
        "token_count": {
            "type": "integer",
            "index": False  # For monitoring chunk size distribution
        },
        "created_at": {
            "type": "text",  # ISO 8601 timestamp
            "index": False
        }
    }
}
```

**Qdrant Point Structure** (example):
```json
{
  "id": 12345,  // Qdrant internal ID (auto-generated)
  "vector": [0.123, -0.456, ..., 0.789],  // 1024-dimensional embedding
  "payload": {
    "chunk_id": "chunk-550e8400-e29b-41d4",
    "book_id": "book-550e8400-e29b-41d4",
    "text": "Chapter 1: Fundamentals of Machine Learning. Machine learning is a subset of artificial intelligence...",
    "page_number": 5,
    "section_name": "1.1 What is Machine Learning?",
    "chapter_name": "Chapter 1: Fundamentals",
    "text_hash": "b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6q7",
    "token_count": 342,
    "created_at": "2025-12-14T11:15:00Z"
  }
}
```

#### Neon PostgreSQL Metadata Schema

```sql
-- Neon PostgreSQL: chunks_metadata table
CREATE TABLE chunks_metadata (
    id SERIAL PRIMARY KEY,
    chunk_id UUID NOT NULL UNIQUE,
    book_id UUID NOT NULL,
    page_number INTEGER,
    section_name VARCHAR(255),
    chapter_name VARCHAR(255),
    text_hash VARCHAR(64),  -- SHA-256 of chunk text for deduplication
    token_count INTEGER,
    embedding_dimension INTEGER DEFAULT 1024,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    indexed_date TIMESTAMP,

    -- Constraints
    CONSTRAINT fk_chunks_book FOREIGN KEY (book_id) REFERENCES books(book_id) ON DELETE CASCADE
);

-- Indexes for efficient queries
CREATE INDEX idx_chunks_book_id ON chunks_metadata(book_id);
CREATE INDEX idx_chunks_chunk_id ON chunks_metadata(chunk_id);
CREATE INDEX idx_chunks_text_hash ON chunks_metadata(text_hash);
CREATE INDEX idx_chunks_page ON chunks_metadata(page_number);
CREATE UNIQUE INDEX idx_chunks_book_hash ON chunks_metadata(book_id, text_hash);

-- Composite index for re-indexing (delete old chunks by book_id)
CREATE INDEX idx_chunks_reindex ON chunks_metadata(book_id, indexed_date);
```

**Metadata Fields**:
| Field | Type | Constraint | Purpose |
|-------|------|-----------|---------|
| id | SERIAL | PK | Auto-increment primary key |
| chunk_id | UUID | UNIQUE, NOT NULL | Stable identifier (matches Qdrant payload) |
| book_id | UUID | FK (books) | Links to parent book |
| page_number | INTEGER | nullable | 1-based page number for attribution |
| section_name | VARCHAR(255) | nullable | Section heading (e.g., "1.1 What is ML?") |
| chapter_name | VARCHAR(255) | nullable | Chapter name (e.g., "Chapter 1: Fundamentals") |
| text_hash | VARCHAR(64) | nullable | SHA-256 of chunk text for deduplication |
| token_count | INTEGER | nullable | Token count (for validation) |
| embedding_dimension | INTEGER | DEFAULT 1024 | Embedding vector dimension |
| created_at | TIMESTAMP | NOT NULL | Record creation time |
| indexed_date | TIMESTAMP | nullable | When embedded and stored in Qdrant |

**Example**:
```json
{
  "id": 1001,
  "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
  "book_id": "550e8400-e29b-41d4-a716-446655440001",
  "page_number": 5,
  "section_name": "1.1 What is Machine Learning?",
  "chapter_name": "Chapter 1: Fundamentals",
  "text_hash": "b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6q7",
  "token_count": 342,
  "embedding_dimension": 1024,
  "created_at": "2025-12-14T11:15:00Z",
  "indexed_date": "2025-12-14T11:15:30Z"
}
```

---

### Entity 3: Query

Represents a user's question submitted to the chatbot.

**Purpose**: Track all queries for audit, accuracy evaluation, and quality assurance

**Primary Storage**: Neon PostgreSQL (table: `queries`)

```sql
-- Neon PostgreSQL: queries table
CREATE TABLE queries (
    query_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    book_id UUID NOT NULL,
    user_id VARCHAR(255),  -- Anonymous user identifier (hashed or session ID)
    query_text TEXT NOT NULL,
    mode VARCHAR(20) DEFAULT 'full' CHECK (mode IN ('full', 'selected')),
    selected_text_hash VARCHAR(64),  -- text_hash if mode='selected'
    response_text TEXT,
    retrieved_chunk_ids TEXT[],  -- Array of chunk IDs used in response
    latency_ms BIGINT,  -- Time to generate response (ms)
    accuracy_flag VARCHAR(20) CHECK (accuracy_flag IN ('correct', 'partial', 'incorrect', 'null')),
    accuracy_notes TEXT,  -- Human notes on evaluation
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    CONSTRAINT fk_queries_book FOREIGN KEY (book_id) REFERENCES books(book_id) ON DELETE CASCADE
);

-- Indexes
CREATE INDEX idx_queries_book_id ON queries(book_id);
CREATE INDEX idx_queries_user_id ON queries(user_id);
CREATE INDEX idx_queries_mode ON queries(mode);
CREATE INDEX idx_queries_created_at ON queries(created_at);
CREATE INDEX idx_queries_accuracy ON queries(accuracy_flag);
```

**Fields**:
| Field | Type | Constraint | Purpose |
|-------|------|-----------|---------|
| query_id | UUID | PK | Unique identifier |
| book_id | UUID | FK (books) | Which book was queried |
| user_id | VARCHAR(255) | nullable | Anonymized user identifier |
| query_text | TEXT | NOT NULL | The question asked |
| mode | VARCHAR(20) | DEFAULT 'full' | full \| selected (select-text mode) |
| selected_text_hash | VARCHAR(64) | nullable | text_hash if mode='selected' |
| response_text | TEXT | nullable | Generated response |
| retrieved_chunk_ids | TEXT[] | nullable | Array of chunk IDs used (PostgreSQL array type) |
| latency_ms | BIGINT | nullable | Response latency in milliseconds |
| accuracy_flag | VARCHAR(20) | nullable | correct \| partial \| incorrect \| null (for human evaluation) |
| accuracy_notes | TEXT | nullable | Human reviewer notes |
| created_at | TIMESTAMP | NOT NULL | Query submission time |
| updated_at | TIMESTAMP | NOT NULL | Last update time |

**Example**:
```json
{
  "query_id": "550e8400-e29b-41d4-a716-446655440002",
  "book_id": "550e8400-e29b-41d4-a716-446655440001",
  "user_id": "user_hash_abc123",
  "query_text": "What are the main types of machine learning?",
  "mode": "full",
  "selected_text_hash": null,
  "response_text": "Machine learning can be broadly categorized into three types...",
  "retrieved_chunk_ids": ["chunk-001", "chunk-042", "chunk-089"],
  "latency_ms": 2340,
  "accuracy_flag": "correct",
  "accuracy_notes": "Response accurately covers supervised, unsupervised, and reinforcement learning from Chapter 1.",
  "created_at": "2025-12-14T14:30:00Z",
  "updated_at": "2025-12-14T14:30:05Z"
}
```

---

### Entity 4: Session (Optional)

Represents a user's chat session with the bot (conversation history).

**Purpose**: Enable multi-turn conversations, history persistence, and usage tracking

**Primary Storage**: Neon PostgreSQL (table: `sessions`)

**Note**: Sessions are optional for MVP. Implement if multi-turn support is desired.

```sql
-- Neon PostgreSQL: sessions table
CREATE TABLE sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(255),
    book_id UUID NOT NULL,
    title VARCHAR(255) DEFAULT 'Chat Session',
    query_count INTEGER DEFAULT 0,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP,  -- Auto-delete after 30 days

    -- Constraints
    CONSTRAINT fk_sessions_book FOREIGN KEY (book_id) REFERENCES books(book_id) ON DELETE CASCADE
);

-- Indexes
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_book_id ON sessions(book_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);
```

**Fields**:
| Field | Type | Constraint | Purpose |
|-------|------|-----------|---------|
| session_id | UUID | PK | Unique identifier |
| user_id | VARCHAR(255) | nullable | Anonymized user identifier |
| book_id | UUID | FK (books) | Which book in session |
| title | VARCHAR(255) | DEFAULT 'Chat Session' | User-provided or auto-generated title |
| query_count | INTEGER | DEFAULT 0 | Total queries in session |
| created_at | TIMESTAMP | NOT NULL | Session start time |
| updated_at | TIMESTAMP | NOT NULL | Last activity time |
| expires_at | TIMESTAMP | nullable | Automatic deletion time (e.g., +30 days) |

**Link to Queries**: Session contains multiple queries
```sql
-- In queries table, add optional session_id
ALTER TABLE queries ADD COLUMN session_id UUID REFERENCES sessions(session_id) ON DELETE CASCADE;
```

---

## Storage Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      User Query Request                          │
└────────────────┬────────────────────────────────────────────────┘
                 │
                 ▼
        ┌────────────────────┐
        │  FastAPI Backend   │
        │  (routes.py)       │
        └────────┬───────────┘
                 │
         ┌───────┴───────┐
         │               │
         ▼               ▼
    ┌─────────┐    ┌──────────┐
    │ Qdrant  │    │   Neon   │
    │  Cloud  │    │PostgreSQL│
    └─────────┘    └──────────┘
         │               │
         │               │
    ┌────▼────┐     ┌────▼──────┐
    │ Vectors │     │  Metadata  │
    │ (ANN)   │     │ (Relational)
    └─────────┘     └────────────┘
         │               │
         │ (embed_v4.0) │
         │ (search_doc) │
         │ (search_query)
         │
    ┌────▼──────────────┐
    │  Cohere APIs      │
    │ - Embed v4.0      │
    │ - Rerank v4.0-pro │
    │ - Chat (Generate) │
    └───────────────────┘
```

---

## API Contracts & Data Flow

### Ingest Flow (Book Upload)

```
POST /ingest
│
├─ Input: File (PDF/text)
│
├─ Create: Book record (status='pending')
│
├─ Process:
│  ├─ Extract text from file
│  ├─ Chunk semantically (300-500 tokens)
│  ├─ Embed with Cohere (input_type="search_document")
│  ├─ Upsert to Qdrant (vectors + metadata payload)
│  └─ Insert to Neon (chunks_metadata)
│
├─ Update: Book record (status='indexed', chunk_count=N)
│
└─ Output: { book_id, status, chunk_count, indexed_date }
```

### Query Flow (Full-Book Mode)

```
POST /chat { query, mode='full' }
│
├─ Input: query_text, book_id, mode='full'
│
├─ Create: Query record (status='pending')
│
├─ Retrieve:
│  ├─ Embed query with Cohere (input_type="search_query")
│  ├─ Search Qdrant (top_k=20-30, filter by book_id)
│  ├─ Rerank with Cohere (final top_k=8-10)
│  └─ Assemble documents for Cohere Chat API
│
├─ Generate:
│  ├─ Call Cohere Chat with documents parameter
│  └─ Stream or batch response
│
├─ Update: Query record (response_text, retrieved_chunk_ids, latency_ms)
│
└─ Output: { response, citations (page, section, chapter), latency }
```

### Query Flow (Select-Text Mode)

```
POST /chat { query, mode='selected', selected_text }
│
├─ Input: query_text, book_id, mode='selected', selected_text
│
├─ Validate: selected_text ≥ 10 characters
│
├─ Hash: selected_text → text_hash (SHA-256)
│
├─ Query Neon: Find chunks with matching text_hash
│  └─ Retrieve: chunk_ids, chunk pages, sections
│
├─ Create: Query record (mode='selected', selected_text_hash=...)
│
├─ Retrieve (constrained):
│  ├─ Embed query with Cohere (input_type="search_query")
│  ├─ Search Qdrant (filter: text_hash IN selected_chunks)
│  │  ├─ Get top_k=8-10 within selected passage ONLY
│  │  └─ Reject any results outside selected_text_hash
│  └─ Assemble documents (guaranteed from selected passage only)
│
├─ Generate:
│  ├─ Call Cohere Chat with selected-passage documents
│  └─ Response grounded ONLY in selected text
│
├─ Update: Query record (response_text, retrieved_chunk_ids, latency_ms)
│
└─ Output: { response, selected_passage_attribution, latency, zero_leakage_verified }
```

---

## Re-Indexing Strategy

**Use Case**: Admin uploads updated version of a book; system must:
1. Delete all old chunks from Qdrant and Neon for the same book_id
2. Process and store new chunks
3. Guarantee no duplication or orphaned records

**Implementation**:

```python
# backend/ingestion/main.py

def reindex_book(book_id: UUID, file_path: str) -> dict:
    """
    Re-index a book: delete old, insert new chunks.
    Guarantees idempotence and no duplication.
    """

    # Step 1: Mark book status as 're-indexing'
    update_book_status(book_id, "re-indexing")

    # Step 2: Retrieve old chunks from Neon (metadata only)
    old_chunks = query_neon(
        "SELECT chunk_id FROM chunks_metadata WHERE book_id = %s",
        (book_id,)
    )
    old_chunk_ids = [c['chunk_id'] for c in old_chunks]

    # Step 3: Delete from Qdrant (by chunk_id)
    qdrant_client.delete(
        collection_name="book_vectors",
        points_selector=qdrant.models.PointIdList(
            ids=[qdrant_id for qdrant_id in get_qdrant_ids_for_chunks(old_chunk_ids)]
        )
    )

    # Step 4: Delete from Neon (by book_id)
    execute_neon(
        "DELETE FROM chunks_metadata WHERE book_id = %s",
        (book_id,)
    )

    # Step 5: Process new file
    new_chunks = ingest_and_chunk(file_path, book_id)

    # Step 6: Embed and store new chunks
    for chunk in new_chunks:
        embedding = embed_with_cohere(chunk['text'])
        upsert_to_qdrant(chunk, embedding)
        insert_to_neon(chunk, book_id)

    # Step 7: Update book status and chunk count
    update_book_status(book_id, "indexed", chunk_count=len(new_chunks))

    return {
        "book_id": str(book_id),
        "old_chunk_count": len(old_chunks),
        "new_chunk_count": len(new_chunks),
        "status": "indexed",
        "indexed_date": datetime.utcnow().isoformat()
    }
```

---

## Indexes & Query Optimization

### Critical Indexes (Neon PostgreSQL)

| Table | Index | Columns | Purpose |
|-------|-------|---------|---------|
| books | idx_books_status | status | Filter books by indexing status |
| books | idx_books_file_hash | file_hash | Deduplication (prevent duplicate uploads) |
| chunks_metadata | idx_chunks_book_id | book_id | Re-indexing (delete old chunks by book) |
| chunks_metadata | idx_chunks_text_hash | text_hash | Select-text mode (find chunks by selected text) |
| chunks_metadata | idx_chunks_reindex | (book_id, indexed_date) | Composite: re-indexing with timestamp filtering |
| queries | idx_queries_book_id | book_id | Analytical queries by book |
| queries | idx_queries_accuracy | accuracy_flag | Accuracy evaluation queries |
| queries | idx_queries_created_at | created_at | Time-based filtering (e.g., last 7 days) |

### Qdrant Indexes (Vector Storage)

| Payload Field | Index | Purpose |
|---------------|-------|---------|
| book_id | Yes | Filter vectors by book (full-book mode) |
| chunk_id | Yes | Exact match for deduplication |
| text_hash | Yes | Select-text mode (filter by selected passage) |
| page_number | Yes | Optional: page-level filtering |

---

## Data Retention & Cleanup Policies

### Book Records
- **Retention**: Indefinite (until admin deletes)
- **Cleanup**: Manual deletion via admin endpoint
- **Cascade**: Deleting book cascades to chunks_metadata, queries, sessions

### Query Records
- **Retention**: 90 days (configurable)
- **Cleanup**: Automated weekly job to purge queries older than 90 days
- **Purpose**: Audit trail for accuracy review; purge after analysis

### Sessions (Optional)
- **Retention**: 30 days (configurable via expires_at)
- **Cleanup**: Automated daily job to delete expired sessions
- **Purpose**: Clean up inactive user sessions

**Migration Script** (PostgreSQL):
```sql
-- Weekly cleanup job (run via cron or pg_cron extension)
CREATE EXTENSION IF NOT EXISTS pg_cron;

SELECT cron.schedule(
    'purge_old_queries',
    '0 2 * * *',  -- 2 AM UTC daily
    'DELETE FROM queries WHERE created_at < NOW() - INTERVAL ''90 days'''
);

SELECT cron.schedule(
    'purge_expired_sessions',
    '0 1 * * *',  -- 1 AM UTC daily
    'DELETE FROM sessions WHERE expires_at < NOW()'
);
```

---

## Data Validation & Constraints

### Input Validation (API Layer)

| Entity | Field | Validation Rule | Error Code |
|--------|-------|-----------------|-----------|
| Book | total_pages | 1–10,000 | 400 BAD_REQUEST |
| Book | file_size | <500MB | 413 PAYLOAD_TOO_LARGE |
| Chunk | text | 10–5000 characters | 400 BAD_REQUEST |
| Chunk | token_count | 100–1000 | 400 BAD_REQUEST |
| Query | query_text | 1–5000 characters | 400 BAD_REQUEST |
| Query (selected) | selected_text | 10–10000 characters | 400 BAD_REQUEST |
| Session | title | 1–255 characters | 400 BAD_REQUEST |

### Database Constraints

| Constraint | Table | Rationale |
|-----------|-------|-----------|
| UNIQUE (chunk_id) | chunks_metadata | Prevent duplicate chunks |
| UNIQUE (book_id, text_hash) | chunks_metadata | Prevent duplicate chunks within same book |
| UNIQUE (file_hash) | books | Prevent duplicate book uploads |
| FK (book_id) → books | chunks_metadata | Ensure chunks link to valid books |
| FK (book_id) → books | queries | Ensure queries link to valid books |
| CHECK (status IN (...)) | books | Enforce valid status values |
| CHECK (mode IN (...)) | queries | Enforce full \| selected modes |

---

## Monitoring & Metrics

### Key Metrics Tracked (queries table)

```python
# Latency distribution (from queries.latency_ms)
metrics = {
    "p50_latency_ms": 1200,
    "p95_latency_ms": 3500,
    "p99_latency_ms": 4800,
}

# Accuracy metrics (from queries.accuracy_flag)
accuracy_metrics = {
    "total_queries": 2500,
    "correct_count": 2350,
    "partial_count": 100,
    "incorrect_count": 50,
    "accuracy_percentage": 94.0,  # (correct / total) * 100
}

# Query mode distribution
mode_metrics = {
    "full_mode_count": 2000,
    "selected_mode_count": 500,
}
```

### Queries for Monitoring

```sql
-- Accuracy over time (last 7 days)
SELECT
    DATE(created_at) as day,
    ROUND(100.0 * SUM(CASE WHEN accuracy_flag = 'correct' THEN 1 ELSE 0 END) / COUNT(*), 2) as accuracy_pct
FROM queries
WHERE created_at > NOW() - INTERVAL '7 days'
GROUP BY DATE(created_at)
ORDER BY day DESC;

-- Latency distribution
SELECT
    PERCENTILE_CONT(0.50) WITHIN GROUP (ORDER BY latency_ms) as p50,
    PERCENTILE_CONT(0.95) WITHIN GROUP (ORDER BY latency_ms) as p95,
    PERCENTILE_CONT(0.99) WITHIN GROUP (ORDER BY latency_ms) as p99
FROM queries
WHERE created_at > NOW() - INTERVAL '24 hours';

-- Chunk distribution by book
SELECT
    b.title,
    COUNT(c.chunk_id) as chunk_count,
    AVG(c.token_count) as avg_tokens,
    MIN(c.token_count) as min_tokens,
    MAX(c.token_count) as max_tokens
FROM books b
LEFT JOIN chunks_metadata c ON b.book_id = c.book_id
WHERE b.status = 'indexed'
GROUP BY b.book_id, b.title
ORDER BY chunk_count DESC;

-- Select-text usage
SELECT
    ROUND(100.0 * SUM(CASE WHEN mode = 'selected' THEN 1 ELSE 0 END) / COUNT(*), 2) as selected_mode_pct
FROM queries
WHERE created_at > NOW() - INTERVAL '7 days';
```

---

## Migration & Setup

### Initial Database Setup

**Neon PostgreSQL** (run once at startup):

```bash
# backend/scripts/setup_neon.py
python setup_neon.py --create-tables --create-indexes

# Output:
# Creating table: books
# Creating table: chunks_metadata
# Creating table: queries
# Creating table: sessions
# Creating indexes...
# ✓ All tables and indexes created successfully
```

**Qdrant Cloud** (run once at startup):

```bash
# backend/scripts/setup_qdrant.py
python setup_qdrant.py --create-collection

# Output:
# Creating collection: book_vectors (dim=1024, metric=cosine)
# ✓ Collection created successfully
```

---

## Summary Table

| Entity | Storage | Queryable | Searchable | Retention |
|--------|---------|-----------|-----------|-----------|
| Book | Neon | Yes | By title, status, hash | Indefinite |
| Chunk | Qdrant (vectors) + Neon (metadata) | Yes (metadata) | By text_hash, page, chapter | Indefinite |
| Query | Neon | Yes | By accuracy, latency, mode | 90 days |
| Session | Neon | Yes | By user, book, creation date | 30 days |

---

## Next Steps

1. **Create migration scripts**: `backend/scripts/setup_neon.py` and `backend/scripts/setup_qdrant.py`
2. **Implement database clients**: `backend/database/postgres_client.py` and `backend/database/qdrant_client.py`
3. **Add validation layer**: Pydantic models in `backend/app/api/models.py`
4. **Create monitoring queries**: Add to `backend/utils/metrics.py`
5. **Test re-indexing**: Unit tests in `backend/tests/unit/test_ingestion_main.py`
