"""
Physical AI & Humanoid Robotics Textbook - RAG Chatbot Backend
Using: Qdrant (vector DB) + Cohere (embeddings) + Sitemap ingestion

Features:
- Auto-ingest from sitemap.xml
- Cohere embeddings (English)
- Qdrant vector database
- REST API for RAG queries
"""

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import httpx
import xml.etree.ElementTree as ET
from typing import List, Optional
import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import logging
from datetime import datetime
import uuid

# Load environment variables from .env file
load_dotenv()

# Configure logging
logging.basicConfig(level=os.getenv("LOG_LEVEL", "INFO"))
logger = logging.getLogger(__name__)

# Initialize FastAPI
app = FastAPI(
    title="Physical AI RAG Chatbot",
    version="2.0.0",
    description="RAG-powered chatbot using Qdrant + Cohere"
)

# CORS Configuration
cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================================================
# Configuration
# ============================================================================

SITEMAP_URL = os.getenv("SITEMAP_URL", "https://physical-ai-textbook-two.vercel.app/sitemap.xml")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION = "physical_ai_textbook"

if not COHERE_API_KEY:
    raise ValueError("COHERE_API_KEY environment variable is required")
if not QDRANT_URL:
    raise ValueError("QDRANT_URL environment variable is required")
if not QDRANT_API_KEY:
    raise ValueError("QDRANT_API_KEY environment variable is required")

# Initialize clients
cohere_client = cohere.ClientV2(api_key=COHERE_API_KEY)
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
)

# ============================================================================
# Data Models
# ============================================================================

class ChatRequest(BaseModel):
    question: str
    include_sources: bool = True

class Citation(BaseModel):
    text: str
    source: str
    url: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    citations: List[Citation]
    model: str = "cohere"
    timestamp: str

class HealthResponse(BaseModel):
    status: str
    version: str
    timestamp: str
    qdrant_collection: Optional[str] = None
    total_documents: int = 0

# ============================================================================
# Helper Functions
# ============================================================================

def fetch_urls_from_sitemap(sitemap_url: str) -> List[str]:
    """Fetch all URLs from sitemap.xml"""
    try:
        response = httpx.get(sitemap_url, timeout=10.0)
        response.raise_for_status()

        root = ET.fromstring(response.content)
        namespace = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

        urls = []
        for url_elem in root.findall('.//ns:loc', namespace):
            url = url_elem.text
            if url and url.endswith(('.html', '/')):
                urls.append(url)

        logger.info(f"Found {len(urls)} URLs in sitemap")
        return urls
    except Exception as e:
        logger.error(f"Error fetching sitemap: {e}")
        raise

def fetch_and_parse_page(url: str) -> Optional[str]:
    """Fetch page content from URL"""
    try:
        response = httpx.get(url, timeout=10.0)
        response.raise_for_status()

        # Extract text from HTML (simple approach)
        from html.parser import HTMLParser

        class TextExtractor(HTMLParser):
            def __init__(self):
                super().__init__()
                self.text = []

            def handle_data(self, data):
                if data.strip():
                    self.text.append(data.strip())

        extractor = TextExtractor()
        extractor.feed(response.text)
        return ' '.join(extractor.text)
    except Exception as e:
        logger.error(f"Error fetching page {url}: {e}")
        return None

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 100) -> List[str]:
    """Split text into overlapping chunks"""
    chunks = []
    words = text.split()

    for i in range(0, len(words), chunk_size - overlap):
        chunk = ' '.join(words[i:i + chunk_size])
        if chunk.strip():
            chunks.append(chunk)

    return chunks

def initialize_qdrant_collection():
    """Create Qdrant collection if it doesn't exist"""
    try:
        qdrant_client.recreate_collection(
            collection_name=QDRANT_COLLECTION,
            vectors_config=VectorParams(size=4096, distance=Distance.COSINE),
        )
        logger.info(f"Created Qdrant collection: {QDRANT_COLLECTION}")
    except Exception as e:
        logger.info(f"Collection may already exist: {e}")

def ingest_content():
    """Ingest content from sitemap into Qdrant"""
    try:
        logger.info("Starting content ingestion...")

        # Initialize collection
        initialize_qdrant_collection()

        # Fetch URLs from sitemap
        urls = fetch_urls_from_sitemap(SITEMAP_URL)

        total_chunks = 0
        points = []

        for url in urls:
            logger.info(f"Processing: {url}")
            content = fetch_and_parse_page(url)

            if not content:
                continue

            # Chunk content
            chunks = chunk_text(content)

            # Embed with Cohere
            for chunk in chunks:
                try:
                    # Embed using Cohere
                    response = cohere_client.embed(
                        model="embed-english-v3.0",
                        input_type="search_document",
                        texts=[chunk]
                    )

                    # Handle Cohere embedding response - response.embeddings is iterable with (type, vectors) tuples
                    embedding_tuple = list(response.embeddings)[0]
                    # embedding_tuple is ('float_', [[vector_values]])
                    embedding = embedding_tuple[1][0] if isinstance(embedding_tuple, tuple) else embedding_tuple
                    point_id = str(uuid.uuid4())

                    point = PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={
                            "text": chunk,
                            "source_url": url,
                            "chunk_index": len(points),
                            "timestamp": datetime.utcnow().isoformat()
                        }
                    )
                    points.append(point)
                    total_chunks += 1

                    # Batch upload every 100 chunks
                    if len(points) >= 100:
                        qdrant_client.upsert(
                            collection_name=QDRANT_COLLECTION,
                            points=points,
                            wait=True
                        )
                        logger.info(f"Uploaded {len(points)} chunks to Qdrant")
                        points = []

                except Exception as e:
                    logger.error(f"Error embedding chunk: {e}")
                    continue

        # Upload remaining points
        if points:
            qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION,
                points=points,
                wait=True
            )
            logger.info(f"Uploaded final {len(points)} chunks to Qdrant")

        logger.info(f"Ingestion complete! Total chunks: {total_chunks}")
        return total_chunks

    except Exception as e:
        logger.error(f"Ingestion error: {e}")
        raise

# ============================================================================
# Endpoints
# ============================================================================

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint"""
    try:
        collection_info = qdrant_client.get_collection(QDRANT_COLLECTION)
        total_docs = collection_info.points_count
    except:
        total_docs = 0

    return HealthResponse(
        status="ok",
        version="2.0.0",
        timestamp=datetime.utcnow().isoformat(),
        qdrant_collection=QDRANT_COLLECTION,
        total_documents=total_docs
    )

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    """RAG chatbot endpoint using Qdrant + Cohere"""
    try:
        # 1. Embed query with Cohere
        query_response = cohere_client.embed(
            model="embed-english-v3.0",
            input_type="search_query",
            texts=[request.question]
        )
        # Handle Cohere embedding response - response.embeddings is iterable with (type, vectors) tuples
        embedding_tuple = list(query_response.embeddings)[0]
        # embedding_tuple is ('float_', [[vector_values]])
        query_embedding = embedding_tuple[1][0] if isinstance(embedding_tuple, tuple) else embedding_tuple

        # 2. Search Qdrant for similar chunks
        search_results = qdrant_client.query_points(
            collection_name=QDRANT_COLLECTION,
            query=query_embedding,
            limit=5
        ).points

        # 3. Prepare context from search results
        context_chunks = []
        sources = []

        for result in search_results:
            chunk_text = result.payload.get("text", "")
            source_url = result.payload.get("source_url", "")

            context_chunks.append(chunk_text)
            if source_url not in [s["source"] for s in sources]:
                sources.append({
                    "text": chunk_text[:200],
                    "source": source_url.replace("https://physical-ai-textbook-two.vercel.app/", ""),
                    "url": source_url
                })

        # 4. Generate response with Cohere
        context = "\n\n".join(context_chunks)

        prompt = f"""Based on this textbook content, answer the question:

Content:
{context}

Question: {request.question}

Answer clearly and cite the source where information comes from."""

        response = cohere_client.generate(
            model="command-r-plus",
            prompt=prompt,
            max_tokens=1000,
            temperature=0.7
        )

        answer = response.generations[0].text.strip()

        # Create citations
        citations = []
        if request.include_sources:
            for source in sources[:3]:  # Top 3 sources
                citations.append(Citation(**source))

        return ChatResponse(
            answer=answer,
            citations=citations,
            timestamp=datetime.utcnow().isoformat()
        )

    except Exception as e:
        logger.error(f"Chat error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/reindex")
async def reindex(background_tasks: BackgroundTasks):
    """Re-ingest content from sitemap (background task)"""
    background_tasks.add_task(ingest_content)
    return {"status": "reindexing", "message": "Content ingestion started in background"}

@app.on_event("startup")
async def startup_event():
    """Initialize on startup"""
    try:
        # Check if collection exists, if not ingest
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if QDRANT_COLLECTION not in collection_names:
            logger.info("Collection not found, starting ingestion...")
            await ingest_content()
        else:
            logger.info(f"Collection {QDRANT_COLLECTION} found")
    except Exception as e:
        logger.error(f"Startup error: {e}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
