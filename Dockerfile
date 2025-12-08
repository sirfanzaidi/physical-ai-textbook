# Multi-stage Dockerfile to reduce image size
# Force rebuild: 2025-12-08 - Updated Qdrant + Cohere backend
FROM python:3.11-slim as builder

WORKDIR /app

# Install build dependencies only in builder stage
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements (production only, no dev deps)
COPY backend/requirements-prod.txt requirements.txt

# Install dependencies with wheel caching
RUN pip install --no-cache-dir --user \
    -r requirements.txt

# Final stage - minimal runtime image
FROM python:3.11-slim

WORKDIR /app

# Copy Python packages from builder
COPY --from=builder /root/.local /root/.local

# Set PATH to use local Python packages
ENV PATH=/root/.local/bin:$PATH
ENV PYTHONUNBUFFERED=1
ENV HOST=0.0.0.0
ENV PORT=8000
ENV CHROMADB_PATH=/tmp/embeddings
ENV CORS_ORIGINS=*

# Copy application code only (models will be downloaded at runtime)
COPY backend/ .

# Create data directory
RUN mkdir -p /tmp/embeddings

# Expose port
EXPOSE 8000

# Run the application
CMD ["python", "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
