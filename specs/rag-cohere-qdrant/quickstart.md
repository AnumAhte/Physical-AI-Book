# Quickstart: RAG System (Cohere + Qdrant)

**Date**: 2025-12-10
**Feature**: rag-cohere-qdrant

---

## Prerequisites

- Python 3.10+
- Cohere API key (free tier)
- Qdrant Cloud account (free tier)

---

## 1. Get API Keys

### Cohere API Key

1. Go to [dashboard.cohere.com](https://dashboard.cohere.com)
2. Sign up or log in
3. Navigate to API Keys
4. Copy your API key

### Qdrant Cloud

1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create a free cluster
3. Copy your cluster URL and API key

---

## 2. Environment Setup

Create a `.env` file in the `backend/` directory:

```bash
# Cohere (REQUIRED)
COHERE_API_KEY=your-cohere-api-key

# Qdrant Cloud (REQUIRED)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# LLM Provider (choose one)
LLM_PROVIDER=anthropic  # or "openai"
ANTHROPIC_API_KEY=your-anthropic-key
# OR
OPENAI_API_KEY=your-openai-key

# Optional
LOG_LEVEL=INFO
```

---

## 3. Install Dependencies

```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate (Windows)
.\venv\Scripts\activate

# Activate (Linux/Mac)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### Required packages (requirements.txt)

```text
fastapi>=0.109.0
uvicorn>=0.27.0
cohere>=5.0.0
qdrant-client>=1.7.0
anthropic>=0.18.0
openai>=1.12.0
pydantic>=2.5.0
python-dotenv>=1.0.0
httpx>=0.26.0
langchain-text-splitters>=0.0.1
```

---

## 4. Create Qdrant Collection

Run this script once to create the collection:

```python
# scripts/create_collection.py
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
import os
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

client.recreate_collection(
    collection_name="textbook",
    vectors_config=VectorParams(
        size=1024,  # Cohere embed-english-v3.0
        distance=Distance.COSINE
    )
)

print("Collection 'textbook' created successfully!")
```

Run:

```bash
python scripts/create_collection.py
```

---

## 5. Ingest Textbook Content

```bash
python -m src.ingest --docs-path ../docs
```

This will:
1. Read all markdown files from `/docs`
2. Split into chunks (500 tokens, 100 overlap)
3. Generate Cohere embeddings
4. Store in Qdrant

---

## 6. Start the API Server

### Development

```bash
uvicorn src.main:app --reload --port 8000
```

### Production

```bash
uvicorn src.main:app --host 0.0.0.0 --port 8000
```

---

## 7. Test the API

### Health Check

```bash
curl http://localhost:8000/api/health
```

Expected response:

```json
{
  "status": "healthy",
  "services": {
    "qdrant": "connected",
    "cohere": "connected"
  },
  "version": "1.0.0"
}
```

### Ask a Question

```bash
curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?", "top_k": 5}'
```

Expected response:

```json
{
  "answer": "Physical AI refers to artificial intelligence systems that interact with the physical world...",
  "citations": [
    {
      "chapter": "intro",
      "chapter_title": "Introduction to Physical AI",
      "source_file": "docs/week-01-02-intro/intro.md",
      "excerpt": "Physical AI refers to..."
    }
  ],
  "confidence": 0.92
}
```

### Ask About Selected Text

```bash
curl -X POST http://localhost:8000/api/ask-selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this in simpler terms",
    "selected_text": "ROS 2 nodes communicate using a publish-subscribe pattern..."
  }'
```

---

## 8. Verify No SQL Dependencies

Confirm that there are NO SQL-related imports in the codebase:

```bash
# Should return nothing
grep -r "asyncpg\|sqlalchemy\|postgresql\|neon" backend/src/
```

---

## Troubleshooting

### "Cohere API rate limit exceeded"

- Free tier: 100 calls/minute
- Solution: Add retry logic with exponential backoff

### "Qdrant connection failed"

- Check QDRANT_URL format (must include https://)
- Verify QDRANT_API_KEY is correct
- Ensure cluster is running in Qdrant Cloud dashboard

### "No relevant results found"

- Verify textbook content was ingested
- Check collection exists: `client.get_collection("textbook")`
- Try rephrasing the question

---

## Project Structure

```text
backend/
├── src/
│   ├── main.py          # FastAPI app
│   ├── config.py        # Environment config
│   ├── api.py           # API routes
│   ├── embeddings.py    # Cohere embed client
│   ├── rerank.py        # Cohere rerank client
│   ├── vector_db.py     # Qdrant client
│   ├── rag_pipeline.py  # RAG orchestration
│   ├── chunker.py       # Text chunking
│   ├── ingest.py        # Ingestion script
│   └── llm.py           # LLM generation
├── tests/
├── requirements.txt
└── .env
```

---

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Implement each module following the task list
3. Run tests: `pytest tests/`
4. Deploy to Vercel

---

*Quickstart generated for /sp.plan Phase 1*
