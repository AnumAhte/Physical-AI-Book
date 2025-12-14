# Data Model: RAG System (Cohere + Qdrant)

**Date**: 2025-12-10
**Feature**: rag-cohere-qdrant
**Storage**: Qdrant Cloud (vector database only, NO SQL)

---

## Qdrant Collection: `textbook`

### Vector Configuration

```json
{
  "collection_name": "textbook",
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  },
  "on_disk_payload": true,
  "optimizers_config": {
    "indexing_threshold": 20000
  }
}
```

### Point Structure

Each point in the collection represents a text chunk:

```json
{
  "id": "uuid-v4",
  "vector": [0.123, -0.456, ...],  // 1024 dimensions from Cohere
  "payload": {
    "chunk_id": "week-01-02-intro_intro_chunk_001",
    "content": "Physical AI refers to artificial intelligence systems that interact...",
    "chapter": "intro",
    "chapter_title": "Introduction to Physical AI",
    "week": 1,
    "source_file": "docs/week-01-02-intro/intro.md",
    "heading": "What is Physical AI?",
    "char_start": 0,
    "char_end": 500,
    "created_at": "2025-12-10T00:00:00Z"
  }
}
```

### Payload Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `chunk_id` | string | Yes | Unique identifier for the chunk |
| `content` | string | Yes | The actual text content (for display) |
| `chapter` | string | Yes | Chapter slug (e.g., "intro", "ros2-overview") |
| `chapter_title` | string | Yes | Human-readable chapter title |
| `week` | integer | Yes | Course week number (1-13) |
| `source_file` | string | Yes | Path to source markdown file |
| `heading` | string | No | Nearest heading above the chunk |
| `char_start` | integer | No | Character offset in source file |
| `char_end` | integer | No | End character offset |
| `created_at` | string | Yes | ISO 8601 timestamp |

---

## Pydantic Models (Python)

### Chunk Model

```python
from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional
import uuid

class TextChunk(BaseModel):
    """A chunk of text from the textbook."""

    chunk_id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    content: str = Field(..., min_length=1, max_length=5000)
    chapter: str = Field(..., pattern=r"^[a-z0-9-]+$")
    chapter_title: str
    week: int = Field(..., ge=1, le=13)
    source_file: str
    heading: Optional[str] = None
    char_start: Optional[int] = None
    char_end: Optional[int] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "intro_chunk_001",
                "content": "Physical AI refers to artificial intelligence...",
                "chapter": "intro",
                "chapter_title": "Introduction to Physical AI",
                "week": 1,
                "source_file": "docs/week-01-02-intro/intro.md",
                "heading": "What is Physical AI?",
                "char_start": 0,
                "char_end": 500
            }
        }
```

### Search Result Model

```python
class SearchResult(BaseModel):
    """A search result from Qdrant."""

    chunk_id: str
    content: str
    chapter: str
    chapter_title: str
    week: int
    source_file: str
    heading: Optional[str] = None
    score: float = Field(..., ge=0.0, le=1.0)

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "intro_chunk_001",
                "content": "Physical AI refers to...",
                "chapter": "intro",
                "chapter_title": "Introduction to Physical AI",
                "week": 1,
                "source_file": "docs/week-01-02-intro/intro.md",
                "heading": "What is Physical AI?",
                "score": 0.89
            }
        }
```

### Reranked Result Model

```python
class RerankedResult(BaseModel):
    """A reranked search result from Cohere."""

    chunk_id: str
    content: str
    chapter: str
    source_file: str
    relevance_score: float = Field(..., ge=0.0, le=1.0)

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "intro_chunk_001",
                "content": "Physical AI refers to...",
                "chapter": "intro",
                "source_file": "docs/week-01-02-intro/intro.md",
                "relevance_score": 0.95
            }
        }
```

---

## API Request/Response Models

### Ask Request

```python
class AskRequest(BaseModel):
    """Request body for /api/ask endpoint."""

    question: str = Field(..., min_length=3, max_length=1000)
    top_k: int = Field(default=5, ge=1, le=20)

    class Config:
        json_schema_extra = {
            "example": {
                "question": "What is Physical AI?",
                "top_k": 5
            }
        }
```

### Ask Selected Request

```python
class AskSelectedRequest(BaseModel):
    """Request body for /api/ask-selected endpoint."""

    question: str = Field(..., min_length=3, max_length=1000)
    selected_text: str = Field(..., min_length=10, max_length=10000)

    class Config:
        json_schema_extra = {
            "example": {
                "question": "Explain this concept in simpler terms",
                "selected_text": "Physical AI refers to artificial intelligence systems that interact with the physical world through embodied agents..."
            }
        }
```

### RAG Response

```python
class Citation(BaseModel):
    """A citation reference in the response."""

    chapter: str
    chapter_title: str
    source_file: str
    excerpt: str = Field(..., max_length=200)

class RAGResponse(BaseModel):
    """Response from RAG endpoints."""

    answer: str
    citations: list[Citation]
    confidence: float = Field(..., ge=0.0, le=1.0)

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "Physical AI refers to AI systems that can perceive and interact with the real world through sensors and actuators...",
                "citations": [
                    {
                        "chapter": "intro",
                        "chapter_title": "Introduction to Physical AI",
                        "source_file": "docs/week-01-02-intro/intro.md",
                        "excerpt": "Physical AI refers to artificial intelligence systems that..."
                    }
                ],
                "confidence": 0.92
            }
        }
```

---

## Indexing Schema

### Collection Creation

```python
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

def create_collection(client: QdrantClient):
    """Create the textbook collection if it doesn't exist."""

    client.recreate_collection(
        collection_name="textbook",
        vectors_config=VectorParams(
            size=1024,  # Cohere embed-english-v3.0
            distance=Distance.COSINE
        )
    )
```

### Point Insertion

```python
from qdrant_client.models import PointStruct

def insert_chunks(client: QdrantClient, chunks: list[TextChunk], embeddings: list[list[float]]):
    """Insert chunks with their embeddings into Qdrant."""

    points = [
        PointStruct(
            id=chunk.chunk_id,
            vector=embedding,
            payload=chunk.model_dump()
        )
        for chunk, embedding in zip(chunks, embeddings)
    ]

    client.upsert(
        collection_name="textbook",
        points=points
    )
```

---

## Query Patterns

### Semantic Search

```python
def search_similar(client: QdrantClient, query_embedding: list[float], top_k: int = 10):
    """Search for similar chunks."""

    results = client.search(
        collection_name="textbook",
        query_vector=query_embedding,
        limit=top_k,
        with_payload=True
    )

    return [
        SearchResult(
            chunk_id=r.id,
            content=r.payload["content"],
            chapter=r.payload["chapter"],
            chapter_title=r.payload["chapter_title"],
            week=r.payload["week"],
            source_file=r.payload["source_file"],
            heading=r.payload.get("heading"),
            score=r.score
        )
        for r in results
    ]
```

### Filter by Week

```python
from qdrant_client.models import Filter, FieldCondition, Range

def search_by_week(client: QdrantClient, query_embedding: list[float], week: int):
    """Search within a specific week."""

    return client.search(
        collection_name="textbook",
        query_vector=query_embedding,
        query_filter=Filter(
            must=[
                FieldCondition(
                    key="week",
                    range=Range(gte=week, lte=week)
                )
            ]
        ),
        limit=10
    )
```

---

## Data Validation Rules

1. **Content Length**: 10 - 5000 characters per chunk
2. **Chapter Slug**: Lowercase alphanumeric with hyphens only
3. **Week Range**: 1 - 13 (matching course structure)
4. **Vector Dimension**: Exactly 1024 (Cohere embed-english-v3.0)
5. **Unique IDs**: UUID v4 for all chunk_ids

---

*Data model generated for /sp.plan Phase 1*
