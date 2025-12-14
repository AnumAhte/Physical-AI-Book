# Research: RAG System with Cohere + Qdrant

**Date**: 2025-12-10
**Feature**: rag-cohere-qdrant
**Purpose**: Resolve technical unknowns and document technology decisions

---

## Decision 1: Embedding Model

**Decision**: Use Cohere `embed-english-v3.0` with `input_type="search_document"` for indexing and `input_type="search_query"` for queries.

**Rationale**:
- Constitution Section VIII mandates Cohere embeddings
- embed-english-v3.0 is Cohere's latest English model
- 1024-dimensional vectors with excellent retrieval quality
- Free tier: 100 API calls/minute, sufficient for our scale

**Alternatives Considered**:
- ❌ sentence-transformers (local): Constitution prohibits
- ❌ OpenAI embeddings: Constitution prohibits for RAG
- ❌ Cohere embed-multilingual-v3.0: English-only textbook

**Implementation Notes**:
```python
import cohere

co = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

# For indexing documents
embeddings = co.embed(
    texts=chunks,
    model="embed-english-v3.0",
    input_type="search_document"
).embeddings

# For search queries
query_embedding = co.embed(
    texts=[query],
    model="embed-english-v3.0",
    input_type="search_query"
).embeddings[0]
```

---

## Decision 2: Reranking Strategy

**Decision**: Use Cohere `rerank-english-v3.0` to rerank top-k search results before LLM generation.

**Rationale**:
- Constitution requires Cohere rerank
- Significantly improves retrieval precision
- Reranking top 10 → 5 is optimal cost/quality tradeoff
- Free tier: 100 rerank calls/minute

**Alternatives Considered**:
- ❌ No reranking: Lower quality results
- ❌ LLM-based reranking: Too expensive, too slow
- ❌ Custom reranker: Constitution mandates Cohere

**Implementation Notes**:
```python
# Rerank search results
reranked = co.rerank(
    model="rerank-english-v3.0",
    query=user_query,
    documents=[doc.text for doc in search_results],
    top_n=5
)

# Use reranked.results for LLM context
```

---

## Decision 3: Qdrant Collection Schema

**Decision**: Single collection "textbook" with payload containing chapter metadata.

**Rationale**:
- Simple schema for single-textbook use case
- Payload enables filtering by chapter/week if needed
- 1024 dimensions matches Cohere embed-english-v3.0

**Alternatives Considered**:
- ❌ Multiple collections per chapter: Unnecessary complexity
- ❌ Sparse vectors: Not needed for textbook search

**Schema**:
```json
{
  "collection_name": "textbook",
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  },
  "payload_schema": {
    "chunk_id": "string",
    "chapter": "string",
    "week": "integer",
    "content": "string",
    "source_file": "string"
  }
}
```

---

## Decision 4: Chunk Strategy

**Decision**: 500 tokens per chunk with 100 token overlap, using langchain RecursiveCharacterTextSplitter.

**Rationale**:
- 500 tokens fits well in LLM context windows
- Overlap preserves context at boundaries
- RecursiveCharacterTextSplitter handles markdown well

**Alternatives Considered**:
- ❌ 1000 tokens: Too large, reduces retrieval precision
- ❌ 200 tokens: Too small, loses context
- ❌ Sentence-based: Inconsistent chunk sizes

**Implementation Notes**:
```python
from langchain_text_splitters import RecursiveCharacterTextSplitter

splitter = RecursiveCharacterTextSplitter(
    chunk_size=500,
    chunk_overlap=100,
    separators=["\n## ", "\n### ", "\n\n", "\n", " "]
)
```

---

## Decision 5: Select-Text Flow

**Decision**: Selected text bypasses Qdrant search entirely. Use Cohere rerank to compare question with selected text chunks.

**Rationale**:
- Constitution specifies "Highlighted text must bypass Qdrant"
- User already selected relevant content
- Rerank confirms relevance before LLM generation

**Flow**:
```text
1. User highlights text on page
2. Frontend sends: { question, selectedText }
3. Backend splits selectedText into chunks
4. Cohere rerank: question vs chunks
5. Top chunk(s) sent to LLM
6. Response generated from selected content only
```

---

## Decision 6: LLM for Response Generation

**Decision**: Support both Anthropic Claude and OpenAI GPT-4 via environment configuration.

**Rationale**:
- Constitution allows both (OpenAI for generation, not embeddings)
- Claude aligns with Claude Code integration
- Flexibility for cost/performance tradeoffs

**Configuration**:
```python
LLM_PROVIDER = os.getenv("LLM_PROVIDER", "anthropic")  # or "openai"
```

---

## Decision 7: Error Handling Strategy

**Decision**: Graceful degradation with user-friendly error messages.

**Error Cases**:
| Error | Handling |
|-------|----------|
| Cohere rate limit | Retry with exponential backoff (3x) |
| Qdrant connection | Return 503 with retry message |
| No relevant results | Return message asking for rephrasing |
| LLM generation fail | Return partial response if rerank succeeded |

---

## Decision 8: Caching Strategy

**Decision**: No caching for MVP. Add Redis caching in future iteration if needed.

**Rationale**:
- Textbook content is static, but queries vary
- Cohere free tier is sufficient for expected load
- Adding cache would violate minimalist principle for MVP

---

## Decision 9: Logging Without SQL

**Decision**: Use structured JSON logging to stdout/stderr, no persistent storage.

**Rationale**:
- Constitution prohibits SQL
- Vercel/serverless platforms capture stdout logs
- Can add log aggregation service (Axiom, Logtail) later

**Implementation**:
```python
import logging
import json

class JSONFormatter(logging.Formatter):
    def format(self, record):
        return json.dumps({
            "timestamp": self.formatTime(record),
            "level": record.levelname,
            "message": record.getMessage(),
            "extra": getattr(record, "extra", {})
        })
```

---

## Summary of Key Decisions

| Area | Decision |
|------|----------|
| Embeddings | Cohere embed-english-v3.0 |
| Reranking | Cohere rerank-english-v3.0 |
| Vector DB | Qdrant Cloud (single "textbook" collection) |
| Chunking | 500 tokens, 100 overlap |
| Select-Text | Bypass Qdrant, rerank only |
| LLM | Anthropic Claude (default) or OpenAI GPT-4 |
| Logging | JSON to stdout (no SQL) |
| Caching | None for MVP |

---

*Research completed for /sp.plan Phase 0*
