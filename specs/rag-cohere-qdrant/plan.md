# Implementation Plan: RAG System Refactor (Cohere + Qdrant)

**Branch**: `main` | **Date**: 2025-12-10 | **Spec**: [specs/rag-cohere-qdrant/spec.md](./spec.md)
**Input**: Feature specification + Constitution Section VIII mandate

## Summary

Refactor the RAG backend to use Cohere for embeddings/reranking and Qdrant as the sole storage solution, completely removing PostgreSQL/Neon dependencies. This is a MANDATORY change per the project constitution.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:
- FastAPI (web framework)
- cohere (embeddings + rerank)
- qdrant-client (vector storage)
- anthropic OR openai (response generation)

**Storage**: Qdrant Cloud (free tier) - NO SQL ALLOWED
**Testing**: pytest, pytest-asyncio, httpx
**Target Platform**: Linux server / Vercel serverless
**Project Type**: Web application (backend API)
**Performance Goals**: < 3s response time for RAG queries
**Constraints**: Free-tier only, zero hallucination, textbook-only answers
**Scale/Scope**: Single textbook (~50 chapters, ~500KB content)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Simplicity | PASS | Removing SQL simplifies architecture |
| II. Accuracy | PASS | RAG from textbook content only |
| III. Minimalist | PASS | Cohere + Qdrant only, no SQL |
| IV. SDD | PASS | Following spec-driven workflow |
| V. RAG Integrity | PASS | Answers only from textbook |
| VI. Modular | PASS | Clear separation: embed/search/rerank/generate |
| VII. CI/CD Ready | PASS | Stateless API, environment config |
| **VIII. Storage** | **MANDATORY** | Must use Cohere + Qdrant ONLY |

### Section VIII Compliance Checklist

- [ ] Remove asyncpg dependency
- [ ] Remove sqlalchemy dependency
- [ ] Remove all PostgreSQL/Neon code
- [ ] Implement Cohere embeddings (embed-english-v3.0)
- [ ] Implement Cohere rerank
- [ ] All embeddings stored in Qdrant only
- [ ] Select-text bypasses Qdrant, uses Cohere rerank only

## Project Structure

### Documentation (this feature)

```text
specs/rag-cohere-qdrant/
├── plan.md              # This file
├── research.md          # Technology decisions
├── data-model.md        # Qdrant collection schema
├── quickstart.md        # Local development setup
└── contracts/
    └── api.yaml         # OpenAPI specification
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── __init__.py
│   ├── main.py              # FastAPI app entry point
│   ├── config.py            # Environment configuration
│   ├── middleware.py        # CORS middleware
│   ├── api.py               # API routes
│   ├── embeddings.py        # Cohere embeddings (NEW)
│   ├── rerank.py            # Cohere rerank (NEW)
│   ├── vector_db.py         # Qdrant client (REFACTOR)
│   ├── rag_pipeline.py      # RAG orchestration (REFACTOR)
│   ├── chunker.py           # Text chunking
│   ├── ingest.py            # Content ingestion script
│   ├── llm.py               # LLM response generation
│   └── formatter.py         # Response formatting
├── tests/
│   ├── test_api.py
│   ├── test_embeddings.py
│   ├── test_rerank.py
│   └── test_rag_pipeline.py
├── requirements.txt         # UPDATED - remove SQL deps
└── pyproject.toml          # UPDATED - remove SQL deps
```

**Structure Decision**: Web application with backend/ directory. No frontend changes needed - Docusaurus frontend already exists.

## Removed Components

The following will be REMOVED from the codebase:

```text
REMOVE FROM requirements.txt:
- asyncpg>=0.29.0
- sqlalchemy[asyncio]>=2.0.25

REMOVE FROM .env.example:
- NEON_DATABASE_URL

REMOVE FILES (if they exist):
- backend/src/logger.py (if it uses PostgreSQL)
- Any SQL migration files

REMOVE CODE:
- All SQLAlchemy models
- All asyncpg connections
- All database session management
```

## New Components

```text
ADD TO requirements.txt:
+ cohere>=5.0.0

ADD TO .env.example:
+ COHERE_API_KEY=your-cohere-api-key

NEW FILES:
+ backend/src/embeddings.py   # Cohere embed client
+ backend/src/rerank.py       # Cohere rerank client
```

## Architecture Diagram

```mermaid
graph TB
    subgraph "Frontend (Docusaurus)"
        A[User Query] --> B[Ask AI Button]
        C[Selected Text] --> D[Ask Selected Button]
    end

    subgraph "Backend (FastAPI)"
        B --> E[/api/ask]
        D --> F[/api/ask-selected]

        E --> G[Cohere Embed]
        G --> H[Qdrant Search]
        H --> I[Cohere Rerank]

        F --> I

        I --> J[LLM Generate]
        J --> K[Response + Citations]
    end

    subgraph "External Services"
        G -.-> L[Cohere API]
        I -.-> L
        H -.-> M[Qdrant Cloud]
        J -.-> N[Anthropic/OpenAI]
    end
```

## RAG Pipeline Flows

### Flow 1: General Question (`/api/ask`)

```text
1. User submits question
2. Cohere embeds question (embed-english-v3.0)
3. Qdrant vector search (top 10 chunks)
4. Cohere rerank results (top 5)
5. LLM generates answer with citations
6. Return response with source references
```

### Flow 2: Selected Text (`/api/ask-selected`)

```text
1. User highlights text + asks question
2. Skip Qdrant search (text already selected)
3. Cohere rerank: question vs. selected text chunks
4. LLM generates answer about highlighted content
5. Return response
```

### Flow 3: Ingestion (Offline)

```text
1. Load markdown chapters from /docs
2. Split into chunks (500 tokens, 100 overlap)
3. Cohere embed each chunk
4. Store in Qdrant collection "textbook"
5. Log ingestion metrics
```

## Complexity Tracking

No complexity violations. This refactor REDUCES complexity by:
- Removing SQL dependencies
- Using managed services (Cohere, Qdrant Cloud)
- Simplifying the data layer to vectors only

## Next Steps

1. **Phase 0**: Generate `research.md` with Cohere/Qdrant best practices
2. **Phase 1**: Generate `data-model.md`, `contracts/api.yaml`, `quickstart.md`
3. **Phase 2**: Generate `tasks.md` via `/sp.tasks` command

---

*Plan generated by /sp.plan command*
