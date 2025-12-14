# Feature: RAG System Refactor - Cohere + Qdrant Only

## Objective

Refactor the RAG (Retrieval-Augmented Generation) backend system to use Cohere + Qdrant exclusively, removing all Neon/PostgreSQL dependencies. This aligns with the project constitution's Section VIII mandate.

## Background

The current backend implementation includes:
- asyncpg and sqlalchemy for PostgreSQL/Neon (PROHIBITED)
- sentence-transformers for embeddings (TO BE REPLACED with Cohere)
- qdrant-client for vector storage (KEEP)

## Requirements

### Must Have
1. **Cohere Embeddings**: Replace sentence-transformers with Cohere embed-english-v3.0
2. **Cohere Rerank**: Implement reranking for improved retrieval accuracy
3. **Qdrant Only**: All data storage in Qdrant Cloud (free tier)
4. **No SQL**: Remove asyncpg, sqlalchemy, and all PostgreSQL code
5. **FastAPI Backend**: Maintain FastAPI as the web framework

### RAG Pipeline (per Constitution)
1. **Ingestion**: Chunk textbook → Cohere Embeddings → Qdrant
2. **Query**: User query → Cohere Embed → Qdrant search → Cohere Rerank → OpenAI/Claude Response
3. **Select-Text**: Highlighted text bypasses Qdrant, direct to Cohere rerank

### API Endpoints
- `POST /api/ask` - General question from textbook
- `POST /api/ask-selected` - Question about selected/highlighted text
- `GET /api/health` - Health check endpoint

### Constraints
- Free-tier services only
- Zero hallucination - answers only from textbook content
- Response time < 3 seconds

## Success Criteria
- All PostgreSQL/Neon code removed
- Cohere embeddings working for ingestion
- Cohere rerank improving retrieval quality
- RAG responses accurately cite textbook content
- All tests pass
