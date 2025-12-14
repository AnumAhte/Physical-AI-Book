---
id: 003
title: RAG Implementation - Cohere Qdrant Backend
stage: green
date: 2025-12-10
surface: agent
model: claude-opus-4-5-20251101
feature: rag-cohere-qdrant
branch: main
user: CBM
command: /sp.implement
labels: ["rag", "cohere", "qdrant", "fastapi", "implementation"]
links:
  spec: specs/rag-cohere-qdrant/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/requirements.txt
  - backend/pyproject.toml
  - backend/src/config.py
  - backend/src/models/__init__.py
  - backend/src/models/chunk.py
  - backend/src/models/search.py
  - backend/src/models/rerank.py
  - backend/src/models/requests.py
  - backend/src/models/responses.py
  - backend/src/middleware.py
  - backend/src/logger.py
  - backend/src/embeddings.py
  - backend/src/rerank.py
  - backend/src/vector_db.py
  - backend/src/chunker.py
  - backend/src/ingest.py
  - backend/src/llm.py
  - backend/src/rag_pipeline.py
  - backend/src/formatter.py
  - backend/src/api.py
  - backend/src/main.py
  - backend/.env.example
  - backend/tests/test_embeddings.py
  - backend/tests/test_rerank.py
  - backend/tests/test_vector_db.py
  - backend/tests/test_chunker.py
  - backend/tests/test_ingest.py
  - backend/tests/test_api.py
  - backend/tests/test_rag_pipeline.py
tests:
  - backend/tests/test_embeddings.py
  - backend/tests/test_rerank.py
  - backend/tests/test_vector_db.py
  - backend/tests/test_chunker.py
  - backend/tests/test_ingest.py
  - backend/tests/test_api.py
  - backend/tests/test_rag_pipeline.py
---

## Prompt

/sp.implement

## Response snapshot

Implemented complete RAG backend refactor per Constitution Section VIII:

**Phase 1: Setup (T001-T007)**
- Removed asyncpg, sqlalchemy from requirements.txt
- Added cohere>=5.0.0
- Verified no SQL imports remain

**Phase 2: Foundational (T008-T017)**
- Created config.py with Cohere, Qdrant, LLM settings
- Created Pydantic models: TextChunk, SearchResult, RerankedResult
- Created request/response models: AskRequest, RAGResponse, Citation
- Created middleware.py (CORS) and logger.py (JSON to stdout)

**Phase 3: US1 - Cohere Embeddings (T018-T024)**
- embeddings.py: embed_text(), embed_documents()
- 1024-dimensional vectors with input_type parameters
- Retry with exponential backoff for rate limits

**Phase 4: US2 - Cohere Rerank (T025-T030)**
- rerank.py: rerank_results(), rerank_text_chunks()
- Returns sorted results by relevance score

**Phase 5: US3 - Qdrant Vector Store (T031-T038)**
- vector_db.py: create_collection(), search_similar(), upsert_chunks()
- 1024 dimensions, Cosine distance
- Filtering by chapter/week

**Phase 6: US4 - Content Ingestion (T039-T047)**
- chunker.py: 500 tokens, 100 overlap per research.md
- ingest.py: CLI with --docs-path --recreate options

**Phase 7: US5 - /api/ask (T048-T058)**
- Full RAG pipeline: embed → search(10) → rerank(5) → generate
- llm.py with Anthropic/OpenAI support
- rag_pipeline.py: ask_question() orchestration

**Phase 8: US6 - /api/ask-selected (T059-T065)**
- Bypasses Qdrant per constitution
- Flow: chunk → rerank → generate

**Phase 9: US7 - /api/health (T066-T070)**
- Health checks for Qdrant and Cohere
- Returns status per contracts/api.yaml

**Phase 10: US8 - FastAPI App (T071-T077)**
- main.py with CORS, exception handlers
- .env.example with all required variables

**Deployment**: https://physical-ai-book-one.vercel.app/

## Outcome

- ✅ Impact: Complete RAG backend refactor - Constitution Section VIII compliant
- 🧪 Tests: 7 test files created covering all modules
- 📁 Files: 28 files created/modified
- 🔁 Next prompts: Deploy backend, run ingestion, test endpoints
- 🧠 Reflection: Clean implementation with proper separation of concerns

## Evaluation notes (flywheel)

- Failure modes observed: check-prerequisites.ps1 returned wrong feature directory
- Graders run and results (PASS/FAIL): Manual verification - SQL removed, Cohere+Qdrant implemented
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Deploy to Vercel and test with real API keys
