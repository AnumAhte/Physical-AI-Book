---
id: 001
title: RAG Refactor Plan - Cohere + Qdrant
stage: plan
date: 2025-12-10
surface: agent
model: claude-opus-4-5-20251101
feature: rag-cohere-qdrant
branch: main
user: CBM
command: /sp.plan
labels: ["rag", "cohere", "qdrant", "refactor", "constitution"]
links:
  spec: specs/rag-cohere-qdrant/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/rag-cohere-qdrant/spec.md
  - specs/rag-cohere-qdrant/plan.md
  - specs/rag-cohere-qdrant/research.md
  - specs/rag-cohere-qdrant/data-model.md
  - specs/rag-cohere-qdrant/contracts/api.yaml
  - specs/rag-cohere-qdrant/quickstart.md
tests:
  - None (planning phase)
---

## Prompt

/sp.plan "refactor RAG system to use Cohere + Qdrant only. Remove all Neon/Postgres usage."

## Response snapshot

Created comprehensive implementation plan for RAG system refactor per Constitution Section VIII mandate:

**Phase 0 - Research (research.md):**
- 9 technology decisions documented
- Cohere embed-english-v3.0 (1024 dimensions)
- Cohere rerank-english-v3.0
- Qdrant Cloud single collection "textbook"
- 500 token chunks with 100 overlap
- No caching for MVP
- JSON logging to stdout (no SQL)

**Phase 1 - Design Artifacts:**
- data-model.md: Qdrant collection schema, Pydantic models (TextChunk, SearchResult, RerankedResult, AskRequest, RAGResponse, Citation)
- contracts/api.yaml: OpenAPI 3.1.0 spec with 3 endpoints (/api/ask, /api/ask-selected, /api/health)
- quickstart.md: Local development setup guide

**Architecture:**
- Two RAG flows: General question (full pipeline) and Select-text (bypasses Qdrant)
- Removed components: asyncpg, sqlalchemy, PostgreSQL/Neon code
- New components: cohere>=5.0.0, embeddings.py, rerank.py

## Outcome

- ✅ Impact: Complete design artifacts for RAG refactor aligned with constitution
- 🧪 Tests: None (planning phase - tests defined in plan for implementation)
- 📁 Files: 6 files created in specs/rag-cohere-qdrant/
- 🔁 Next prompts: /sp.tasks to generate implementation tasks, then /sp.implement
- 🧠 Reflection: Constitution Section VIII is critical - all SQL must be removed

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (planning phase)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks and validate against constitution checklist
