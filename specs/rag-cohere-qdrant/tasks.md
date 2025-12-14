# Tasks: RAG System Refactor (Cohere + Qdrant)

**Input**: Design documents from `/specs/rag-cohere-qdrant/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/api.yaml

**Tests**: Tests included as this is a critical refactor with constitution compliance requirements.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/` for Python backend
- Tests in `backend/tests/`

---

## Phase 1: Setup (Remove SQL Dependencies)

**Purpose**: Remove prohibited SQL dependencies and prepare clean slate per Constitution Section VIII

- [x] T001 Remove asyncpg from backend/requirements.txt
- [x] T002 Remove sqlalchemy from backend/requirements.txt
- [x] T003 [P] Remove NEON_DATABASE_URL from backend/.env.example
- [x] T004 [P] Add cohere>=5.0.0 to backend/requirements.txt
- [x] T005 Delete any SQL migration files in backend/ (if present)
- [x] T006 Delete backend/src/database.py (if exists - SQL connection code)
- [x] T007 Verify no SQL imports remain: grep for asyncpg, sqlalchemy, postgresql

**Checkpoint**: All SQL dependencies removed - Constitution Section VIII partial compliance

---

## Phase 2: Foundational (Core Infrastructure)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T008 Create backend/src/config.py with environment configuration (Cohere, Qdrant, LLM provider)
- [x] T009 [P] Create backend/src/models/__init__.py for Pydantic models package
- [x] T010 [P] Create TextChunk model in backend/src/models/chunk.py per data-model.md
- [x] T011 [P] Create SearchResult model in backend/src/models/search.py per data-model.md
- [x] T012 [P] Create RerankedResult model in backend/src/models/rerank.py per data-model.md
- [x] T013 [P] Create AskRequest and AskSelectedRequest models in backend/src/models/requests.py
- [x] T014 [P] Create RAGResponse and Citation models in backend/src/models/responses.py
- [x] T015 [P] Create ErrorResponse and HealthResponse models in backend/src/models/responses.py
- [x] T016 Create backend/src/middleware.py with CORS configuration
- [x] T017 Setup JSON logging in backend/src/logger.py (no SQL, stdout only per research.md)

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Cohere Embeddings (Priority: P1) 🎯 MVP

**Goal**: Replace sentence-transformers with Cohere embed-english-v3.0 (1024 dimensions)

**Independent Test**: Can embed a text string and receive 1024-dimensional vector

### Tests for User Story 1

- [x] T018 [P] [US1] Create test for embedding generation in backend/tests/test_embeddings.py
- [x] T019 [P] [US1] Create test for batch embedding in backend/tests/test_embeddings.py

### Implementation for User Story 1

- [x] T020 [US1] Create Cohere client initialization in backend/src/embeddings.py
- [x] T021 [US1] Implement embed_text() function with input_type="search_query" in backend/src/embeddings.py
- [x] T022 [US1] Implement embed_documents() function with input_type="search_document" in backend/src/embeddings.py
- [x] T023 [US1] Add error handling for Cohere rate limits (retry with backoff) in backend/src/embeddings.py
- [x] T024 [US1] Add logging for embedding operations in backend/src/embeddings.py

**Checkpoint**: Cohere embeddings working - can embed queries and documents

---

## Phase 4: User Story 2 - Cohere Rerank (Priority: P1)

**Goal**: Implement Cohere rerank-english-v3.0 for improved retrieval accuracy

**Independent Test**: Can rerank a list of documents against a query and return sorted results

### Tests for User Story 2

- [x] T025 [P] [US2] Create test for reranking in backend/tests/test_rerank.py
- [x] T026 [P] [US2] Create test for rerank with top_n parameter in backend/tests/test_rerank.py

### Implementation for User Story 2

- [x] T027 [US2] Create Cohere rerank client in backend/src/rerank.py
- [x] T028 [US2] Implement rerank_results() function in backend/src/rerank.py
- [x] T029 [US2] Add error handling for Cohere rate limits in backend/src/rerank.py
- [x] T030 [US2] Add logging for rerank operations in backend/src/rerank.py

**Checkpoint**: Cohere rerank working - can rerank search results

---

## Phase 5: User Story 3 - Qdrant Vector Store (Priority: P1)

**Goal**: Refactor Qdrant client to work with Cohere embeddings (1024 dimensions)

**Independent Test**: Can store and retrieve vectors from Qdrant collection

### Tests for User Story 3

- [x] T031 [P] [US3] Create test for Qdrant connection in backend/tests/test_vector_db.py
- [x] T032 [P] [US3] Create test for vector search in backend/tests/test_vector_db.py

### Implementation for User Story 3

- [x] T033 [US3] Refactor Qdrant client initialization in backend/src/vector_db.py
- [x] T034 [US3] Implement create_collection() for "textbook" collection (1024 dims, Cosine) in backend/src/vector_db.py
- [x] T035 [US3] Implement search_similar() function in backend/src/vector_db.py
- [x] T036 [US3] Implement upsert_chunks() function in backend/src/vector_db.py
- [x] T037 [US3] Add filtering by chapter/week in backend/src/vector_db.py
- [x] T038 [US3] Add error handling for Qdrant connection failures in backend/src/vector_db.py

**Checkpoint**: Qdrant vector store working with Cohere embeddings

---

## Phase 6: User Story 4 - Content Ingestion (Priority: P2)

**Goal**: Ingest textbook markdown content into Qdrant via Cohere embeddings

**Independent Test**: Can chunk markdown file and store in Qdrant

### Tests for User Story 4

- [x] T039 [P] [US4] Create test for text chunking in backend/tests/test_chunker.py
- [x] T040 [P] [US4] Create test for ingestion pipeline in backend/tests/test_ingest.py

### Implementation for User Story 4

- [x] T041 [US4] Implement markdown chunking in backend/src/chunker.py (500 tokens, 100 overlap)
- [x] T042 [US4] Extract chapter metadata (title, week, heading) in backend/src/chunker.py
- [x] T043 [US4] Create ingestion script in backend/src/ingest.py
- [x] T044 [US4] Implement batch embedding for chunks in backend/src/ingest.py
- [x] T045 [US4] Implement batch upsert to Qdrant in backend/src/ingest.py
- [x] T046 [US4] Add progress logging for ingestion in backend/src/ingest.py
- [x] T047 [US4] Create CLI entry point: python -m src.ingest --docs-path in backend/src/ingest.py

**Checkpoint**: Can ingest textbook content into Qdrant

---

## Phase 7: User Story 5 - General Question Flow /api/ask (Priority: P1) 🎯 MVP

**Goal**: Implement full RAG pipeline: Embed → Search → Rerank → Generate

**Independent Test**: Can ask "What is Physical AI?" and get answer with citations

### Tests for User Story 5

- [x] T048 [P] [US5] Create contract test for POST /api/ask in backend/tests/test_api.py
- [x] T049 [P] [US5] Create test for RAG pipeline in backend/tests/test_rag_pipeline.py

### Implementation for User Story 5

- [x] T050 [US5] Create LLM client abstraction in backend/src/llm.py (Anthropic/OpenAI)
- [x] T051 [US5] Implement generate_response() with citations in backend/src/llm.py
- [x] T052 [US5] Create RAG pipeline orchestration in backend/src/rag_pipeline.py
- [x] T053 [US5] Implement ask_question() flow: embed → search(10) → rerank(5) → generate in backend/src/rag_pipeline.py
- [x] T054 [US5] Add confidence score calculation in backend/src/rag_pipeline.py
- [x] T055 [US5] Create response formatter in backend/src/formatter.py
- [x] T056 [US5] Implement POST /api/ask endpoint in backend/src/api.py
- [x] T057 [US5] Add request validation per contracts/api.yaml in backend/src/api.py
- [x] T058 [US5] Add error responses (400, 429, 503) per contracts/api.yaml in backend/src/api.py

**Checkpoint**: /api/ask endpoint working - full RAG pipeline functional

---

## Phase 8: User Story 6 - Selected Text Flow /api/ask-selected (Priority: P2)

**Goal**: Implement selected text flow that bypasses Qdrant per constitution

**Independent Test**: Can ask question about highlighted text without Qdrant search

### Tests for User Story 6

- [x] T059 [P] [US6] Create contract test for POST /api/ask-selected in backend/tests/test_api.py
- [x] T060 [P] [US6] Create test for selected text pipeline in backend/tests/test_rag_pipeline.py

### Implementation for User Story 6

- [x] T061 [US6] Implement split_selected_text() to chunk user selection in backend/src/rag_pipeline.py
- [x] T062 [US6] Implement ask_selected() flow: chunk → rerank → generate in backend/src/rag_pipeline.py
- [x] T063 [US6] Verify NO Qdrant search in selected text flow in backend/src/rag_pipeline.py
- [x] T064 [US6] Implement POST /api/ask-selected endpoint in backend/src/api.py
- [x] T065 [US6] Add request validation (min 10 chars selected) in backend/src/api.py

**Checkpoint**: /api/ask-selected endpoint working - bypasses Qdrant correctly

---

## Phase 9: User Story 7 - Health Check /api/health (Priority: P2)

**Goal**: Implement health check endpoint for monitoring

**Independent Test**: Can hit /api/health and see service status

### Tests for User Story 7

- [x] T066 [P] [US7] Create contract test for GET /api/health in backend/tests/test_api.py

### Implementation for User Story 7

- [x] T067 [US7] Implement Qdrant health check in backend/src/vector_db.py
- [x] T068 [US7] Implement Cohere health check in backend/src/embeddings.py
- [x] T069 [US7] Implement GET /api/health endpoint in backend/src/api.py
- [x] T070 [US7] Return service status per contracts/api.yaml schema in backend/src/api.py

**Checkpoint**: /api/health endpoint working - returns service status

---

## Phase 10: User Story 8 - FastAPI Application (Priority: P1) 🎯 MVP

**Goal**: Wire up FastAPI application with all endpoints

**Independent Test**: Can start server and access all endpoints

### Implementation for User Story 8

- [x] T071 [US8] Create FastAPI app in backend/src/main.py
- [x] T072 [US8] Register all API routes in backend/src/main.py
- [x] T073 [US8] Add CORS middleware in backend/src/main.py
- [x] T074 [US8] Add exception handlers in backend/src/main.py
- [x] T075 [US8] Configure uvicorn for development in backend/src/main.py
- [x] T076 [US8] Create backend/.env.example with all required variables
- [x] T077 [US8] Update backend/pyproject.toml with correct dependencies

**Checkpoint**: FastAPI application running with all endpoints

---

## Phase 11: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and documentation

- [x] T078 [P] Verify no SQL imports: grep -r "asyncpg\|sqlalchemy" backend/src/
- [ ] T079 [P] Run all tests: pytest backend/tests/
- [ ] T080 [P] Test /api/ask with sample question
- [ ] T081 [P] Test /api/ask-selected with sample text
- [ ] T082 [P] Test /api/health endpoint
- [x] T083 Validate Constitution Section VIII compliance checklist
- [ ] T084 Run quickstart.md validation steps
- [ ] T085 Update README.md with new setup instructions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-10)**: All depend on Foundational phase completion
- **Polish (Phase 11)**: Depends on all user stories being complete

### User Story Dependencies

| Story | Depends On | Can Parallel With |
|-------|-----------|-------------------|
| US1 (Embeddings) | Phase 2 | US2 |
| US2 (Rerank) | Phase 2 | US1 |
| US3 (Qdrant) | US1 | - |
| US4 (Ingestion) | US1, US3 | - |
| US5 (/api/ask) | US1, US2, US3 | US6, US7 |
| US6 (/api/ask-selected) | US2 | US5, US7 |
| US7 (/api/health) | US3 | US5, US6 |
| US8 (FastAPI App) | US5, US6, US7 | - |

### Parallel Opportunities per Phase

**Phase 2 (Foundational)**:
```bash
# All model creation tasks can run in parallel:
T009, T010, T011, T012, T013, T014, T015
```

**Phase 3-4 (Embeddings + Rerank)**:
```bash
# Tests can run in parallel:
T018, T019, T025, T026
# US1 and US2 implementation can run in parallel
```

**Phase 7-9 (API Endpoints)**:
```bash
# All contract tests can run in parallel:
T048, T059, T066
```

---

## Implementation Strategy

### MVP First (Phases 1-3, 7, 10)

1. Complete Phase 1: Remove SQL dependencies
2. Complete Phase 2: Foundational models and config
3. Complete Phase 3: Cohere Embeddings (US1)
4. Complete Phase 5: Qdrant Vector Store (US3) - needed for /api/ask
5. Complete Phase 4: Cohere Rerank (US2)
6. Complete Phase 7: /api/ask endpoint (US5)
7. Complete Phase 10: FastAPI App (US8)
8. **STOP and VALIDATE**: Test MVP independently

### Full Implementation

1. MVP above
2. Add Phase 6: Content Ingestion (US4)
3. Add Phase 8: /api/ask-selected (US6)
4. Add Phase 9: /api/health (US7)
5. Complete Phase 11: Polish

---

## Constitution Section VIII Compliance Checklist

- [x] Remove asyncpg dependency (T001)
- [x] Remove sqlalchemy dependency (T002)
- [x] Remove all PostgreSQL/Neon code (T005, T006, T007)
- [x] Implement Cohere embeddings embed-english-v3.0 (T020-T024)
- [x] Implement Cohere rerank (T027-T030)
- [x] All embeddings stored in Qdrant only (T033-T038)
- [x] Select-text bypasses Qdrant, uses Cohere rerank only (T061-T063)

---

## Summary

- **Total Tasks**: 85
- **Phase 1 (Setup)**: 7 tasks
- **Phase 2 (Foundational)**: 10 tasks
- **US1 (Embeddings)**: 7 tasks
- **US2 (Rerank)**: 6 tasks
- **US3 (Qdrant)**: 8 tasks
- **US4 (Ingestion)**: 9 tasks
- **US5 (/api/ask)**: 12 tasks
- **US6 (/api/ask-selected)**: 7 tasks
- **US7 (/api/health)**: 5 tasks
- **US8 (FastAPI App)**: 7 tasks
- **Phase 11 (Polish)**: 8 tasks

**Parallel Opportunities**: 35 tasks marked [P]
**MVP Scope**: Phases 1-3, 5, 4, 7, 10 (US1, US2, US3, US5, US8)

---

*Tasks generated by /sp.tasks command*
