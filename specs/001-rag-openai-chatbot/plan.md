# Implementation Plan: RAG Chatbot with OpenAI

**Branch**: `001-rag-openai-chatbot` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-openai-chatbot/spec.md`

## Summary

Add a RAG chatbot to the Physical AI Textbook that answers questions using embedded textbook content stored in Qdrant Cloud. The backend will use OpenAI gpt-4o-mini for response generation with streaming support. The frontend will provide a chat UI at `/chat` route using React components within the existing Docusaurus framework.

**Key Discovery**: The backend RAG pipeline already exists with Cohere embeddings + Qdrant search + Cohere rerank. The current LLM integration supports both Anthropic and OpenAI. Changes required:
1. Add streaming support to the existing OpenAI integration
2. Create new streaming endpoint `/api/ask/stream`
3. Build frontend chat page using React (ChatKit or custom components)

## Technical Context

**Language/Version**: Python 3.10+ (backend), TypeScript/React 19 (frontend)
**Primary Dependencies**:
- Backend: FastAPI, qdrant-client, cohere, openai (existing)
- Frontend: Docusaurus 3.9.2, React 19, @stream-io/chat or custom components
**Storage**: Qdrant Cloud (existing, pre-populated with textbook embeddings)
**Testing**: pytest, pytest-asyncio (backend), manual/E2E (frontend)
**Target Platform**: Web (Vercel frontend, backend server)
**Project Type**: Web application (existing monorepo)
**Performance Goals**: First token < 3s, full response < 10s
**Constraints**: OpenAI only (no Anthropic), streaming required, Cohere for embeddings
**Scale/Scope**: Single collection (~textbook content), moderate traffic

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Simplicity and Clarity | ✅ PASS | Minimal changes to existing architecture |
| II. Accuracy and Veracity | ✅ PASS | RAG ensures answers from textbook only |
| III. Minimalist Design | ✅ PASS | Reuses existing backend, adds minimal frontend |
| IV. Spec-Driven Development | ✅ PASS | Following SDD workflow |
| V. RAG Chatbot Integrity | ✅ PASS | Answers sourced only from textbook content |
| VI. Modular Architecture | ✅ PASS | Separate backend/frontend, clear concerns |
| VII. CI/CD Readiness | ✅ PASS | Existing workflows can be extended |
| VIII. Storage & RAG Requirements | ✅ PASS | Using Qdrant + Cohere (no SQL) |

**Constitution Section VIII Compliance**:
- ❌ SQL / NEON: Not used ✅
- ✅ Qdrant Cloud: Existing collection ✅
- ✅ Cohere Embeddings: Already integrated ✅
- ✅ FastAPI: Existing backend ✅
- ✅ RAG Pipeline: Chunk → Cohere Embeddings → Qdrant → Cohere Rerank → OpenAI ✅

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-openai-chatbot/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── openapi.yaml     # API contract
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api.py           # Add streaming endpoint
│   ├── llm.py           # Add streaming generation
│   ├── models/
│   │   ├── requests.py  # Existing (no changes)
│   │   └── responses.py # Existing (no changes)
│   └── ...              # Existing modules
└── tests/
    └── test_streaming.py # New streaming tests

src/
├── pages/
│   └── chat.tsx         # NEW: Chat page
├── components/
│   └── Chat/            # NEW: Chat components
│       ├── ChatContainer.tsx
│       ├── MessageBubble.tsx
│       ├── ChatInput.tsx
│       └── TypingIndicator.tsx
└── css/
    └── chat.module.css  # NEW: Chat styles
```

**Structure Decision**: Web application structure (Option 2) - backend + frontend already separated. Extending existing structure with new chat components and streaming endpoint.

## Complexity Tracking

No violations requiring justification. The implementation reuses existing infrastructure.

---

## Phase 0 Artifacts

See [research.md](./research.md) for technology decisions and best practices.

## Phase 1 Artifacts

- [data-model.md](./data-model.md) - Entity definitions
- [contracts/openapi.yaml](./contracts/openapi.yaml) - API contract for streaming endpoint
- [quickstart.md](./quickstart.md) - Local development guide
