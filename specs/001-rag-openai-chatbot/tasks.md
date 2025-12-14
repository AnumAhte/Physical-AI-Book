# Tasks: RAG Chatbot with OpenAI

**Input**: Design documents from `/specs/001-rag-openai-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml

**Tests**: Not explicitly requested in specification. Manual testing via quickstart.md.

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/src/`, `backend/tests/`
- **Frontend**: `src/pages/`, `src/components/`, `src/css/`

---

## Phase 1: Setup

**Purpose**: Verify existing infrastructure and prepare for new development

- [x] T001 Verify backend runs with `uvicorn src.main:app --reload` in backend/
- [x] T002 Verify frontend runs with `npm start` at project root
- [x] T003 [P] Verify Qdrant connection via `GET /api/health` endpoint
- [x] T004 [P] Verify OpenAI API key is configured in backend/.env (LLM_PROVIDER=openai)

---

## Phase 2: Foundational (Backend Streaming Infrastructure)

**Purpose**: Core streaming infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Add StreamChunk model to backend/src/models/responses.py per data-model.md
- [x] T006 Update backend/src/models/__init__.py to export StreamChunk
- [x] T007 Add `generate_response_stream()` async generator function in backend/src/llm.py
- [x] T008 Update backend/src/config.py to set gpt-4o-mini as default model for openai provider
- [x] T009 Add streaming RAG pipeline function `ask_question_stream()` in backend/src/rag_pipeline.py
- [x] T010 Add POST /api/ask/stream endpoint in backend/src/api.py using StreamingResponse
- [x] T011 Test streaming endpoint manually with curl: `curl -X POST http://localhost:8000/api/ask/stream -H "Content-Type: application/json" -d '{"question":"What is Physical AI?"}' --no-buffer`

**Checkpoint**: Backend streaming infrastructure ready - frontend implementation can now begin

---

## Phase 3: User Story 1 - Ask a Question About Textbook Content (Priority: P1) MVP

**Goal**: User can ask questions and receive streamed answers from textbook content

**Independent Test**: Navigate to /chat, type a question, verify streaming response appears progressively

### Implementation for User Story 1

- [x] T012 [P] [US1] Create TypeScript types in src/types/chat.ts (Message, Citation, ChatState, StreamEvent)
- [x] T013 [P] [US1] Create chat styles in src/css/chat.module.css (container, messages, bubbles, input)
- [x] T014 [US1] Create ChatInput component in src/components/Chat/ChatInput.tsx (input field + send button)
- [x] T015 [US1] Create MessageBubble component in src/components/Chat/MessageBubble.tsx (user/assistant styling)
- [x] T016 [US1] Create TypingIndicator component in src/components/Chat/TypingIndicator.tsx (loading animation)
- [x] T017 [US1] Create useStreamingChat hook in src/hooks/useStreamingChat.ts (SSE fetch + state management)
- [x] T018 [US1] Create ChatContainer component in src/components/Chat/ChatContainer.tsx (orchestrates state and renders children)
- [x] T019 [US1] Create chat page at src/pages/chat.tsx integrating ChatContainer
- [x] T020 [US1] Add /chat route to Docusaurus navbar in docusaurus.config.ts
- [x] T021 [US1] Test end-to-end: ask "What is Physical AI?" and verify streaming response

**Checkpoint**: User Story 1 complete - users can ask questions and receive streaming answers

---

## Phase 4: User Story 2 - View Source References (Priority: P2)

**Goal**: Users can see which textbook sections were used to generate the answer

**Independent Test**: Ask a question, verify citations appear below the answer with chapter/section info

### Implementation for User Story 2

- [x] T022 [P] [US2] Create SourceCard component in src/components/Chat/SourceCard.tsx (displays single citation) - INLINE in MessageBubble
- [x] T023 [US2] Create CitationList component in src/components/Chat/CitationList.tsx (renders list of SourceCards) - INLINE in MessageBubble
- [x] T024 [US2] Update MessageBubble in src/components/Chat/MessageBubble.tsx to render CitationList for assistant messages
- [x] T025 [US2] Update chat.module.css in src/css/chat.module.css with citation styling
- [x] T026 [US2] Test citations: ask a question and verify source references display correctly

**Checkpoint**: User Story 2 complete - citations display with chapter, title, and excerpt

---

## Phase 5: User Story 3 - Handle Out-of-Scope Questions (Priority: P2)

**Goal**: System gracefully handles questions not covered by textbook content

**Independent Test**: Ask an off-topic question (e.g., "What is the capital of France?"), verify appropriate response

### Implementation for User Story 3

- [x] T027 [US3] Update system prompt in backend/src/llm.py to explicitly decline out-of-scope questions
- [x] T028 [US3] Add low-confidence handling in backend/src/rag_pipeline.py ask_question_stream() (threshold check)
- [x] T029 [US3] Update MessageBubble in src/components/Chat/MessageBubble.tsx to style low-confidence responses differently
- [x] T030 [US3] Test out-of-scope: ask "What is the capital of France?" and verify polite decline message

**Checkpoint**: User Story 3 complete - out-of-scope questions handled gracefully

---

## Phase 6: User Story 4 - Conversational Experience (Priority: P3)

**Goal**: Multi-turn conversation with message history displayed in chronological order

**Independent Test**: Ask multiple questions in sequence, verify all messages remain visible

### Implementation for User Story 4

- [x] T031 [P] [US4] Create MessageList component in src/components/Chat/MessageList.tsx (scrollable container with auto-scroll)
- [x] T032 [US4] Update ChatContainer in src/components/Chat/ChatContainer.tsx to use MessageList
- [x] T033 [US4] Add auto-scroll to bottom behavior when new messages arrive in MessageList
- [x] T034 [US4] Add empty state message in ChatContainer ("Ask a question about the textbook...")
- [x] T035 [US4] Test conversation: ask 3+ questions and verify all messages persist in order

**Checkpoint**: User Story 4 complete - full conversational experience

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, responsiveness, and final validation

- [x] T036 [P] Add error display component in src/components/Chat/ErrorMessage.tsx
- [x] T037 [P] Add mobile-responsive styles in src/css/chat.module.css (media queries)
- [x] T038 Update ChatContainer in src/components/Chat/ChatContainer.tsx to display errors from useStreamingChat
- [x] T039 Add input validation in ChatInput (prevent empty submissions, max length)
- [x] T040 Add keyboard support: Enter to send, Shift+Enter for newline in ChatInput
- [ ] T041 Run full quickstart.md validation (both backend and frontend)
- [ ] T042 Final manual testing: test all edge cases from spec.md (empty questions, long questions, service errors)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - verify existing infrastructure
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - US1 (Phase 3): Must complete first (core functionality)
  - US2 (Phase 4): Can start after US1 (extends MessageBubble)
  - US3 (Phase 5): Can start after US1 (extends backend + MessageBubble)
  - US4 (Phase 6): Can start after US1 (extends ChatContainer)
- **Polish (Phase 7)**: Depends on all user stories complete

### User Story Dependencies

- **User Story 1 (P1)**: Foundation only - Core MVP
- **User Story 2 (P2)**: Depends on US1 (MessageBubble exists)
- **User Story 3 (P2)**: Depends on US1 (backend pipeline exists)
- **User Story 4 (P3)**: Depends on US1 (ChatContainer exists)

### Within Each User Story

- Types/styles (marked [P]) can run in parallel
- Components build on each other sequentially
- Integration test is final task in each story

### Parallel Opportunities

**Phase 1 (Setup)**:
```
T003, T004 can run in parallel
```

**Phase 2 (Foundational)**:
```
T005, T006 must complete before T007-T010
T007, T008 can run in parallel
```

**Phase 3 (US1)**:
```
T012, T013 can run in parallel (types + styles)
T014, T015, T016 can run in parallel (independent components)
T017, T018 sequential (hook before container)
```

**Phase 4-6 (US2-US4)**:
```
Once US1 complete, US2, US3, US4 can proceed in parallel if staffed
```

**Phase 7 (Polish)**:
```
T036, T037 can run in parallel
```

---

## Parallel Example: User Story 1 Initial Tasks

```bash
# Launch types and styles in parallel:
Task: "Create TypeScript types in src/types/chat.ts"
Task: "Create chat styles in src/css/chat.module.css"

# Launch independent components in parallel:
Task: "Create ChatInput component in src/components/Chat/ChatInput.tsx"
Task: "Create MessageBubble component in src/components/Chat/MessageBubble.tsx"
Task: "Create TypingIndicator component in src/components/Chat/TypingIndicator.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (verify infrastructure)
2. Complete Phase 2: Foundational (streaming backend)
3. Complete Phase 3: User Story 1 (basic Q&A)
4. **STOP and VALIDATE**: Test basic chat flow
5. Deploy if ready - core functionality works

### Incremental Delivery

1. Setup + Foundational → Backend streaming works
2. Add User Story 1 → Basic chat with streaming (MVP!)
3. Add User Story 2 → Citations visible
4. Add User Story 3 → Out-of-scope handling
5. Add User Story 4 → Full conversation history
6. Polish → Production ready

### Suggested MVP Scope

**Minimum Viable Product (Tasks T001-T021)**:
- Phases 1-3 only (Setup + Foundational + US1)
- User can ask questions and receive streaming answers
- 21 tasks total for MVP

---

## Summary

| Metric | Count |
|--------|-------|
| Total Tasks | 42 |
| Setup Tasks | 4 |
| Foundational Tasks | 7 |
| User Story 1 Tasks | 10 |
| User Story 2 Tasks | 5 |
| User Story 3 Tasks | 4 |
| User Story 4 Tasks | 5 |
| Polish Tasks | 7 |
| Parallel Opportunities | 15 tasks marked [P] |
| MVP Tasks | 21 (Phases 1-3) |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently testable after completion
- Verify streaming works manually after Phase 2
- Commit after each task or logical group
- Stop at any checkpoint to validate independently
