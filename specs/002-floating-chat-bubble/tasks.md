# Tasks: Floating Chat Bubble

**Input**: Design documents from `/specs/002-floating-chat-bubble/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: Not requested in spec - manual testing only per quickstart.md checklist.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Project type**: Docusaurus web app (single project structure)
- **Components**: `src/components/FloatingChat/`
- **Theme**: `src/theme/`
- **Styles**: `src/css/`
- **Reused**: `src/components/Chat/`, `src/hooks/`

---

## Phase 1: Setup

**Purpose**: Project initialization and Docusaurus theme swizzle

- [x] T001 Swizzle Docusaurus Root component using `npx docusaurus swizzle @docusaurus/theme-classic Root --wrap --typescript` to create src/theme/Root.tsx
- [x] T002 [P] Create FloatingChat component directory at src/components/FloatingChat/
- [x] T003 [P] Create barrel export file at src/components/FloatingChat/index.ts

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Create FloatingChatContext provider with state (isOpen, messages, isLoading, error) in src/components/FloatingChat/FloatingChatContext.tsx
- [x] T005 Integrate useStreamingChat hook into FloatingChatContext for message handling in src/components/FloatingChat/FloatingChatContext.tsx
- [x] T006 Implement 50-message limit with FIFO trimming in FloatingChatContext (FR-014) in src/components/FloatingChat/FloatingChatContext.tsx
- [x] T007 Create analytics utility function emitChatAnalytics() in src/components/FloatingChat/analytics.ts
- [x] T008 Wire FloatingChatProvider into Root.tsx wrapper in src/theme/Root.tsx
- [x] T009 Create base floatingChat.module.css with CSS custom properties in src/css/floatingChat.module.css

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Quick Question While Reading (Priority: P1) 🎯 MVP

**Goal**: User can see bubble on any doc page, click to open panel, ask question, receive streaming answer

**Independent Test**: Open any doc page → Click bubble → Type question → Receive answer with streaming

### Implementation for User Story 1

- [x] T010 [P] [US1] Create FloatingChatBubble component with chat icon button in src/components/FloatingChat/FloatingChatBubble.tsx
- [x] T011 [P] [US1] Add bubble positioning styles (fixed, bottom-right, z-index 9999) in src/css/floatingChat.module.css
- [x] T012 [US1] Create FloatingChatPanel component with header, MessageList, and ChatInput in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T013 [US1] Add panel styles (fixed positioning, dimensions, shadow, border-radius) in src/css/floatingChat.module.css
- [x] T014 [US1] Implement open/close animation (transform + opacity, 300ms) in src/css/floatingChat.module.css
- [x] T015 [US1] Add loading indicator display during streaming in FloatingChatPanel in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T016 [US1] Emit chat_opened analytics event on panel open in src/components/FloatingChat/FloatingChatBubble.tsx
- [x] T017 [US1] Emit message_sent analytics event on message submission in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T018 [US1] Render FloatingChatBubble in Root.tsx below FloatingChatProvider in src/theme/Root.tsx
- [x] T019 [US1] Update FloatingChat/index.ts exports in src/components/FloatingChat/index.ts

**Checkpoint**: User Story 1 complete - bubble visible, panel opens, can ask questions and receive streaming answers

---

## Phase 4: User Story 2 - Minimize Chat to Continue Reading (Priority: P1)

**Goal**: User can close panel without losing conversation, reopen to see history, panel doesn't interfere with scrolling

**Independent Test**: Open chat → Send message → Close panel → Scroll page → Reopen → Conversation visible

### Implementation for User Story 2

- [x] T020 [US2] Add close button to FloatingChatPanel header in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T021 [US2] Wire close button to togglePanel() from context in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T022 [US2] Ensure messages state persists when isOpen changes to false in src/components/FloatingChat/FloatingChatContext.tsx
- [x] T023 [US2] Verify panel uses position:fixed to not interfere with page scroll in src/css/floatingChat.module.css

**Checkpoint**: User Story 2 complete - minimize works, conversation persists, no scroll interference

---

## Phase 5: User Story 3 - Ask Follow-up Questions (Priority: P2)

**Goal**: Multi-turn conversations work, user can scroll through message history in panel

**Independent Test**: Ask question → Get answer → Ask follow-up referencing previous context → AI responds contextually

### Implementation for User Story 3

- [x] T024 [US3] Verify MessageList component handles scrolling within panel bounds in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T025 [US3] Add max-height and overflow-y:auto to message area in src/css/floatingChat.module.css
- [x] T026 [US3] Verify conversation history is passed to API for context (already in useStreamingChat) - validate in src/hooks/useStreamingChat.ts

**Checkpoint**: User Story 3 complete - follow-up questions work with context, history scrollable

---

## Phase 6: User Story 4 - Start Fresh Conversation (Priority: P3)

**Goal**: User can clear conversation and start fresh

**Independent Test**: Have conversation → Click "New Chat" → Conversation clears → New question answered without old context

### Implementation for User Story 4

- [x] T027 [US4] Add "New Chat" button to FloatingChatPanel header in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T028 [US4] Implement clearConversation() function in FloatingChatContext in src/components/FloatingChat/FloatingChatContext.tsx
- [x] T029 [US4] Wire "New Chat" button to clearConversation() in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T030 [US4] Style "New Chat" button in header in src/css/floatingChat.module.css

**Checkpoint**: User Story 4 complete - can clear and start fresh conversation

---

## Phase 7: User Story 5 - Mobile-Friendly Chat Access (Priority: P2)

**Goal**: Chat works well on mobile devices (320px+), touch-friendly targets

**Independent Test**: Open on mobile viewport → Bubble tappable → Panel opens at appropriate size → Input usable → Close button tappable

### Implementation for User Story 5

- [x] T031 [P] [US5] Add responsive breakpoints for bubble (48px touch target minimum) in src/css/floatingChat.module.css
- [x] T032 [P] [US5] Add responsive panel styles at 768px breakpoint in src/css/floatingChat.module.css
- [x] T033 [P] [US5] Add responsive panel styles at 480px breakpoint (near full-width) in src/css/floatingChat.module.css
- [x] T034 [US5] Ensure input font-size 16px+ to prevent iOS zoom in src/css/floatingChat.module.css
- [x] T035 [US5] Add touch-action and -webkit-tap-highlight-color for mobile UX in src/css/floatingChat.module.css

**Checkpoint**: User Story 5 complete - mobile experience optimized

---

## Phase 8: Cross-Cutting - Keyboard Accessibility (FR-013)

**Purpose**: Full keyboard navigation support

- [x] T036 Add Esc key handler to close panel in FloatingChatPanel in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T037 Add focus management (auto-focus input on panel open) in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T038 Ensure Tab navigates through panel elements (header buttons, input, send) in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T039 Add aria-label to bubble button for screen readers in src/components/FloatingChat/FloatingChatBubble.tsx
- [x] T040 Add aria attributes to panel (role="dialog", aria-modal) in src/components/FloatingChat/FloatingChatPanel.tsx

---

## Phase 9: Cross-Cutting - Error Handling & Edge Cases

**Purpose**: Robust error handling per edge cases in spec

- [x] T041 Display error message in panel when API fails (reuse ErrorMessage component) in src/components/FloatingChat/FloatingChatPanel.tsx
- [x] T042 Emit error_occurred analytics event on errors in src/components/FloatingChat/FloatingChatContext.tsx
- [x] T043 Add retry capability after error in src/components/FloatingChat/FloatingChatPanel.tsx

---

## Phase 10: Polish & Validation

**Purpose**: Final cleanup and validation

- [ ] T044 Verify bubble appears on all doc pages (test 5+ different pages)
- [ ] T045 Verify conversation persists across 10+ page navigations (SC-005)
- [ ] T046 Verify animations complete in <300ms (SC-004)
- [ ] T047 Test on 320px viewport (SC-006)
- [ ] T048 Run full quickstart.md checklist validation
- [x] T049 Build production bundle and test (`npm run build && npm run serve`)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 - BLOCKS all user stories
- **Phases 3-7 (User Stories)**: All depend on Phase 2 completion
  - US1 (P1) and US2 (P1) are both Priority 1 - do sequentially
  - US3-US5 can follow in priority order
- **Phases 8-9 (Cross-Cutting)**: Can start after US1 is complete
- **Phase 10 (Polish)**: Depends on all user stories being complete

### User Story Dependencies

| Story | Priority | Dependencies | Can Start After |
|-------|----------|--------------|-----------------|
| US1 - Quick Question | P1 | None | Phase 2 |
| US2 - Minimize | P1 | US1 | Phase 3 |
| US3 - Follow-ups | P2 | US1, US2 | Phase 4 |
| US4 - Fresh Start | P3 | US1 | Phase 3 |
| US5 - Mobile | P2 | US1 | Phase 3 |

### Parallel Opportunities

**Within Phase 1**:
```
T002 + T003 (both [P])
```

**Within Phase 3 (US1)**:
```
T010 + T011 (both [P])
```

**Within Phase 7 (US5)**:
```
T031 + T032 + T033 (all [P])
```

---

## Implementation Strategy

### MVP First (User Story 1 + 2 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T009)
3. Complete Phase 3: User Story 1 (T010-T019)
4. Complete Phase 4: User Story 2 (T020-T023)
5. **STOP and VALIDATE**: Test bubble, panel, messaging, minimize
6. Deploy/demo if ready - this is a functional MVP!

### Incremental Delivery

1. Setup + Foundational → Core infrastructure ready
2. Add US1 + US2 → MVP (can ask questions, minimize panel)
3. Add US3 → Follow-up questions work
4. Add US4 → Can clear conversations
5. Add US5 → Mobile optimized
6. Add Cross-cutting → Keyboard accessibility, error handling
7. Polish → Final validation

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 49 |
| **Setup Phase** | 3 |
| **Foundational Phase** | 6 |
| **User Story 1 (P1)** | 10 |
| **User Story 2 (P1)** | 4 |
| **User Story 3 (P2)** | 3 |
| **User Story 4 (P3)** | 4 |
| **User Story 5 (P2)** | 5 |
| **Cross-Cutting** | 8 |
| **Polish** | 6 |
| **Parallel Tasks [P]** | 10 |
| **MVP Scope** | T001-T023 (23 tasks) |

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story is independently testable once its phase is complete
- Reuses existing components: MessageList, MessageBubble, ChatInput, ErrorMessage, useStreamingChat
- No new backend work required - reuses /api/ask/stream
- Commit after each task or logical group for clean git history
