# Implementation Plan: Floating Chat Bubble

**Branch**: `002-floating-chat-bubble` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-floating-chat-bubble/spec.md`

## Summary

Create a floating chat bubble widget that appears on all documentation pages, allowing users to ask questions about the Physical AI textbook without navigating away. The implementation will reuse existing chat components (MessageBubble, MessageList, ChatInput) and the `useStreamingChat` hook, wrapping them in a new floating UI container with cross-page state persistence via React Context and Docusaurus theme swizzling.

## Technical Context

**Language/Version**: TypeScript 5.x, React 19.0, Docusaurus 3.9.2
**Primary Dependencies**: React, Docusaurus preset-classic, CSS Modules, existing chat components
**Storage**: In-memory React Context (session-based, no database)
**Testing**: Manual testing (Docusaurus project, no test framework configured)
**Target Platform**: Web (modern browsers, responsive down to 320px)
**Project Type**: Web (Docusaurus documentation site)
**Performance Goals**: Animation <300ms, interaction response <100ms
**Constraints**: No external state libraries, reuse existing streaming API, free-tier backend (Render)
**Scale/Scope**: Single floating widget, 50 message limit per session, ~20 documentation pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Simplicity and Clarity | ✅ PASS | Reuses existing components, minimal new code |
| II. Accuracy and Veracity | ✅ PASS | Uses existing RAG backend (textbook-only answers) |
| III. Minimalist Design | ✅ PASS | Lightweight CSS, no new dependencies |
| IV. Spec-Driven Development | ✅ PASS | Following SDD workflow |
| V. RAG Chatbot Integrity | ✅ PASS | Same backend, textbook-only responses |
| VI. Modular Architecture | ✅ PASS | New components in src/components/FloatingChat/ |
| VII. CI/CD Readiness | ✅ PASS | No changes to build pipeline |
| VIII. Storage Requirements | ✅ PASS | No database needed (React state only) |

**Gate Result**: PASS - No violations. Proceed to Phase 0.

## Project Structure

### Documentation (this feature)

```text
specs/002-floating-chat-bubble/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (N/A - frontend only)
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code (repository root)

```text
src/
├── components/
│   ├── Chat/                    # Existing (reuse)
│   │   ├── ChatContainer.tsx
│   │   ├── ChatInput.tsx
│   │   ├── MessageBubble.tsx
│   │   ├── MessageList.tsx
│   │   ├── TypingIndicator.tsx
│   │   ├── ErrorMessage.tsx
│   │   └── index.ts
│   └── FloatingChat/            # NEW
│       ├── FloatingChatBubble.tsx   # Main bubble component
│       ├── FloatingChatPanel.tsx    # Expanded chat panel
│       ├── FloatingChatContext.tsx  # Cross-page state persistence
│       └── index.ts
├── css/
│   ├── chat.module.css          # Existing (reuse)
│   └── floatingChat.module.css  # NEW - floating-specific styles
├── hooks/
│   └── useStreamingChat.ts      # Existing (reuse)
├── theme/                       # NEW (Docusaurus swizzle)
│   └── Root.tsx                 # Wrap app with FloatingChatProvider
├── types/
│   └── chat.ts                  # Existing (extend if needed)
└── pages/
    └── chat.tsx                 # Existing (keep as-is)
```

**Structure Decision**: Extending existing web structure. New FloatingChat module with Docusaurus theme swizzle for global injection.

## Architecture Overview

### Component Hierarchy

```
<Root>                              # Docusaurus theme wrapper (swizzled)
  └── <FloatingChatProvider>        # Context for cross-page state
       └── <Layout>                 # Docusaurus Layout
            ├── <Page Content>      # Documentation content
            └── <FloatingChatBubble># Fixed position widget
                 └── <FloatingChatPanel>  # Expanded panel (conditional)
                      ├── <Header>        # Title + close + clear buttons
                      ├── <MessageList>   # Reused from Chat/
                      └── <ChatInput>     # Reused from Chat/
```

### State Flow

```
FloatingChatContext (global)
├── isOpen: boolean              # Panel expanded state
├── messages: Message[]          # Conversation history
├── isLoading: boolean           # Streaming state
├── error: string | null         # Error state
├── togglePanel(): void          # Open/close
├── sendMessage(q): Promise      # Delegates to useStreamingChat
├── clearConversation(): void    # Reset messages
└── emitAnalytics(event): void   # Track events
```

### Key Design Decisions

1. **Docusaurus Root Swizzle**: Use `@theme/Root` to inject the floating bubble globally without modifying every page.

2. **React Context for Persistence**: Messages persist across page navigation because Context lives above the router.

3. **Component Reuse**: MessageList, MessageBubble, ChatInput reused directly; only container/layout changes.

4. **CSS Modules**: Consistent with existing styling approach; new floatingChat.module.css for bubble-specific styles.

5. **Keyboard Accessibility**: Built into FloatingChatBubble with Esc handler and focus management.

6. **Analytics via Custom Event**: Simple `window.dispatchEvent` pattern for analytics events (chat_opened, message_sent, error_occurred).

## Complexity Tracking

> No violations - table not needed.

## Risk Analysis

| Risk | Mitigation |
|------|------------|
| Docusaurus version incompatibility with swizzle | Use "wrap" swizzle mode (safer than eject) |
| Context state lost on hard refresh | Acceptable per spec (session-only persistence) |
| Z-index conflicts with Docusaurus UI | Use high z-index (9999) and test across pages |
| Mobile keyboard pushing layout | Use fixed positioning with bottom offset |

## Dependencies

### Existing (No Changes)

- `useStreamingChat` hook - streaming API logic
- `MessageBubble`, `MessageList`, `ChatInput` components
- `chat.module.css` - message styling
- `/api/ask/stream` endpoint on Render backend

### New (To Create)

- `FloatingChatBubble` component
- `FloatingChatPanel` component
- `FloatingChatContext` provider
- `floatingChat.module.css` styles
- `@theme/Root.tsx` wrapper

## API Contract

**No new backend APIs required.** Reuses existing:

```
POST /api/ask/stream
Content-Type: application/json

Request:  { "question": "string" }
Response: Server-Sent Events (SSE)
          data: { content, done, citations?, confidence?, lowConfidence?, error? }
```

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Implementation order:
   - P0: FloatingChatContext + Root wrapper (cross-page persistence foundation)
   - P1: FloatingChatBubble + FloatingChatPanel (core UI)
   - P2: Keyboard accessibility + analytics events
   - P3: Mobile responsive styling
   - P4: Integration testing across pages
