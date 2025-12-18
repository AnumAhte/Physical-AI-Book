# Data Model: Floating Chat Bubble

**Feature**: 002-floating-chat-bubble
**Date**: 2025-12-17

## Overview

This feature is primarily UI-focused with no new persistent data storage. All state is managed in-memory via React Context and reuses existing type definitions from the chat module.

## Entities

### 1. FloatingChatState (New)

**Purpose**: Global state for the floating chat bubble widget.

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| isOpen | boolean | Whether chat panel is expanded | Default: false |
| messages | Message[] | Conversation history | Max 50 items (FR-014) |
| isLoading | boolean | Streaming response in progress | - |
| error | string \| null | Current error message | Cleared on new message |
| streamingMessageId | string \| null | ID of message being streamed | null when not streaming |

**State Transitions**:
```
closed → open      (user clicks bubble)
open → closed      (user clicks close/Esc)
idle → loading     (user sends message)
loading → idle     (response complete or error)
any → cleared      (user clicks "new chat")
```

### 2. Message (Existing - Reused)

**Source**: `src/types/chat.ts`

| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique identifier (timestamp + random) |
| role | 'user' \| 'assistant' | Message sender |
| content | string | Message text |
| citations | Citation[] | Source references (assistant only) |
| confidence | number | 0-1 confidence score (assistant only) |
| lowConfidence | boolean | Backend-flagged low confidence |
| isError | boolean | Error message flag |

### 3. Citation (Existing - Reused)

**Source**: `src/types/chat.ts`

| Field | Type | Description |
|-------|------|-------------|
| chapter | string | Chapter identifier |
| chapterTitle | string | Chapter display name |
| sourceFile | string | Source markdown file |
| excerpt | string | Relevant text excerpt |

### 4. AnalyticsEvent (New)

**Purpose**: Structure for emitted analytics events.

| Field | Type | Description |
|-------|------|-------------|
| event | ChatAnalyticsEvent | Event type enum |
| timestamp | number | Unix timestamp (ms) |
| data | object \| undefined | Event-specific payload |

**Event Types (ChatAnalyticsEvent enum)**:
- `chat_opened` - User opened the chat panel
- `message_sent` - User sent a message (data: { messageLength })
- `error_occurred` - Error during operation (data: { error })

## Relationships

```
FloatingChatState
    │
    ├── contains → Message[] (0..50)
    │                  │
    │                  └── contains → Citation[] (0..n)
    │
    └── emits → AnalyticsEvent
```

## Validation Rules

### Message Limit (FR-014)
- Maximum 50 messages per session
- When limit exceeded, remove oldest message (FIFO)
- Both user and assistant messages count toward limit

### Message Content
- User message: Non-empty string (trimmed)
- Assistant message: May be empty during streaming, populated incrementally

### Panel State
- Only one state at a time (open XOR closed)
- Panel closes on Esc key press
- Panel state persists across page navigation

## No Database Schema

This feature uses no persistent storage:
- ❌ No SQL database
- ❌ No localStorage/sessionStorage
- ❌ No cookies
- ✅ React Context (in-memory, session-scoped)

Conversation is lost on:
- Browser refresh (hard navigation)
- Browser close
- Manual clear by user

This aligns with spec assumption: "Session-based conversation persistence is acceptable (no cross-session persistence required)"
