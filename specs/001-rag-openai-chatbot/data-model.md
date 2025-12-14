# Data Model: RAG Chatbot with OpenAI

**Feature Branch**: `001-rag-openai-chatbot`
**Date**: 2025-12-14

## Overview

This document defines the data entities for the RAG Chatbot feature. All models are Pydantic data classes (per Constitution Section VIII - NO SQL).

---

## Backend Entities

### Existing Entities (No Changes)

These entities are already implemented in `backend/src/models/`:

#### TextChunk
```python
class TextChunk(BaseModel):
    """A chunk of textbook content with metadata."""
    content: str
    chapter: str
    chapter_title: str
    source_file: str
    chunk_index: int
```

#### SearchResult
```python
class SearchResult(BaseModel):
    """Result from Qdrant vector search."""
    content: str
    chapter: str
    chapter_title: str
    source_file: str
    score: float
```

#### RerankedResult
```python
class RerankedResult(BaseModel):
    """Result after Cohere reranking."""
    content: str
    chapter: str
    chapter_title: str
    source_file: str
    relevance_score: float
```

#### Citation
```python
class Citation(BaseModel):
    """A citation reference in the response."""
    chapter: str
    chapter_title: str
    source_file: str
    excerpt: str = Field(..., max_length=200)
```

#### RAGResponse
```python
class RAGResponse(BaseModel):
    """Response from RAG endpoints."""
    answer: str
    citations: list[Citation]
    confidence: float = Field(..., ge=0.0, le=1.0)
```

#### AskRequest
```python
class AskRequest(BaseModel):
    """Request body for POST /api/ask endpoint."""
    question: str = Field(..., min_length=3, max_length=1000)
    top_k: int = Field(default=5, ge=1, le=20)
```

---

### New Entities for Streaming

#### StreamChunk
```python
class StreamChunk(BaseModel):
    """A chunk of streamed response content."""
    content: str           # Partial text content
    done: bool = False     # True when streaming complete
    citations: list[Citation] | None = None  # Only on final chunk
    confidence: float | None = None          # Only on final chunk
    error: str | None = None                 # Error message if any
```

**Usage**: Serialized as JSON in SSE `data:` field.

**Example Sequence**:
```
data: {"content": "Physical ", "done": false}
data: {"content": "AI refers ", "done": false}
data: {"content": "to...", "done": false}
data: {"content": "", "done": true, "citations": [...], "confidence": 0.92}
```

---

## Frontend Entities (TypeScript)

### Message
```typescript
interface Message {
  id: string;                    // Unique identifier (UUID)
  role: 'user' | 'assistant';    // Message sender
  content: string;               // Message text
  citations?: Citation[];        // Source references (assistant only)
  confidence?: number;           // Response confidence (assistant only)
  timestamp: Date;               // When message was created
  isStreaming?: boolean;         // True while response is streaming
}
```

### Citation (Frontend)
```typescript
interface Citation {
  chapter: string;
  chapterTitle: string;
  sourceFile: string;
  excerpt: string;
}
```

### ChatState
```typescript
interface ChatState {
  messages: Message[];           // All chat messages
  isLoading: boolean;            // True while waiting for response
  error: string | null;          // Error message if any
  streamingMessageId: string | null; // ID of currently streaming message
}
```

### StreamEvent
```typescript
interface StreamEvent {
  content: string;
  done: boolean;
  citations?: Citation[];
  confidence?: number;
  error?: string;
}
```

---

## Entity Relationships

```
User Input (question)
    │
    ▼
AskRequest ──────────► Backend RAG Pipeline
    │                         │
    │                         ▼
    │                  SearchResult[] (from Qdrant)
    │                         │
    │                         ▼
    │                  RerankedResult[] (from Cohere)
    │                         │
    │                         ▼
    │                  StreamChunk[] (SSE events)
    │                         │
    ▼                         ▼
Message (user) ◄────── Message (assistant)
    │                    │
    │                    ├── content (accumulated)
    │                    ├── citations[]
    │                    └── confidence
    │
    ▼
ChatState.messages[]
```

---

## Validation Rules

### Question Validation
- Minimum length: 3 characters
- Maximum length: 1000 characters
- Non-empty after trimming whitespace

### Response Validation
- Citations excerpt max 200 characters
- Confidence between 0.0 and 1.0
- At least one citation when confidence > 0

### Frontend State Rules
- Only one message can have `isStreaming: true` at a time
- `streamingMessageId` must match an existing message ID or be null
- Messages ordered by timestamp ascending
