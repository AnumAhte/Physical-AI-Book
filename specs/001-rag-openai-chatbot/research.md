# Research: RAG Chatbot with OpenAI

**Feature Branch**: `001-rag-openai-chatbot`
**Date**: 2025-12-14

## Executive Summary

This document resolves all technical unknowns for the RAG Chatbot feature. The existing backend already provides a solid foundation - the main additions are streaming support and frontend chat UI.

---

## 1. Streaming Implementation for OpenAI

### Decision
Use OpenAI's native streaming API with FastAPI's `StreamingResponse` and Server-Sent Events (SSE).

### Rationale
- OpenAI SDK (>=1.0) has built-in streaming support via `stream=True` parameter
- FastAPI supports `StreamingResponse` natively for SSE
- SSE is simpler than WebSockets for unidirectional streaming
- Browser `EventSource` API handles SSE natively

### Alternatives Considered
| Alternative | Rejected Because |
|------------|------------------|
| WebSockets | Overkill for unidirectional streaming; requires connection management |
| Polling | Poor UX, higher latency, wasted requests |
| Long-polling | Complex to implement, SSE is more efficient |

### Implementation Pattern
```python
# FastAPI streaming endpoint
from fastapi.responses import StreamingResponse

@router.post("/ask/stream")
async def ask_stream(request: AskRequest):
    async def generate():
        # ... retrieval steps (embed, search, rerank) ...
        async for chunk in openai_stream(question, context):
            yield f"data: {json.dumps({'content': chunk})}\n\n"
        yield f"data: {json.dumps({'done': True, 'citations': citations})}\n\n"

    return StreamingResponse(generate(), media_type="text/event-stream")
```

---

## 2. Frontend Chat UI Framework

### Decision
Build custom React components within Docusaurus (no external chat library).

### Rationale
- Docusaurus 3.x uses React 19 - most chat libraries haven't updated for React 19 compatibility
- ChatKit (Stream) requires paid backend service, not just UI components
- Custom components are lightweight and match existing Docusaurus styling
- Full control over SSE consumption and state management

### Alternatives Considered
| Alternative | Rejected Because |
|------------|------------------|
| @stream-io/chat | Requires Stream backend service (paid), not standalone UI |
| react-chat-elements | Not actively maintained, React 19 compatibility unknown |
| @chatscope/chat-ui-kit-react | Heavy dependency, potential React 19 issues |

### Component Architecture
```
Chat/
├── ChatContainer.tsx    # Main container, manages state
├── MessageList.tsx      # Scrollable message area
├── MessageBubble.tsx    # Individual message display
├── ChatInput.tsx        # Input field + send button
├── TypingIndicator.tsx  # Loading animation
└── SourceCard.tsx       # Citation display
```

---

## 3. SSE Consumption in React

### Decision
Use native `fetch()` with `ReadableStream` for SSE consumption.

### Rationale
- Native browser API, no additional dependencies
- Works with POST requests (EventSource only supports GET)
- Full control over parsing and error handling

### Implementation Pattern
```typescript
const response = await fetch('/api/ask/stream', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ question }),
});

const reader = response.body.getReader();
const decoder = new TextDecoder();

while (true) {
  const { value, done } = await reader.read();
  if (done) break;

  const text = decoder.decode(value);
  // Parse SSE data lines
  const lines = text.split('\n');
  for (const line of lines) {
    if (line.startsWith('data: ')) {
      const data = JSON.parse(line.slice(6));
      // Update state with streamed content
    }
  }
}
```

---

## 4. OpenAI Model Selection

### Decision
Use `gpt-4o-mini` as specified in requirements.

### Rationale
- Cost-effective for high-volume RAG queries
- Fast response times (< 3s first token typical)
- Sufficient quality for textbook Q&A with provided context
- Explicitly specified in feature requirements

### Configuration
```python
# backend/src/config.py addition
llm_model: str = "gpt-4o-mini"  # Override default when LLM_PROVIDER=openai
```

---

## 5. CORS Configuration

### Decision
Extend existing CORS settings to support streaming.

### Rationale
- Backend already has CORS configured for Vercel frontend
- SSE requires same CORS headers as regular requests
- No special streaming-specific CORS needed

### Existing Config (backend/src/config.py:41)
```python
cors_origins: list[str] = ["http://localhost:3000", "https://physical-ai-book-one.vercel.app"]
```

---

## 6. Error Handling Strategy

### Decision
Use structured SSE error events with graceful degradation.

### Rationale
- SSE allows sending error events mid-stream
- Frontend can display partial responses + error message
- Maintains user trust with transparent error reporting

### Error Event Format
```json
{
  "error": true,
  "code": "OPENAI_ERROR",
  "message": "Unable to complete response. Partial answer shown above.",
  "partial": true
}
```

---

## 7. State Management (Frontend)

### Decision
Use React's built-in `useState` and `useReducer` hooks.

### Rationale
- Chat state is local to the chat page (no global state needed)
- Messages array + loading state is simple enough for useState
- No Redux/Zustand overhead for single-page chat

### State Shape
```typescript
interface ChatState {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  streamingContent: string;
}

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  timestamp: Date;
}
```

---

## 8. Backend API Changes Summary

### Existing Endpoints (No Changes)
- `POST /api/ask` - Synchronous RAG endpoint
- `POST /api/ask-selected` - Selected text endpoint
- `GET /api/health` - Health check

### New Endpoint
- `POST /api/ask/stream` - Streaming RAG endpoint

### Modified Files
| File | Change |
|------|--------|
| `backend/src/api.py` | Add `/ask/stream` endpoint |
| `backend/src/llm.py` | Add `generate_response_stream()` function |
| `backend/src/config.py` | Add gpt-4o-mini as default for openai provider |

---

## 9. Dependencies

### Backend (Existing - No New Dependencies)
- `openai>=1.12.0` - Already supports streaming
- `fastapi>=0.109.0` - Already supports StreamingResponse

### Frontend (No New Dependencies)
- Using native fetch API for SSE
- Using existing React 19 from Docusaurus

---

## Resolved Unknowns

| Unknown | Resolution |
|---------|------------|
| Chat UI library | Custom React components (no external library) |
| Streaming protocol | SSE via FastAPI StreamingResponse |
| Frontend SSE consumption | Native fetch + ReadableStream |
| OpenAI model | gpt-4o-mini (per spec) |
| State management | React useState/useReducer |
| Error handling | Structured SSE error events |
