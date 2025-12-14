# Quickstart: RAG Chatbot with OpenAI

**Feature Branch**: `001-rag-openai-chatbot`
**Date**: 2025-12-14

## Prerequisites

- Node.js >= 20.0
- Python >= 3.10
- Git
- API Keys:
  - `OPENAI_API_KEY` - OpenAI API access
  - `COHERE_API_KEY` - Cohere embeddings & rerank
  - `QDRANT_URL` and `QDRANT_API_KEY` - Qdrant Cloud

## Setup

### 1. Clone and Checkout Branch

```bash
git clone https://github.com/your-username/Ai-book.git
cd Ai-book
git checkout 001-rag-openai-chatbot
```

### 2. Backend Setup

```bash
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your API keys:
# - Set LLM_PROVIDER=openai
# - Set OPENAI_API_KEY=your-key
# - Set COHERE_API_KEY=your-key
# - Set QDRANT_URL and QDRANT_API_KEY
```

### 3. Frontend Setup

```bash
cd ..  # Return to root

# Install dependencies
npm install

# No additional configuration needed
# Frontend will connect to backend at localhost:8000
```

## Running Locally

### Start Backend (Terminal 1)

```bash
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn src.main:app --reload --port 8000
```

**Verify**: Open http://localhost:8000/api/health

Expected response:
```json
{
  "status": "healthy",
  "services": {
    "qdrant": "connected",
    "cohere": "connected"
  },
  "version": "1.0.0"
}
```

### Start Frontend (Terminal 2)

```bash
npm start
```

**Verify**: Open http://localhost:3000/chat

## Testing the Chat

1. Navigate to http://localhost:3000/chat
2. Type a question about the textbook (e.g., "What is Physical AI?")
3. Click Send or press Enter
4. Watch the response stream in real-time
5. View source citations below the answer

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/ask` | POST | Synchronous RAG query |
| `/api/ask/stream` | POST | Streaming RAG query (SSE) |
| `/api/ask-selected` | POST | Query about selected text |
| `/api/health` | GET | Health check |

### Test Streaming Endpoint

```bash
curl -X POST http://localhost:8000/api/ask/stream \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}' \
  --no-buffer
```

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| `LLM_PROVIDER` | Yes | Set to `openai` |
| `OPENAI_API_KEY` | Yes | OpenAI API key |
| `COHERE_API_KEY` | Yes | Cohere API key |
| `QDRANT_URL` | Yes | Qdrant Cloud URL |
| `QDRANT_API_KEY` | Yes | Qdrant Cloud API key |
| `LLM_MODEL` | No | Default: `gpt-4o-mini` |
| `SEARCH_TOP_K` | No | Default: `10` |
| `RERANK_TOP_N` | No | Default: `5` |

## Troubleshooting

### "Service unavailable" error
- Check backend is running on port 8000
- Verify API keys in `.env`
- Check Qdrant Cloud connection

### CORS errors in browser
- Ensure backend CORS settings include `http://localhost:3000`
- Check `backend/src/config.py` cors_origins

### Streaming not working
- Verify browser supports `fetch` with `ReadableStream`
- Check network tab for SSE connection
- Ensure `Content-Type: text/event-stream` in response

### Empty responses
- Verify Qdrant collection has textbook embeddings
- Check question relevance to textbook content
- Review backend logs for retrieval issues

## Development Workflow

1. Make backend changes in `backend/src/`
2. Backend auto-reloads with `--reload` flag
3. Make frontend changes in `src/`
4. Frontend hot-reloads automatically
5. Test via chat UI at http://localhost:3000/chat

## Running Tests

### Backend Tests
```bash
cd backend
pytest tests/ -v
```

### Frontend Tests (Manual)
1. Open http://localhost:3000/chat
2. Test various questions
3. Verify streaming behavior
4. Check error handling (disconnect backend, send empty questions)
