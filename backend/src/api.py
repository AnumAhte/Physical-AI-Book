"""API routes for RAG backend.

Implements:
- POST /api/ask - General question endpoint
- POST /api/ask/stream - Streaming question endpoint (SSE)
- POST /api/ask-selected - Selected text endpoint
- GET /api/health - Health check endpoint
"""

import json

from fastapi import APIRouter, HTTPException, status
from fastapi.responses import StreamingResponse

from .config import get_settings
from .embeddings import check_cohere_health
from .logger import get_logger
from .models import (
    AskRequest,
    AskSelectedRequest,
    ErrorResponse,
    HealthResponse,
    RAGResponse,
)
from .rag_pipeline import ask_question, ask_question_stream, ask_selected
from .vector_db import check_qdrant_health

logger = get_logger()

router = APIRouter(prefix="/api", tags=["RAG"])


@router.post(
    "/ask",
    response_model=RAGResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        429: {"model": ErrorResponse, "description": "Rate limit exceeded"},
        503: {"model": ErrorResponse, "description": "Service unavailable"},
    },
)
async def ask_endpoint(request: AskRequest) -> RAGResponse:
    """Ask a question about the textbook.

    Performs the full RAG pipeline:
    1. Cohere embedding of the question
    2. Qdrant vector search (top 10)
    3. Cohere rerank (top 5)
    4. LLM response generation with citations
    """
    logger.info(f"POST /api/ask: {request.question[:50]}...")

    try:
        response = ask_question(
            question=request.question,
            top_k=request.top_k,
        )
        return response

    except ValueError as e:
        logger.error(f"Validation error: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "INVALID_REQUEST", "message": str(e)},
        )

    except Exception as e:
        error_msg = str(e).lower()
        if "rate" in error_msg:
            logger.warning(f"Rate limit exceeded: {e}")
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "error": "RATE_LIMIT_EXCEEDED",
                    "message": "Too many requests. Please try again later.",
                },
            )

        logger.error(f"Service error: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail={
                "error": "SERVICE_UNAVAILABLE",
                "message": "Unable to process request. Please try again.",
            },
        )


@router.post(
    "/ask/stream",
    responses={
        200: {"content": {"text/event-stream": {}}, "description": "Streaming SSE response"},
        400: {"model": ErrorResponse, "description": "Invalid request"},
        429: {"model": ErrorResponse, "description": "Rate limit exceeded"},
        503: {"model": ErrorResponse, "description": "Service unavailable"},
    },
)
async def ask_stream_endpoint(request: AskRequest) -> StreamingResponse:
    """Ask a question with streaming response via Server-Sent Events.

    Performs the full RAG pipeline with streaming:
    1. Cohere embedding of the question
    2. Qdrant vector search (top 10)
    3. Cohere rerank (top 5)
    4. OpenAI streaming response generation

    Returns SSE stream with JSON chunks:
    - Content chunks: {"content": "...", "done": false}
    - Final chunk: {"content": "", "done": true, "citations": [...], "confidence": 0.92}
    - Error chunk: {"content": "", "done": true, "error": "..."}
    """
    logger.info(f"POST /api/ask/stream: {request.question[:50]}...")

    async def generate_sse():
        """Generate Server-Sent Events from the streaming pipeline."""
        try:
            async for chunk in ask_question_stream(
                question=request.question,
                top_k=request.top_k,
            ):
                # Serialize chunk to JSON and format as SSE
                chunk_json = chunk.model_dump_json()
                yield f"data: {chunk_json}\n\n"

        except ValueError as e:
            logger.error(f"Validation error in stream: {e}")
            error_chunk = {"content": "", "done": True, "error": str(e)}
            yield f"data: {json.dumps(error_chunk)}\n\n"

        except Exception as e:
            error_msg = str(e).lower()
            if "rate" in error_msg:
                logger.warning(f"Rate limit exceeded in stream: {e}")
                error_chunk = {"content": "", "done": True, "error": "Rate limit exceeded. Please try again later."}
            else:
                logger.error(f"Service error in stream: {e}")
                error_chunk = {"content": "", "done": True, "error": "Unable to process request. Please try again."}
            yield f"data: {json.dumps(error_chunk)}\n\n"

    return StreamingResponse(
        generate_sse(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",  # Disable nginx buffering
        },
    )


@router.post(
    "/ask-selected",
    response_model=RAGResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        429: {"model": ErrorResponse, "description": "Rate limit exceeded"},
    },
)
async def ask_selected_endpoint(request: AskSelectedRequest) -> RAGResponse:
    """Ask a question about selected/highlighted text.

    Bypasses Qdrant search per constitution.
    Performs:
    1. Split selected text into chunks
    2. Cohere rerank: question vs chunks
    3. LLM response from top-ranked chunks only
    """
    logger.info(f"POST /api/ask-selected: {request.question[:50]}...")

    try:
        response = ask_selected(
            question=request.question,
            selected_text=request.selected_text,
        )
        return response

    except ValueError as e:
        logger.error(f"Validation error: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": "INVALID_REQUEST", "message": str(e)},
        )

    except Exception as e:
        error_msg = str(e).lower()
        if "rate" in error_msg:
            logger.warning(f"Rate limit exceeded: {e}")
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "error": "RATE_LIMIT_EXCEEDED",
                    "message": "Too many requests. Please try again later.",
                },
            )

        logger.error(f"Service error: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail={
                "error": "SERVICE_UNAVAILABLE",
                "message": "Unable to process request. Please try again.",
            },
        )


@router.get(
    "/health",
    response_model=HealthResponse,
    responses={
        503: {"model": HealthResponse, "description": "Service unhealthy"},
    },
)
async def health_endpoint() -> HealthResponse:
    """Check if the API and its dependencies are healthy."""
    settings = get_settings()

    # Check services
    qdrant_healthy = await check_qdrant_health()
    cohere_healthy = await check_cohere_health()

    services = {
        "qdrant": "connected" if qdrant_healthy else "disconnected",
        "cohere": "connected" if cohere_healthy else "disconnected",
    }

    # Determine overall status
    if qdrant_healthy and cohere_healthy:
        status_val = "healthy"
    elif qdrant_healthy or cohere_healthy:
        status_val = "degraded"
    else:
        status_val = "unhealthy"

    response = HealthResponse(
        status=status_val,
        services=services,
        version=settings.api_version,
    )

    if status_val == "unhealthy":
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=response.model_dump(),
        )

    return response
