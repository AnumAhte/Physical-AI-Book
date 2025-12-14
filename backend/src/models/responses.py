"""Response models for RAG API endpoints."""

from typing import Literal, Optional

from pydantic import BaseModel, Field


class Citation(BaseModel):
    """A citation reference in the response."""

    chapter: str
    chapter_title: str
    source_file: str
    excerpt: str = Field(..., max_length=200)


class RAGResponse(BaseModel):
    """Response from RAG endpoints."""

    answer: str
    citations: list[Citation]
    confidence: float = Field(..., ge=0.0, le=1.0)

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "Physical AI refers to AI systems that can perceive and interact with the real world through sensors and actuators...",
                "citations": [
                    {
                        "chapter": "intro",
                        "chapter_title": "Introduction to Physical AI",
                        "source_file": "docs/week-01-02-intro/intro.md",
                        "excerpt": "Physical AI refers to artificial intelligence systems that...",
                    }
                ],
                "confidence": 0.92,
            }
        }


class ErrorResponse(BaseModel):
    """Error response from API."""

    error: str
    message: str
    details: Optional[dict] = None

    class Config:
        json_schema_extra = {
            "example": {
                "error": "RATE_LIMIT_EXCEEDED",
                "message": "Too many requests. Please try again later.",
                "details": {"retry_after": 60},
            }
        }


class HealthResponse(BaseModel):
    """Response from health check endpoint."""

    status: Literal["healthy", "degraded", "unhealthy"]
    services: dict[str, Literal["connected", "disconnected"]]
    version: str

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "services": {
                    "qdrant": "connected",
                    "cohere": "connected",
                },
                "version": "1.0.0",
            }
        }


class StreamChunk(BaseModel):
    """A chunk of streamed response content for SSE."""

    content: str = ""
    done: bool = False
    citations: Optional[list[Citation]] = None
    confidence: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    lowConfidence: bool = False  # True when confidence < threshold (out-of-scope indicator)
    error: Optional[str] = None

    class Config:
        json_schema_extra = {
            "example": {
                "content": "Physical AI refers to ",
                "done": False,
                "citations": None,
                "confidence": None,
                "lowConfidence": False,
                "error": None,
            }
        }
