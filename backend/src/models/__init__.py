"""Pydantic models for RAG backend.

Constitution Section VIII: Cohere + Qdrant ONLY (NO SQL)
All models are Pydantic data classes, NOT SQLAlchemy ORM models.
"""

from .chunk import TextChunk
from .rerank import RerankedResult
from .requests import AskRequest, AskSelectedRequest
from .responses import Citation, ErrorResponse, HealthResponse, RAGResponse, StreamChunk
from .search import SearchResult

__all__ = [
    "TextChunk",
    "SearchResult",
    "RerankedResult",
    "AskRequest",
    "AskSelectedRequest",
    "RAGResponse",
    "Citation",
    "ErrorResponse",
    "HealthResponse",
    "StreamChunk",
]
