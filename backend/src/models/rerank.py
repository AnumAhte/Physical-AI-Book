"""RerankedResult model for RAG backend.

Represents a reranked search result from Cohere.
"""

from pydantic import BaseModel, Field


class RerankedResult(BaseModel):
    """A reranked search result from Cohere."""

    chunk_id: str
    content: str
    chapter: str
    chapter_title: str
    source_file: str
    relevance_score: float = Field(..., ge=0.0, le=1.0)

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "intro_chunk_001",
                "content": "Physical AI refers to artificial intelligence systems...",
                "chapter": "intro",
                "chapter_title": "Introduction to Physical AI",
                "source_file": "docs/week-01-02-intro/intro.md",
                "relevance_score": 0.95,
            }
        }
