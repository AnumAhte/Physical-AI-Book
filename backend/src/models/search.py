"""SearchResult model for RAG backend.

Represents a search result from Qdrant vector search.
"""

from typing import Optional

from pydantic import BaseModel, Field


class SearchResult(BaseModel):
    """A search result from Qdrant."""

    chunk_id: str
    content: str
    chapter: str
    chapter_title: str
    week: int
    source_file: str
    heading: Optional[str] = None
    score: float = Field(..., ge=0.0, le=1.0)

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "intro_chunk_001",
                "content": "Physical AI refers to artificial intelligence systems...",
                "chapter": "intro",
                "chapter_title": "Introduction to Physical AI",
                "week": 1,
                "source_file": "docs/week-01-02-intro/intro.md",
                "heading": "What is Physical AI?",
                "score": 0.89,
            }
        }
