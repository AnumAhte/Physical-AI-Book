"""TextChunk model for RAG backend.

Represents a chunk of text from the textbook stored in Qdrant.
"""

import uuid
from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


class TextChunk(BaseModel):
    """A chunk of text from the textbook."""

    chunk_id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    content: str = Field(..., min_length=1, max_length=5000)
    chapter: str = Field(..., pattern=r"^[a-z0-9-]+$")
    chapter_title: str
    week: int = Field(..., ge=1, le=13)
    source_file: str
    heading: Optional[str] = None
    char_start: Optional[int] = None
    char_end: Optional[int] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "intro_chunk_001",
                "content": "Physical AI refers to artificial intelligence systems that interact with the physical world...",
                "chapter": "intro",
                "chapter_title": "Introduction to Physical AI",
                "week": 1,
                "source_file": "docs/week-01-02-intro/intro.md",
                "heading": "What is Physical AI?",
                "char_start": 0,
                "char_end": 500,
            }
        }

    def to_qdrant_payload(self) -> dict:
        """Convert to Qdrant payload format."""
        return {
            "chunk_id": self.chunk_id,
            "content": self.content,
            "chapter": self.chapter,
            "chapter_title": self.chapter_title,
            "week": self.week,
            "source_file": self.source_file,
            "heading": self.heading,
            "char_start": self.char_start,
            "char_end": self.char_end,
            "created_at": self.created_at.isoformat(),
        }
