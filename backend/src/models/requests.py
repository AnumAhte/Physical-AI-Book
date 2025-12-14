"""Request models for RAG API endpoints."""

from pydantic import BaseModel, Field


class AskRequest(BaseModel):
    """Request body for POST /api/ask endpoint."""

    question: str = Field(..., min_length=3, max_length=1000)
    top_k: int = Field(default=5, ge=1, le=20)

    class Config:
        json_schema_extra = {
            "example": {
                "question": "What is Physical AI?",
                "top_k": 5,
            }
        }


class AskSelectedRequest(BaseModel):
    """Request body for POST /api/ask-selected endpoint."""

    question: str = Field(..., min_length=3, max_length=1000)
    selected_text: str = Field(..., min_length=10, max_length=10000)

    class Config:
        json_schema_extra = {
            "example": {
                "question": "Explain this concept in simpler terms",
                "selected_text": "Physical AI refers to artificial intelligence systems that interact with the physical world through embodied agents...",
            }
        }
