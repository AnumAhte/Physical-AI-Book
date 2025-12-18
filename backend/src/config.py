"""Configuration management for RAG backend.

Constitution Section VIII: Cohere + Qdrant ONLY (NO SQL)
"""

import os
from functools import lru_cache
from typing import List, Literal

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Cohere API (REQUIRED per Constitution)
    cohere_api_key: str = ""
    cohere_embed_model: str = "embed-english-v3.0"
    cohere_rerank_model: str = "rerank-english-v3.0"

    # Qdrant Cloud (REQUIRED per Constitution)
    qdrant_url: str = ""
    qdrant_api_key: str = ""
    qdrant_collection: str = "textbook"

    # LLM Provider for response generation
    llm_provider: Literal["anthropic", "openai"] = "openai"
    anthropic_api_key: str = ""
    openai_api_key: str = ""
    llm_model: str = "gpt-4o-mini"  # Default for OpenAI; use claude-sonnet-4-20250514 for Anthropic

    # RAG Pipeline settings
    search_top_k: int = 10  # Initial Qdrant search results
    rerank_top_n: int = 5   # Results after Cohere rerank
    chunk_size: int = 500   # Tokens per chunk
    chunk_overlap: int = 100  # Overlap between chunks

    # API settings
    api_title: str = "Physical AI Textbook RAG API"
    api_version: str = "1.0.0"
    # CORS origins - set via env var as JSON array: '["http://localhost:3000", "https://example.com"]'
    cors_origins: List[str] = ["http://localhost:3000", "https://physical-ai-book-one.vercel.app"]

    # Logging
    log_level: str = "INFO"

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        extra = "ignore"


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()


# Convenience accessors
def get_cohere_api_key() -> str:
    """Get Cohere API key."""
    key = get_settings().cohere_api_key
    if not key:
        key = os.getenv("COHERE_API_KEY", "")
    return key


def get_qdrant_url() -> str:
    """Get Qdrant URL."""
    url = get_settings().qdrant_url
    if not url:
        url = os.getenv("QDRANT_URL", "")
    return url


def get_qdrant_api_key() -> str:
    """Get Qdrant API key."""
    key = get_settings().qdrant_api_key
    if not key:
        key = os.getenv("QDRANT_API_KEY", "")
    return key
