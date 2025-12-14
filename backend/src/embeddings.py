"""Cohere embeddings client for RAG backend.

Constitution Section VIII: MUST use Cohere embed-english-v3.0
"""

import time
from functools import lru_cache
from typing import Optional

import cohere
from cohere.core import ApiError

from .config import get_cohere_api_key, get_settings
from .logger import get_logger

logger = get_logger()

# Retry configuration
MAX_RETRIES = 3
INITIAL_BACKOFF = 1.0  # seconds


@lru_cache
def get_cohere_client() -> cohere.Client:
    """Get cached Cohere client instance."""
    api_key = get_cohere_api_key()
    if not api_key:
        raise ValueError("COHERE_API_KEY not configured")
    return cohere.Client(api_key=api_key)


def embed_text(
    text: str,
    model: Optional[str] = None,
) -> list[float]:
    """Embed a single text string for search queries.

    Uses input_type='search_query' per Cohere best practices.

    Args:
        text: The text to embed
        model: Optional model override (default: embed-english-v3.0)

    Returns:
        1024-dimensional embedding vector

    Raises:
        ValueError: If Cohere API key not configured
        ApiError: If Cohere API call fails after retries
    """
    settings = get_settings()
    model = model or settings.cohere_embed_model
    client = get_cohere_client()

    for attempt in range(MAX_RETRIES):
        try:
            logger.info(f"Embedding query text (attempt {attempt + 1})")

            response = client.embed(
                texts=[text],
                model=model,
                input_type="search_query",
            )

            embedding = response.embeddings[0]
            logger.info(f"Successfully embedded query (dim={len(embedding)})")
            return embedding

        except ApiError as e:
            if "rate" in str(e).lower() and attempt < MAX_RETRIES - 1:
                backoff = INITIAL_BACKOFF * (2 ** attempt)
                logger.warning(f"Rate limited, retrying in {backoff}s")
                time.sleep(backoff)
            else:
                logger.error(f"Cohere embed failed: {e}")
                raise

    raise ApiError("Max retries exceeded for Cohere embed")


def embed_documents(
    texts: list[str],
    model: Optional[str] = None,
) -> list[list[float]]:
    """Embed multiple text documents for indexing.

    Uses input_type='search_document' per Cohere best practices.

    Args:
        texts: List of texts to embed
        model: Optional model override (default: embed-english-v3.0)

    Returns:
        List of 1024-dimensional embedding vectors

    Raises:
        ValueError: If Cohere API key not configured
        ApiError: If Cohere API call fails after retries
    """
    settings = get_settings()
    model = model or settings.cohere_embed_model
    client = get_cohere_client()

    for attempt in range(MAX_RETRIES):
        try:
            logger.info(f"Embedding {len(texts)} documents (attempt {attempt + 1})")

            response = client.embed(
                texts=texts,
                model=model,
                input_type="search_document",
            )

            embeddings = response.embeddings
            logger.info(f"Successfully embedded {len(embeddings)} documents")
            return embeddings

        except ApiError as e:
            if "rate" in str(e).lower() and attempt < MAX_RETRIES - 1:
                backoff = INITIAL_BACKOFF * (2 ** attempt)
                logger.warning(f"Rate limited, retrying in {backoff}s")
                time.sleep(backoff)
            else:
                logger.error(f"Cohere embed failed: {e}")
                raise

    raise ApiError("Max retries exceeded for Cohere embed")


async def check_cohere_health() -> bool:
    """Check if Cohere API is accessible.

    Returns:
        True if Cohere is healthy, False otherwise
    """
    try:
        client = get_cohere_client()
        # Small test embed to verify connectivity
        response = client.embed(
            texts=["health check"],
            model=get_settings().cohere_embed_model,
            input_type="search_query",
        )
        return len(response.embeddings) > 0
    except Exception as e:
        logger.error(f"Cohere health check failed: {e}")
        return False
