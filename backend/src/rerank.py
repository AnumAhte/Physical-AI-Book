"""Cohere rerank client for RAG backend.

Constitution Section VIII: MUST use Cohere rerank-english-v3.0
"""

import time
from typing import Optional

import cohere
from cohere.core import ApiError

from .config import get_settings
from .embeddings import get_cohere_client
from .logger import get_logger
from .models import RerankedResult, SearchResult

logger = get_logger()

# Retry configuration
MAX_RETRIES = 3
INITIAL_BACKOFF = 1.0  # seconds


def rerank_results(
    query: str,
    search_results: list[SearchResult],
    top_n: Optional[int] = None,
    model: Optional[str] = None,
) -> list[RerankedResult]:
    """Rerank search results using Cohere rerank.

    Args:
        query: The user's question
        search_results: List of SearchResult from Qdrant
        top_n: Number of top results to return (default: from settings)
        model: Optional model override (default: rerank-english-v3.0)

    Returns:
        List of RerankedResult sorted by relevance

    Raises:
        ApiError: If Cohere API call fails after retries
    """
    settings = get_settings()
    model = model or settings.cohere_rerank_model
    top_n = top_n or settings.rerank_top_n
    client = get_cohere_client()

    # Extract documents for reranking
    documents = [result.content for result in search_results]

    for attempt in range(MAX_RETRIES):
        try:
            logger.info(
                f"Reranking {len(documents)} results (attempt {attempt + 1}, top_n={top_n})"
            )

            response = client.rerank(
                model=model,
                query=query,
                documents=documents,
                top_n=top_n,
            )

            # Map reranked results back to original search results
            reranked = []
            for result in response.results:
                original = search_results[result.index]
                reranked.append(
                    RerankedResult(
                        chunk_id=original.chunk_id,
                        content=original.content,
                        chapter=original.chapter,
                        chapter_title=original.chapter_title,
                        source_file=original.source_file,
                        relevance_score=result.relevance_score,
                    )
                )

            logger.info(f"Successfully reranked to {len(reranked)} results")
            return reranked

        except ApiError as e:
            if "rate" in str(e).lower() and attempt < MAX_RETRIES - 1:
                backoff = INITIAL_BACKOFF * (2 ** attempt)
                logger.warning(f"Rate limited, retrying in {backoff}s")
                time.sleep(backoff)
            else:
                logger.error(f"Cohere rerank failed: {e}")
                raise

    raise ApiError("Max retries exceeded for Cohere rerank")


def rerank_text_chunks(
    query: str,
    chunks: list[str],
    top_n: Optional[int] = None,
    model: Optional[str] = None,
) -> list[tuple[int, float]]:
    """Rerank raw text chunks (for selected text flow).

    This function is used when bypassing Qdrant for selected text.

    Args:
        query: The user's question
        chunks: List of text chunks from selected text
        top_n: Number of top results to return
        model: Optional model override

    Returns:
        List of (index, relevance_score) tuples sorted by relevance
    """
    settings = get_settings()
    model = model or settings.cohere_rerank_model
    top_n = top_n or settings.rerank_top_n
    client = get_cohere_client()

    for attempt in range(MAX_RETRIES):
        try:
            logger.info(
                f"Reranking {len(chunks)} text chunks (attempt {attempt + 1})"
            )

            response = client.rerank(
                model=model,
                query=query,
                documents=chunks,
                top_n=top_n,
            )

            results = [(r.index, r.relevance_score) for r in response.results]
            logger.info(f"Successfully reranked {len(results)} chunks")
            return results

        except ApiError as e:
            if "rate" in str(e).lower() and attempt < MAX_RETRIES - 1:
                backoff = INITIAL_BACKOFF * (2 ** attempt)
                logger.warning(f"Rate limited, retrying in {backoff}s")
                time.sleep(backoff)
            else:
                logger.error(f"Cohere rerank failed: {e}")
                raise

    raise ApiError("Max retries exceeded for Cohere rerank")
