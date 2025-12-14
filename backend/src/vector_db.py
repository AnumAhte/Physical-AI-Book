"""Qdrant vector database client for RAG backend.

Constitution Section VIII: Qdrant is the ONLY storage allowed (NO SQL)
"""

import uuid
from functools import lru_cache
from typing import Optional

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    FieldCondition,
    Filter,
    PointStruct,
    Range,
    VectorParams,
)

from .config import get_qdrant_api_key, get_qdrant_url, get_settings
from .logger import get_logger
from .models import SearchResult, TextChunk

logger = get_logger()

# Vector dimensions for Cohere embed-english-v3.0
VECTOR_SIZE = 1024


@lru_cache
def get_qdrant_client() -> QdrantClient:
    """Get cached Qdrant client instance."""
    url = get_qdrant_url()
    api_key = get_qdrant_api_key()

    if not url:
        raise ValueError("QDRANT_URL not configured")

    logger.info(f"Connecting to Qdrant at {url}")
    return QdrantClient(url=url, api_key=api_key)


def create_collection(collection_name: Optional[str] = None) -> None:
    """Create or recreate the textbook collection.

    Args:
        collection_name: Optional collection name override
    """
    settings = get_settings()
    collection_name = collection_name or settings.qdrant_collection
    client = get_qdrant_client()

    logger.info(f"Creating collection '{collection_name}' with {VECTOR_SIZE} dimensions")

    client.recreate_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=VECTOR_SIZE,
            distance=Distance.COSINE,
        ),
    )

    logger.info(f"Collection '{collection_name}' created successfully")


def search_similar(
    query_embedding: list[float],
    top_k: int = 10,
    week_filter: Optional[int] = None,
    chapter_filter: Optional[str] = None,
    collection_name: Optional[str] = None,
) -> list[SearchResult]:
    """Search for similar chunks in Qdrant.

    Args:
        query_embedding: 1024-dimensional query vector
        top_k: Number of results to return
        week_filter: Optional filter by week number
        chapter_filter: Optional filter by chapter slug
        collection_name: Optional collection name override

    Returns:
        List of SearchResult sorted by similarity
    """
    settings = get_settings()
    collection_name = collection_name or settings.qdrant_collection
    client = get_qdrant_client()

    # Build filter conditions
    filter_conditions = []
    if week_filter is not None:
        filter_conditions.append(
            FieldCondition(key="week", range=Range(gte=week_filter, lte=week_filter))
        )
    if chapter_filter is not None:
        filter_conditions.append(
            FieldCondition(key="chapter", match={"value": chapter_filter})
        )

    query_filter = Filter(must=filter_conditions) if filter_conditions else None

    logger.info(f"Searching Qdrant for top {top_k} results")

    response = client.query_points(
        collection_name=collection_name,
        query=query_embedding,
        query_filter=query_filter,
        limit=top_k,
        with_payload=True,
    )

    search_results = []
    for point in response.points:
        payload = point.payload or {}
        search_results.append(
            SearchResult(
                chunk_id=payload.get("chunk_id", str(point.id)),
                content=payload.get("content", ""),
                chapter=payload.get("chapter", ""),
                chapter_title=payload.get("chapter_title", ""),
                week=payload.get("week", 1),
                source_file=payload.get("source_file", ""),
                heading=payload.get("heading"),
                score=point.score,
            )
        )

    logger.info(f"Found {len(search_results)} results")
    return search_results


def upsert_chunks(
    chunks: list[TextChunk],
    embeddings: list[list[float]],
    collection_name: Optional[str] = None,
) -> int:
    """Insert or update chunks in Qdrant.

    Args:
        chunks: List of TextChunk objects
        embeddings: Corresponding embedding vectors
        collection_name: Optional collection name override

    Returns:
        Number of points upserted
    """
    settings = get_settings()
    collection_name = collection_name or settings.qdrant_collection
    client = get_qdrant_client()

    if len(chunks) != len(embeddings):
        raise ValueError("Number of chunks must match number of embeddings")

    points = [
        PointStruct(
            id=str(uuid.uuid5(uuid.NAMESPACE_DNS, chunk.chunk_id)),
            vector=embedding,
            payload=chunk.to_qdrant_payload(),
        )
        for chunk, embedding in zip(chunks, embeddings)
    ]

    logger.info(f"Upserting {len(points)} points to '{collection_name}'")

    client.upsert(
        collection_name=collection_name,
        points=points,
    )

    logger.info(f"Successfully upserted {len(points)} points")
    return len(points)


async def check_qdrant_health() -> bool:
    """Check if Qdrant is accessible.

    Returns:
        True if Qdrant is healthy, False otherwise
    """
    try:
        client = get_qdrant_client()
        # Get collection info to verify connectivity
        settings = get_settings()
        client.get_collection(settings.qdrant_collection)
        return True
    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")
        return False


def get_collection_info() -> dict:
    """Get information about the textbook collection.

    Returns:
        Dictionary with collection statistics
    """
    settings = get_settings()
    client = get_qdrant_client()

    try:
        info = client.get_collection(settings.qdrant_collection)
        return {
            "name": settings.qdrant_collection,
            "vectors_count": info.vectors_count,
            "points_count": info.points_count,
            "status": info.status,
        }
    except Exception as e:
        logger.error(f"Failed to get collection info: {e}")
        return {"error": str(e)}
