"""Response formatting utilities for RAG backend."""

from .models import Citation, RerankedResult


def format_citations(reranked_results: list[RerankedResult]) -> list[Citation]:
    """Format reranked results as citations.

    Args:
        reranked_results: List of RerankedResult from Cohere rerank

    Returns:
        List of Citation objects for the response
    """
    citations = []
    seen_chapters = set()

    for result in reranked_results:
        # Deduplicate by chapter to avoid repetitive citations
        if result.chapter in seen_chapters:
            continue
        seen_chapters.add(result.chapter)

        # Truncate excerpt to 200 chars max (leave room for "...")
        MAX_EXCERPT_LEN = 200
        content = result.content.strip()
        if len(content) <= MAX_EXCERPT_LEN:
            excerpt = content
        else:
            # Truncate to 197 chars to leave room for "..."
            truncated = content[:197]
            # Try to break at a word boundary
            last_space = truncated.rfind(" ")
            if last_space > 150:  # Only break at space if it's not too far back
                excerpt = truncated[:last_space] + "..."
            else:
                excerpt = truncated + "..."

        citations.append(
            Citation(
                chapter=result.chapter,
                chapter_title=result.chapter_title,
                source_file=result.source_file,
                excerpt=excerpt,
            )
        )

    return citations


def calculate_confidence(reranked_results: list[RerankedResult]) -> float:
    """Calculate confidence score from reranked results.

    Uses average of top rerank scores, weighted toward the top result.

    Args:
        reranked_results: List of RerankedResult from Cohere rerank

    Returns:
        Confidence score between 0.0 and 1.0
    """
    if not reranked_results:
        return 0.0

    # Weight top result more heavily
    weights = [1.0]
    for i in range(1, len(reranked_results)):
        weights.append(1.0 / (i + 1))

    total_weight = sum(weights)
    weighted_sum = sum(
        result.relevance_score * weight
        for result, weight in zip(reranked_results, weights)
    )

    confidence = weighted_sum / total_weight

    # Clamp to [0, 1]
    return max(0.0, min(1.0, confidence))


def format_error_message(error_code: str, message: str, details: dict = None) -> dict:
    """Format an error response.

    Args:
        error_code: Error code string
        message: Human-readable error message
        details: Optional additional details

    Returns:
        Error response dictionary
    """
    response = {
        "error": error_code,
        "message": message,
    }
    if details:
        response["details"] = details
    return response
