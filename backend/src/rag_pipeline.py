"""RAG pipeline orchestration.

Implements both:
- General question flow: embed → search → rerank → generate
- Selected text flow: chunk → rerank → generate (NO Qdrant per constitution)
- Streaming flow: embed → search → rerank → stream generate
"""

from typing import AsyncGenerator, Optional

from .chunker import chunk_selected_text
from .config import get_settings
from .embeddings import embed_text
from .formatter import calculate_confidence, format_citations
from .llm import generate_response, generate_response_stream, generate_selected_response
from .logger import get_logger
from .models import Citation, RAGResponse, RerankedResult, StreamChunk
from .rerank import rerank_results, rerank_text_chunks
from .vector_db import search_similar

logger = get_logger()


def ask_question(
    question: str,
    top_k: Optional[int] = None,
) -> RAGResponse:
    """Process a general question through the full RAG pipeline.

    Flow: embed → search → rerank → generate

    Args:
        question: User's question
        top_k: Number of results after reranking (default from config)

    Returns:
        RAGResponse with answer, citations, and confidence
    """
    settings = get_settings()
    search_top_k = settings.search_top_k
    rerank_top_n = top_k or settings.rerank_top_n

    logger.info(f"Processing question: {question[:50]}...")

    # Step 1: Embed the question
    logger.info("Step 1: Embedding question")
    query_embedding = embed_text(question)

    # Step 2: Search Qdrant for similar chunks
    logger.info(f"Step 2: Searching Qdrant (top {search_top_k})")
    search_results = search_similar(query_embedding, top_k=search_top_k)

    if not search_results:
        logger.warning("No search results found")
        return RAGResponse(
            answer="I couldn't find relevant information in the textbook to answer your question. Please try rephrasing or ask about a different topic.",
            citations=[],
            confidence=0.0,
        )

    # Step 3: Rerank results with Cohere
    logger.info(f"Step 3: Reranking to top {rerank_top_n}")
    reranked_results = rerank_results(question, search_results, top_n=rerank_top_n)

    if not reranked_results:
        logger.warning("Reranking returned no results")
        return RAGResponse(
            answer="I couldn't find sufficiently relevant information to answer your question.",
            citations=[],
            confidence=0.0,
        )

    # Step 4: Generate response with LLM
    logger.info("Step 4: Generating response with LLM")
    answer = generate_response(question, reranked_results)

    # Step 5: Format response with citations
    citations = format_citations(reranked_results)
    confidence = calculate_confidence(reranked_results)

    logger.info(f"Generated response with {len(citations)} citations, confidence={confidence:.2f}")

    return RAGResponse(
        answer=answer,
        citations=citations,
        confidence=confidence,
    )


def ask_selected(
    question: str,
    selected_text: str,
    top_n: Optional[int] = None,
) -> RAGResponse:
    """Process a question about selected text.

    Flow: chunk → rerank → generate (NO Qdrant search per constitution)

    Args:
        question: User's question about the selected text
        selected_text: The text highlighted/selected by the user
        top_n: Number of chunks to use after reranking

    Returns:
        RAGResponse with answer, citations, and confidence
    """
    settings = get_settings()
    top_n = top_n or settings.rerank_top_n

    logger.info(f"Processing selected text question: {question[:50]}...")
    logger.info(f"Selected text length: {len(selected_text)} chars")

    # Step 1: Split selected text into chunks
    logger.info("Step 1: Chunking selected text")
    chunks = chunk_selected_text(selected_text)

    if not chunks:
        logger.warning("No chunks created from selected text")
        return RAGResponse(
            answer="The selected text is too short to process. Please select more text.",
            citations=[],
            confidence=0.0,
        )

    # Step 2: Rerank chunks against the question (NO Qdrant!)
    logger.info(f"Step 2: Reranking {len(chunks)} chunks (bypassing Qdrant)")
    reranked = rerank_text_chunks(question, chunks, top_n=top_n)

    if not reranked:
        logger.warning("Reranking returned no results")
        return RAGResponse(
            answer="I couldn't find relevant content in the selected text.",
            citations=[],
            confidence=0.0,
        )

    # Get the most relevant chunks
    relevant_chunks = [chunks[idx] for idx, _ in reranked]
    relevance_scores = [score for _, score in reranked]

    # Step 3: Generate response with LLM
    logger.info("Step 3: Generating response with LLM")
    answer = generate_selected_response(question, selected_text, relevant_chunks)

    # Create citation for selected text
    citations = [
        Citation(
            chapter="selected",
            chapter_title="Selected Text",
            source_file="user_selection",
            excerpt=relevant_chunks[0][:200] if relevant_chunks else selected_text[:200],
        )
    ]

    # Calculate confidence from rerank scores
    confidence = sum(relevance_scores) / len(relevance_scores) if relevance_scores else 0.0

    logger.info(f"Generated selected text response, confidence={confidence:.2f}")

    return RAGResponse(
        answer=answer,
        citations=citations,
        confidence=min(confidence, 1.0),
    )


async def ask_question_stream(
    question: str,
    top_k: Optional[int] = None,
) -> AsyncGenerator[StreamChunk, None]:
    """Process a question through RAG pipeline with streaming response.

    Flow: embed → search → rerank → stream generate

    Args:
        question: User's question
        top_k: Number of results after reranking (default from config)

    Yields:
        StreamChunk objects containing partial content and final metadata
    """
    settings = get_settings()
    search_top_k = settings.search_top_k
    rerank_top_n = top_k or settings.rerank_top_n

    logger.info(f"Processing streaming question: {question[:50]}...")

    # Step 1: Embed the question
    logger.info("Step 1: Embedding question")
    try:
        query_embedding = embed_text(question)
    except Exception as e:
        logger.error(f"Embedding error: {e}")
        yield StreamChunk(
            content="",
            done=True,
            error="Unable to process your question. Please try again.",
        )
        return

    # Step 2: Search Qdrant for similar chunks
    logger.info(f"Step 2: Searching Qdrant (top {search_top_k})")
    try:
        search_results = search_similar(query_embedding, top_k=search_top_k)
    except Exception as e:
        logger.error(f"Search error: {e}")
        yield StreamChunk(
            content="",
            done=True,
            error="Search service temporarily unavailable. Please try again.",
        )
        return

    if not search_results:
        logger.warning("No search results found")
        yield StreamChunk(
            content="I couldn't find relevant information in the textbook to answer your question. Please try rephrasing or ask about a different topic covered in the textbook.",
            done=True,
            citations=[],
            confidence=0.0,
        )
        return

    # Step 3: Rerank results with Cohere
    logger.info(f"Step 3: Reranking to top {rerank_top_n}")
    try:
        reranked_results = rerank_results(question, search_results, top_n=rerank_top_n)
    except Exception as e:
        logger.error(f"Rerank error: {e}")
        yield StreamChunk(
            content="",
            done=True,
            error="Unable to process results. Please try again.",
        )
        return

    if not reranked_results:
        logger.warning("Reranking returned no results")
        yield StreamChunk(
            content="I couldn't find sufficiently relevant information to answer your question.",
            done=True,
            citations=[],
            confidence=0.0,
        )
        return

    # Pre-calculate citations and confidence for final chunk
    citations = format_citations(reranked_results)
    confidence = calculate_confidence(reranked_results)

    # Check for low confidence - threshold 0.4 indicates out-of-scope question
    LOW_CONFIDENCE_THRESHOLD = 0.4
    is_low_confidence = confidence < LOW_CONFIDENCE_THRESHOLD
    if is_low_confidence:
        logger.warning(f"Low confidence ({confidence:.2f}) - may be out-of-scope question")

    # Step 4: Stream response with LLM
    logger.info("Step 4: Streaming response with LLM")
    try:
        async for text_chunk in generate_response_stream(question, reranked_results):
            yield StreamChunk(content=text_chunk, done=False)

        # Final chunk with metadata
        yield StreamChunk(
            content="",
            done=True,
            citations=citations,
            confidence=confidence,
            lowConfidence=is_low_confidence,
        )

        logger.info(f"Streamed response with {len(citations)} citations, confidence={confidence:.2f}")

    except Exception as e:
        logger.error(f"Streaming generation error: {e}")
        yield StreamChunk(
            content="",
            done=True,
            error="Unable to generate response. Please try again.",
        )
