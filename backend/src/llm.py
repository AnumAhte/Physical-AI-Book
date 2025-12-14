"""LLM client for response generation.

Supports both Anthropic Claude and OpenAI GPT-4 via configuration.
Includes streaming support for OpenAI.
"""

from typing import AsyncGenerator, Optional

from .config import get_settings
from .logger import get_logger
from .models import RerankedResult

logger = get_logger()


def get_llm_client():
    """Get the configured LLM client."""
    settings = get_settings()

    if settings.llm_provider == "anthropic":
        import anthropic

        api_key = settings.anthropic_api_key
        if not api_key:
            import os
            api_key = os.getenv("ANTHROPIC_API_KEY", "")
        return anthropic.Anthropic(api_key=api_key)
    else:
        import openai

        api_key = settings.openai_api_key
        if not api_key:
            import os
            api_key = os.getenv("OPENAI_API_KEY", "")
        return openai.OpenAI(api_key=api_key)


def generate_response(
    question: str,
    context_chunks: list[RerankedResult],
    max_tokens: int = 1024,
) -> str:
    """Generate a response using the LLM.

    Args:
        question: User's question
        context_chunks: Relevant chunks from retrieval
        max_tokens: Maximum response tokens

    Returns:
        Generated response text
    """
    settings = get_settings()

    # Build context from chunks
    context_parts = []
    for i, chunk in enumerate(context_chunks, 1):
        context_parts.append(
            f"[{i}] From {chunk.chapter_title} ({chunk.source_file}):\n{chunk.content}"
        )
    context = "\n\n".join(context_parts)

    # Build the prompt
    system_prompt = """You are a helpful assistant for a Physical AI and Humanoid Robotics textbook.
Answer questions ONLY based on the provided context. If the context doesn't contain enough information to answer, say so.
Always cite which source(s) you used in your answer.
Be concise and accurate. Do not make up information."""

    user_prompt = f"""Context from the textbook:

{context}

Question: {question}

Please provide a clear, accurate answer based only on the context above."""

    logger.info(f"Generating response with {settings.llm_provider}")

    if settings.llm_provider == "anthropic":
        return _generate_anthropic(system_prompt, user_prompt, max_tokens)
    else:
        return _generate_openai(system_prompt, user_prompt, max_tokens)


def _generate_anthropic(
    system_prompt: str,
    user_prompt: str,
    max_tokens: int,
) -> str:
    """Generate response using Anthropic Claude."""
    settings = get_settings()
    client = get_llm_client()

    response = client.messages.create(
        model=settings.llm_model,
        max_tokens=max_tokens,
        system=system_prompt,
        messages=[
            {"role": "user", "content": user_prompt}
        ],
    )

    return response.content[0].text


def _generate_openai(
    system_prompt: str,
    user_prompt: str,
    max_tokens: int,
) -> str:
    """Generate response using OpenAI GPT."""
    settings = get_settings()
    client = get_llm_client()

    # Map to OpenAI model name if using Anthropic default
    model = settings.llm_model
    if "claude" in model.lower():
        model = "gpt-4-turbo-preview"

    response = client.chat.completions.create(
        model=model,
        max_tokens=max_tokens,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt},
        ],
    )

    return response.choices[0].message.content


def generate_selected_response(
    question: str,
    selected_text: str,
    relevant_chunks: list[str],
    max_tokens: int = 1024,
) -> str:
    """Generate a response for selected text queries.

    Args:
        question: User's question about the selected text
        selected_text: The text the user selected
        relevant_chunks: Most relevant chunks after reranking
        max_tokens: Maximum response tokens

    Returns:
        Generated response text
    """
    settings = get_settings()

    # Build context from relevant chunks
    context = "\n\n".join(relevant_chunks)

    system_prompt = """You are a helpful assistant explaining content from a Physical AI textbook.
The user has selected some text and has a question about it.
Answer based ONLY on the selected text. Be helpful and explain clearly."""

    user_prompt = f"""Selected text from the textbook:

{context}

User's question: {question}

Please answer the question based on the selected text above."""

    logger.info(f"Generating selected text response with {settings.llm_provider}")

    if settings.llm_provider == "anthropic":
        return _generate_anthropic(system_prompt, user_prompt, max_tokens)
    else:
        return _generate_openai(system_prompt, user_prompt, max_tokens)


async def generate_response_stream(
    question: str,
    context_chunks: list[RerankedResult],
    max_tokens: int = 1024,
) -> AsyncGenerator[str, None]:
    """Generate a streaming response using OpenAI.

    Args:
        question: User's question
        context_chunks: Relevant chunks from retrieval
        max_tokens: Maximum response tokens

    Yields:
        Chunks of generated response text
    """
    settings = get_settings()

    # Build context from chunks
    context_parts = []
    for i, chunk in enumerate(context_chunks, 1):
        context_parts.append(
            f"[{i}] From {chunk.chapter_title} ({chunk.source_file}):\n{chunk.content}"
        )
    context = "\n\n".join(context_parts)

    # Build the prompt
    system_prompt = """You are a helpful assistant for a Physical AI and Humanoid Robotics textbook.

SCOPE: You can ONLY answer questions about topics covered in this textbook, including:
- Physical AI concepts and embodied intelligence
- Humanoid robotics and robot morphology
- Sensors, actuators, and perception systems
- Motion planning, control, and locomotion
- Machine learning for robotics
- Simulation and real-world deployment

RULES:
1. Answer questions ONLY based on the provided context from the textbook.
2. If the context doesn't contain enough information to answer, say: "I don't have enough information in the textbook to answer that question. Try asking about [suggest related topic from context]."
3. Always cite your sources using reference numbers like [1], [2].
4. Be concise and accurate. Never make up information.

OUT-OF-SCOPE QUESTIONS:
If the question is unrelated to the textbook content (e.g., "What is the capital of France?", general trivia, current events, personal advice), respond with:
"I'm designed to answer questions about the Physical AI and Humanoid Robotics textbook. Your question appears to be outside this scope. Please ask me about topics like robot perception, motion planning, humanoid design, or other Physical AI concepts covered in the textbook."

Do NOT attempt to answer out-of-scope questions even if you know the answer."""

    user_prompt = f"""Context from the textbook:

{context}

Question: {question}

Please provide a clear, accurate answer based only on the context above."""

    logger.info(f"Generating streaming response with {settings.llm_provider}")

    if settings.llm_provider != "openai":
        # Fallback to non-streaming for Anthropic
        response = _generate_anthropic(system_prompt, user_prompt, max_tokens)
        yield response
        return

    # OpenAI streaming
    import openai

    api_key = settings.openai_api_key
    if not api_key:
        import os
        api_key = os.getenv("OPENAI_API_KEY", "")

    client = openai.OpenAI(api_key=api_key)

    # Determine model
    model = settings.llm_model
    if "claude" in model.lower():
        model = "gpt-4o-mini"

    try:
        stream = client.chat.completions.create(
            model=model,
            max_tokens=max_tokens,
            stream=True,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
        )

        for chunk in stream:
            if chunk.choices[0].delta.content is not None:
                yield chunk.choices[0].delta.content

    except Exception as e:
        logger.error(f"Streaming error: {e}")
        raise
