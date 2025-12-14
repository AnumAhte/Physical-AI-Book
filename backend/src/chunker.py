"""Text chunking for RAG backend.

Chunks markdown content for embedding and storage in Qdrant.
Uses 500 tokens per chunk with 100 token overlap per research.md.
"""

import re
from typing import Optional

from langchain_text_splitters import RecursiveCharacterTextSplitter

from .logger import get_logger
from .models import TextChunk

logger = get_logger()

# Default chunking parameters per research.md
DEFAULT_CHUNK_SIZE = 500  # tokens
DEFAULT_CHUNK_OVERLAP = 100  # tokens
CHARS_PER_TOKEN = 4  # Approximate


def chunk_markdown(
    content: str,
    source_file: str,
    chapter: str,
    chapter_title: str,
    week: int,
    chunk_size: int = DEFAULT_CHUNK_SIZE,
    chunk_overlap: int = DEFAULT_CHUNK_OVERLAP,
) -> list[TextChunk]:
    """Split markdown content into chunks.

    Args:
        content: Raw markdown content
        source_file: Path to source file
        chapter: Chapter slug (e.g., "intro", "ros2-overview")
        chapter_title: Human-readable chapter title
        week: Course week number (1-13)
        chunk_size: Target tokens per chunk
        chunk_overlap: Overlap tokens between chunks

    Returns:
        List of TextChunk objects ready for embedding
    """
    logger.info(f"Chunking {source_file} (chapter={chapter}, week={week})")

    # Convert token counts to character counts
    char_chunk_size = chunk_size * CHARS_PER_TOKEN
    char_overlap = chunk_overlap * CHARS_PER_TOKEN

    # Use RecursiveCharacterTextSplitter for markdown
    splitter = RecursiveCharacterTextSplitter(
        chunk_size=char_chunk_size,
        chunk_overlap=char_overlap,
        separators=["\n## ", "\n### ", "\n#### ", "\n\n", "\n", " "],
        keep_separator=True,
    )

    # Split the content
    texts = splitter.split_text(content)

    # Extract headings for each chunk
    chunks = []
    for i, text in enumerate(texts):
        heading = extract_nearest_heading(content, text)
        char_start = content.find(text[:50]) if len(text) >= 50 else content.find(text)

        chunk = TextChunk(
            chunk_id=f"{chapter}_chunk_{i:03d}",
            content=text.strip(),
            chapter=chapter,
            chapter_title=chapter_title,
            week=week,
            source_file=source_file,
            heading=heading,
            char_start=char_start if char_start >= 0 else None,
            char_end=char_start + len(text) if char_start >= 0 else None,
        )
        chunks.append(chunk)

    logger.info(f"Created {len(chunks)} chunks from {source_file}")
    return chunks


def extract_nearest_heading(content: str, chunk_text: str) -> Optional[str]:
    """Extract the nearest heading above a chunk.

    Args:
        content: Full markdown content
        chunk_text: The chunk text to find heading for

    Returns:
        Nearest heading text or None
    """
    # Find chunk position
    chunk_start = content.find(chunk_text[:100]) if len(chunk_text) >= 100 else content.find(chunk_text)
    if chunk_start < 0:
        return None

    # Look backwards for heading
    content_before = content[:chunk_start]

    # Find all headings
    heading_pattern = r"^(#{1,4})\s+(.+)$"
    headings = list(re.finditer(heading_pattern, content_before, re.MULTILINE))

    if headings:
        last_heading = headings[-1]
        return last_heading.group(2).strip()

    return None


def extract_metadata_from_path(path: str) -> dict:
    """Extract metadata from file path.

    Expected format: docs/week-XX-YY-topic/filename.md

    Args:
        path: File path

    Returns:
        Dictionary with extracted metadata
    """
    metadata = {"week": 1, "chapter": "unknown"}

    # Extract week number from path
    week_match = re.search(r"week-(\d+)", path)
    if week_match:
        metadata["week"] = int(week_match.group(1))

    # Extract chapter from filename or directory
    parts = path.replace("\\", "/").split("/")
    if len(parts) >= 2:
        # Use directory name as chapter
        dir_name = parts[-2]
        chapter_match = re.search(r"week-\d+-\d+-(.+)", dir_name)
        if chapter_match:
            metadata["chapter"] = chapter_match.group(1)
        else:
            metadata["chapter"] = dir_name

    # Or use filename without extension
    if parts:
        filename = parts[-1].replace(".md", "")
        if filename not in ["index", "README"]:
            metadata["chapter"] = filename

    return metadata


def chunk_selected_text(
    text: str,
    chunk_size: int = DEFAULT_CHUNK_SIZE,
) -> list[str]:
    """Split selected text into chunks for reranking.

    Used in the ask-selected flow which bypasses Qdrant.

    Args:
        text: User-selected text
        chunk_size: Target tokens per chunk

    Returns:
        List of text chunks
    """
    char_chunk_size = chunk_size * CHARS_PER_TOKEN

    splitter = RecursiveCharacterTextSplitter(
        chunk_size=char_chunk_size,
        chunk_overlap=50,  # Small overlap for selected text
        separators=["\n\n", "\n", ". ", " "],
    )

    return splitter.split_text(text)
