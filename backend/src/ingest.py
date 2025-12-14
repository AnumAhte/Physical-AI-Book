"""Content ingestion script for RAG backend.

Ingests markdown textbook content into Qdrant via Cohere embeddings.
Usage: python -m src.ingest --docs-path ../docs
"""

import argparse
import os
import sys
from pathlib import Path
from typing import Optional

from .chunker import chunk_markdown, extract_metadata_from_path
from .embeddings import embed_documents
from .logger import get_logger
from .models import TextChunk
from .vector_db import create_collection, upsert_chunks

logger = get_logger()

# Batch size for embedding API calls
EMBEDDING_BATCH_SIZE = 96


def ingest_file(
    file_path: str,
    chapter: str,
    chapter_title: str,
    week: int,
) -> int:
    """Ingest a single markdown file.

    Args:
        file_path: Path to markdown file
        chapter: Chapter slug
        chapter_title: Human-readable chapter title
        week: Course week number

    Returns:
        Number of chunks ingested
    """
    logger.info(f"Ingesting file: {file_path}")

    # Read file content
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    if not content.strip():
        logger.warning(f"Empty file: {file_path}")
        return 0

    # Chunk the content
    chunks = chunk_markdown(
        content=content,
        source_file=file_path,
        chapter=chapter,
        chapter_title=chapter_title,
        week=week,
    )

    if not chunks:
        logger.warning(f"No chunks created from: {file_path}")
        return 0

    # Embed chunks in batches
    all_embeddings = []
    for i in range(0, len(chunks), EMBEDDING_BATCH_SIZE):
        batch = chunks[i : i + EMBEDDING_BATCH_SIZE]
        texts = [chunk.content for chunk in batch]
        embeddings = embed_documents(texts)
        all_embeddings.extend(embeddings)
        logger.info(f"Embedded batch {i // EMBEDDING_BATCH_SIZE + 1}")

    # Upsert to Qdrant
    upsert_chunks(chunks, all_embeddings)

    logger.info(f"Ingested {len(chunks)} chunks from {file_path}")
    return len(chunks)


def ingest_batch(files: list[dict]) -> int:
    """Ingest multiple files.

    Args:
        files: List of file info dicts with path, chapter, title, week

    Returns:
        Total number of chunks ingested
    """
    total_chunks = 0
    for file_info in files:
        try:
            count = ingest_file(
                file_path=file_info["path"],
                chapter=file_info["chapter"],
                chapter_title=file_info["title"],
                week=file_info["week"],
            )
            total_chunks += count
        except Exception as e:
            logger.error(f"Failed to ingest {file_info['path']}: {e}")

    return total_chunks


def discover_docs(docs_path: str) -> list[dict]:
    """Discover markdown files in docs directory.

    Args:
        docs_path: Path to docs directory

    Returns:
        List of file info dicts
    """
    files = []
    docs_dir = Path(docs_path)

    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_path}")
        return files

    for md_file in docs_dir.rglob("*.md"):
        # Skip index and readme files
        if md_file.name.lower() in ["index.md", "readme.md"]:
            continue

        relative_path = str(md_file.relative_to(docs_dir.parent))
        metadata = extract_metadata_from_path(relative_path)

        # Generate title from filename
        title = md_file.stem.replace("-", " ").replace("_", " ").title()

        files.append({
            "path": str(md_file),
            "chapter": metadata["chapter"],
            "title": title,
            "week": metadata["week"],
        })

    logger.info(f"Discovered {len(files)} markdown files")
    return files


def main(
    docs_path: Optional[str] = None,
    recreate: bool = False,
) -> int:
    """Main ingestion entry point.

    Args:
        docs_path: Path to docs directory
        recreate: Whether to recreate the collection

    Returns:
        Total number of chunks ingested
    """
    # Parse command line args if not provided
    if docs_path is None:
        parser = argparse.ArgumentParser(
            description="Ingest textbook content into Qdrant"
        )
        parser.add_argument(
            "--docs-path",
            type=str,
            default="../docs",
            help="Path to docs directory",
        )
        parser.add_argument(
            "--recreate",
            action="store_true",
            help="Recreate the collection before ingesting",
        )
        args = parser.parse_args()
        docs_path = args.docs_path
        recreate = args.recreate

    logger.info(f"Starting ingestion from {docs_path}")

    # Recreate collection if requested
    if recreate:
        logger.info("Recreating collection...")
        create_collection()

    # Discover and ingest files
    files = discover_docs(docs_path)
    if not files:
        logger.error("No files found to ingest")
        return 0

    total = ingest_batch(files)
    logger.info(f"Ingestion complete: {total} total chunks")

    return total


if __name__ == "__main__":
    sys.exit(0 if main() > 0 else 1)
