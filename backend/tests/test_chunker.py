"""Tests for text chunker module."""

import pytest


class TestTextChunker:
    """Test cases for text chunking."""

    def test_chunk_markdown_returns_chunks(self):
        """Test that chunk_markdown returns list of chunks."""
        from src.chunker import chunk_markdown

        markdown_content = """# Introduction to Physical AI

Physical AI refers to artificial intelligence systems that interact with the physical world through embodied agents.

## What is Physical AI?

Physical AI systems can perceive their environment through sensors, reason about spatial relationships, and act through actuators.

These systems combine traditional AI with robotics, computer vision, and control systems.

## Applications

Physical AI has many applications including:
- Autonomous vehicles
- Warehouse robots
- Service robots
- Healthcare assistants
"""

        chunks = chunk_markdown(
            content=markdown_content,
            source_file="docs/week-01-02-intro/intro.md",
            chapter="intro",
            chapter_title="Introduction to Physical AI",
            week=1,
        )

        assert chunks is not None
        assert len(chunks) > 0
        assert all(hasattr(chunk, "content") for chunk in chunks)
        assert all(hasattr(chunk, "chapter") for chunk in chunks)

    def test_chunk_markdown_respects_chunk_size(self):
        """Test that chunks don't exceed maximum size."""
        from src.chunker import chunk_markdown

        # Create a long markdown content
        long_content = "# Test Chapter\n\n" + ("This is a test sentence. " * 500)

        chunks = chunk_markdown(
            content=long_content,
            source_file="test.md",
            chapter="test",
            chapter_title="Test",
            week=1,
            chunk_size=500,
        )

        # All chunks should be under the token limit (approximately 4 chars per token)
        for chunk in chunks:
            # Allow some buffer for overlap
            assert len(chunk.content) < 3000  # ~500 tokens * 6 chars

    def test_extract_metadata_from_path(self):
        """Test metadata extraction from file path."""
        from src.chunker import extract_metadata_from_path

        path = "docs/week-03-05-ros2/ros2-overview.md"
        metadata = extract_metadata_from_path(path)

        assert metadata["week"] == 3
        assert "ros2" in metadata["chapter"]
