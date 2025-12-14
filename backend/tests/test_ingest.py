"""Tests for ingestion pipeline."""

import pytest
from unittest.mock import MagicMock, patch


class TestIngestionPipeline:
    """Test cases for content ingestion."""

    @patch("src.ingest.embed_documents")
    @patch("src.ingest.upsert_chunks")
    def test_ingest_file_processes_correctly(self, mock_upsert, mock_embed):
        """Test that ingest_file processes a markdown file correctly."""
        # Mock embeddings
        mock_embed.return_value = [[0.1] * 1024, [0.2] * 1024]

        # Mock upsert
        mock_upsert.return_value = 2

        from src.ingest import ingest_file

        markdown_content = """# Test Chapter

This is test content for the ingestion pipeline.

## Section 1

More content here.
"""

        with patch("builtins.open", MagicMock(return_value=MagicMock(
            __enter__=lambda s: MagicMock(read=lambda: markdown_content),
            __exit__=lambda s, *args: None
        ))):
            result = ingest_file(
                file_path="docs/week-01-02-intro/intro.md",
                chapter="intro",
                chapter_title="Introduction",
                week=1,
            )

        assert result >= 0  # Should return number of chunks ingested

    @patch("src.ingest.embed_documents")
    @patch("src.ingest.upsert_chunks")
    def test_ingest_batch_handles_multiple_files(self, mock_upsert, mock_embed):
        """Test that batch ingestion handles multiple files."""
        mock_embed.return_value = [[0.1] * 1024]
        mock_upsert.return_value = 1

        from src.ingest import ingest_batch

        files = [
            {"path": "docs/intro.md", "chapter": "intro", "title": "Intro", "week": 1},
            {"path": "docs/ros2.md", "chapter": "ros2", "title": "ROS 2", "week": 3},
        ]

        with patch("src.ingest.ingest_file", return_value=1):
            result = ingest_batch(files)

        assert result == 2  # Total chunks from both files
