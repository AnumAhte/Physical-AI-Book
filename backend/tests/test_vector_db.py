"""Tests for Qdrant vector database module."""

import pytest
from unittest.mock import MagicMock, patch


class TestQdrantVectorDB:
    """Test cases for Qdrant vector database."""

    @patch("src.vector_db.QdrantClient")
    def test_get_qdrant_client_creates_connection(self, mock_qdrant_class):
        """Test that get_qdrant_client creates a connection."""
        mock_client = MagicMock()
        mock_qdrant_class.return_value = mock_client

        from src.vector_db import get_qdrant_client

        # Clear cache for testing
        get_qdrant_client.cache_clear()

        with patch("src.vector_db.get_qdrant_url", return_value="https://test.qdrant.io"):
            with patch("src.vector_db.get_qdrant_api_key", return_value="test-key"):
                client = get_qdrant_client()

        assert client is not None
        mock_qdrant_class.assert_called_once()

    @patch("src.vector_db.QdrantClient")
    def test_search_similar_returns_results(self, mock_qdrant_class):
        """Test that search_similar returns search results."""
        # Mock Qdrant search response
        mock_point = MagicMock()
        mock_point.id = "chunk_001"
        mock_point.score = 0.89
        mock_point.payload = {
            "chunk_id": "chunk_001",
            "content": "Physical AI refers to...",
            "chapter": "intro",
            "chapter_title": "Introduction to Physical AI",
            "week": 1,
            "source_file": "docs/week-01-02-intro/intro.md",
            "heading": "What is Physical AI?",
        }

        mock_client = MagicMock()
        mock_client.search.return_value = [mock_point]
        mock_qdrant_class.return_value = mock_client

        from src.vector_db import search_similar, get_qdrant_client

        get_qdrant_client.cache_clear()

        with patch("src.vector_db.get_qdrant_url", return_value="https://test.qdrant.io"):
            with patch("src.vector_db.get_qdrant_api_key", return_value="test-key"):
                query_embedding = [0.1] * 1024
                results = search_similar(query_embedding, top_k=10)

        assert results is not None
        assert len(results) == 1
        assert results[0].chunk_id == "chunk_001"
        assert results[0].score == 0.89

    @patch("src.vector_db.QdrantClient")
    def test_create_collection_with_correct_params(self, mock_qdrant_class):
        """Test that create_collection uses correct vector parameters."""
        mock_client = MagicMock()
        mock_qdrant_class.return_value = mock_client

        from src.vector_db import create_collection, get_qdrant_client

        get_qdrant_client.cache_clear()

        with patch("src.vector_db.get_qdrant_url", return_value="https://test.qdrant.io"):
            with patch("src.vector_db.get_qdrant_api_key", return_value="test-key"):
                create_collection()

        # Verify recreate_collection was called
        mock_client.recreate_collection.assert_called_once()
        call_kwargs = mock_client.recreate_collection.call_args.kwargs
        assert call_kwargs["collection_name"] == "textbook"
