"""Tests for Cohere embeddings module."""

import pytest
from unittest.mock import MagicMock, patch


class TestCohereEmbeddings:
    """Test cases for Cohere embeddings."""

    @patch("src.embeddings.cohere.Client")
    def test_embed_text_returns_vector(self, mock_cohere_client):
        """Test that embed_text returns a 1024-dimensional vector."""
        # Mock Cohere response
        mock_response = MagicMock()
        mock_response.embeddings = [[0.1] * 1024]
        mock_cohere_client.return_value.embed.return_value = mock_response

        from src.embeddings import embed_text

        result = embed_text("What is Physical AI?")

        assert result is not None
        assert len(result) == 1024
        mock_cohere_client.return_value.embed.assert_called_once()

    @patch("src.embeddings.cohere.Client")
    def test_embed_documents_batch(self, mock_cohere_client):
        """Test that embed_documents handles batch embedding."""
        # Mock Cohere response for batch
        mock_response = MagicMock()
        mock_response.embeddings = [[0.1] * 1024, [0.2] * 1024, [0.3] * 1024]
        mock_cohere_client.return_value.embed.return_value = mock_response

        from src.embeddings import embed_documents

        texts = [
            "Physical AI introduction",
            "ROS 2 fundamentals",
            "Gazebo simulation",
        ]
        results = embed_documents(texts)

        assert results is not None
        assert len(results) == 3
        assert all(len(vec) == 1024 for vec in results)

    @patch("src.embeddings.cohere.Client")
    def test_embed_text_uses_search_query_type(self, mock_cohere_client):
        """Test that embed_text uses input_type='search_query'."""
        mock_response = MagicMock()
        mock_response.embeddings = [[0.1] * 1024]
        mock_cohere_client.return_value.embed.return_value = mock_response

        from src.embeddings import embed_text

        embed_text("test query")

        call_kwargs = mock_cohere_client.return_value.embed.call_args.kwargs
        assert call_kwargs.get("input_type") == "search_query"

    @patch("src.embeddings.cohere.Client")
    def test_embed_documents_uses_search_document_type(self, mock_cohere_client):
        """Test that embed_documents uses input_type='search_document'."""
        mock_response = MagicMock()
        mock_response.embeddings = [[0.1] * 1024]
        mock_cohere_client.return_value.embed.return_value = mock_response

        from src.embeddings import embed_documents

        embed_documents(["test document"])

        call_kwargs = mock_cohere_client.return_value.embed.call_args.kwargs
        assert call_kwargs.get("input_type") == "search_document"
