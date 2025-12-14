"""Tests for Cohere rerank module."""

import pytest
from unittest.mock import MagicMock, patch


class TestCohereRerank:
    """Test cases for Cohere rerank."""

    @patch("src.rerank.cohere.Client")
    def test_rerank_results_returns_sorted(self, mock_cohere_client):
        """Test that rerank_results returns sorted results by relevance."""
        # Mock Cohere rerank response
        mock_result_1 = MagicMock()
        mock_result_1.index = 1
        mock_result_1.relevance_score = 0.95

        mock_result_2 = MagicMock()
        mock_result_2.index = 0
        mock_result_2.relevance_score = 0.75

        mock_response = MagicMock()
        mock_response.results = [mock_result_1, mock_result_2]
        mock_cohere_client.return_value.rerank.return_value = mock_response

        from src.rerank import rerank_results
        from src.models import SearchResult

        search_results = [
            SearchResult(
                chunk_id="chunk_1",
                content="First result about ROS",
                chapter="ros2",
                chapter_title="ROS 2 Overview",
                week=3,
                source_file="docs/week-03-05-ros2/ros2-overview.md",
                score=0.8,
            ),
            SearchResult(
                chunk_id="chunk_2",
                content="Second result about Physical AI",
                chapter="intro",
                chapter_title="Introduction to Physical AI",
                week=1,
                source_file="docs/week-01-02-intro/intro.md",
                score=0.7,
            ),
        ]

        results = rerank_results("What is Physical AI?", search_results)

        assert results is not None
        assert len(results) == 2
        # Should be sorted by relevance score descending
        assert results[0].relevance_score >= results[1].relevance_score

    @patch("src.rerank.cohere.Client")
    def test_rerank_with_top_n(self, mock_cohere_client):
        """Test that rerank_results respects top_n parameter."""
        mock_results = []
        for i in range(5):
            mock_result = MagicMock()
            mock_result.index = i
            mock_result.relevance_score = 0.9 - (i * 0.1)
            mock_results.append(mock_result)

        mock_response = MagicMock()
        mock_response.results = mock_results[:3]  # top_n=3
        mock_cohere_client.return_value.rerank.return_value = mock_response

        from src.rerank import rerank_results
        from src.models import SearchResult

        search_results = [
            SearchResult(
                chunk_id=f"chunk_{i}",
                content=f"Result {i}",
                chapter="intro",
                chapter_title="Introduction",
                week=1,
                source_file="docs/intro.md",
                score=0.5,
            )
            for i in range(5)
        ]

        results = rerank_results("test query", search_results, top_n=3)

        assert len(results) == 3
        # Verify top_n was passed to Cohere
        call_kwargs = mock_cohere_client.return_value.rerank.call_args.kwargs
        assert call_kwargs.get("top_n") == 3
