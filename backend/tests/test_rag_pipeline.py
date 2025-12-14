"""Tests for RAG pipeline."""

import pytest
from unittest.mock import MagicMock, patch


class TestRAGPipeline:
    """Test cases for RAG pipeline."""

    @patch("src.rag_pipeline.embed_text")
    @patch("src.rag_pipeline.search_similar")
    @patch("src.rag_pipeline.rerank_results")
    @patch("src.rag_pipeline.generate_response")
    def test_ask_question_full_pipeline(
        self, mock_generate, mock_rerank, mock_search, mock_embed
    ):
        """Test that ask_question executes full RAG pipeline."""
        from src.models import SearchResult, RerankedResult, RAGResponse, Citation

        # Mock embeddings
        mock_embed.return_value = [0.1] * 1024

        # Mock search results
        mock_search.return_value = [
            SearchResult(
                chunk_id="chunk_1",
                content="Physical AI refers to...",
                chapter="intro",
                chapter_title="Introduction to Physical AI",
                week=1,
                source_file="docs/intro.md",
                score=0.9,
            )
        ]

        # Mock rerank results
        mock_rerank.return_value = [
            RerankedResult(
                chunk_id="chunk_1",
                content="Physical AI refers to...",
                chapter="intro",
                chapter_title="Introduction to Physical AI",
                source_file="docs/intro.md",
                relevance_score=0.95,
            )
        ]

        # Mock LLM response
        mock_generate.return_value = "Physical AI is about embodied AI systems."

        from src.rag_pipeline import ask_question

        result = ask_question("What is Physical AI?")

        assert result is not None
        assert hasattr(result, "answer")
        assert hasattr(result, "citations")
        assert hasattr(result, "confidence")

        # Verify pipeline order
        mock_embed.assert_called_once()
        mock_search.assert_called_once()
        mock_rerank.assert_called_once()
        mock_generate.assert_called_once()


class TestSelectedTextPipeline:
    """Test cases for selected text pipeline."""

    @patch("src.rag_pipeline.rerank_text_chunks")
    @patch("src.rag_pipeline.generate_response")
    def test_ask_selected_bypasses_qdrant(self, mock_generate, mock_rerank):
        """Test that ask_selected does NOT use Qdrant search."""
        # Mock rerank results (index, score)
        mock_rerank.return_value = [(0, 0.9)]

        # Mock LLM response
        mock_generate.return_value = "This explains the concept in simpler terms."

        from src.rag_pipeline import ask_selected

        with patch("src.rag_pipeline.search_similar") as mock_search:
            result = ask_selected(
                question="Explain this",
                selected_text="Some text about ROS 2 communication patterns.",
            )

            # Qdrant search should NOT be called
            mock_search.assert_not_called()

        assert result is not None
        mock_rerank.assert_called_once()
        mock_generate.assert_called_once()
