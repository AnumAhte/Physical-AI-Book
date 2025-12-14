"""Tests for API endpoints."""

import pytest
from unittest.mock import MagicMock, patch, AsyncMock
from fastapi.testclient import TestClient


class TestAskEndpoint:
    """Test cases for POST /api/ask endpoint."""

    @patch("src.api.ask_question")
    def test_ask_returns_valid_response(self, mock_ask):
        """Test that /api/ask returns a valid RAG response."""
        from src.models import RAGResponse, Citation

        mock_ask.return_value = RAGResponse(
            answer="Physical AI refers to AI systems that interact with the physical world.",
            citations=[
                Citation(
                    chapter="intro",
                    chapter_title="Introduction to Physical AI",
                    source_file="docs/week-01-02-intro/intro.md",
                    excerpt="Physical AI refers to artificial intelligence...",
                )
            ],
            confidence=0.92,
        )

        from src.main import app

        client = TestClient(app)
        response = client.post(
            "/api/ask",
            json={"question": "What is Physical AI?", "top_k": 5},
        )

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data
        assert "citations" in data
        assert "confidence" in data

    def test_ask_validates_question_length(self):
        """Test that /api/ask validates question length."""
        from src.main import app

        client = TestClient(app)

        # Too short
        response = client.post(
            "/api/ask",
            json={"question": "hi"},
        )
        assert response.status_code == 422

        # Too long
        response = client.post(
            "/api/ask",
            json={"question": "x" * 1001},
        )
        assert response.status_code == 422


class TestAskSelectedEndpoint:
    """Test cases for POST /api/ask-selected endpoint."""

    @patch("src.api.ask_selected")
    def test_ask_selected_returns_valid_response(self, mock_ask_selected):
        """Test that /api/ask-selected returns a valid response."""
        from src.models import RAGResponse, Citation

        mock_ask_selected.return_value = RAGResponse(
            answer="This explains the publish-subscribe pattern in ROS 2.",
            citations=[
                Citation(
                    chapter="ros2",
                    chapter_title="ROS 2 Overview",
                    source_file="selected_text",
                    excerpt="ROS 2 nodes communicate using...",
                )
            ],
            confidence=0.88,
        )

        from src.main import app

        client = TestClient(app)
        response = client.post(
            "/api/ask-selected",
            json={
                "question": "Explain this in simpler terms",
                "selected_text": "ROS 2 nodes communicate using a publish-subscribe pattern where nodes can publish messages to topics.",
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert "answer" in data


class TestHealthEndpoint:
    """Test cases for GET /api/health endpoint."""

    @patch("src.api.check_qdrant_health", new_callable=AsyncMock)
    @patch("src.api.check_cohere_health", new_callable=AsyncMock)
    def test_health_returns_status(self, mock_cohere, mock_qdrant):
        """Test that /api/health returns service status."""
        mock_qdrant.return_value = True
        mock_cohere.return_value = True

        from src.main import app

        client = TestClient(app)
        response = client.get("/api/health")

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "services" in data
        assert "version" in data
