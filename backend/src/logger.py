"""JSON logging configuration for RAG backend.

Constitution Section VIII: NO SQL for logging.
All logs go to stdout/stderr as JSON for serverless compatibility.
"""

import json
import logging
import sys
from datetime import datetime
from typing import Any

from .config import get_settings


class JSONFormatter(logging.Formatter):
    """Format log records as JSON for structured logging."""

    def format(self, record: logging.LogRecord) -> str:
        """Format a log record as JSON."""
        log_data: dict[str, Any] = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }

        # Add extra fields if present
        if hasattr(record, "extra"):
            log_data["extra"] = record.extra

        # Add exception info if present
        if record.exc_info:
            log_data["exception"] = self.formatException(record.exc_info)

        return json.dumps(log_data)


def setup_logging() -> logging.Logger:
    """Configure JSON logging to stdout."""
    settings = get_settings()

    # Create logger
    logger = logging.getLogger("rag")
    logger.setLevel(getattr(logging, settings.log_level.upper()))

    # Remove existing handlers
    logger.handlers.clear()

    # Create stdout handler with JSON formatter
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(JSONFormatter())
    logger.addHandler(handler)

    # Prevent propagation to root logger
    logger.propagate = False

    return logger


def get_logger() -> logging.Logger:
    """Get the configured logger instance."""
    logger = logging.getLogger("rag")
    if not logger.handlers:
        return setup_logging()
    return logger


def log_with_extra(
    level: str,
    message: str,
    **extra: Any,
) -> None:
    """Log a message with extra structured data."""
    logger = get_logger()
    record = logger.makeRecord(
        name=logger.name,
        level=getattr(logging, level.upper()),
        fn="",
        lno=0,
        msg=message,
        args=(),
        exc_info=None,
    )
    record.extra = extra  # type: ignore
    logger.handle(record)
