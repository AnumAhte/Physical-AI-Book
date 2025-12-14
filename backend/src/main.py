"""FastAPI application entry point for RAG backend.

Constitution Section VIII: Cohere + Qdrant ONLY (NO SQL)
"""

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from .api import router
from .config import get_settings
from .logger import get_logger, setup_logging
from .middleware import setup_cors

# Initialize logging
setup_logging()
logger = get_logger()

# Get settings
settings = get_settings()

# Create FastAPI app
app = FastAPI(
    title=settings.api_title,
    version=settings.api_version,
    description="""
RAG (Retrieval-Augmented Generation) API for the Physical AI & Humanoid Robotics textbook.

**Stack**: Cohere (embeddings + rerank) + Qdrant (vector storage)

**Constitution Compliance**:
- NO SQL/PostgreSQL/Neon
- Answers ONLY from textbook content
- Zero hallucination enforced
    """,
    docs_url="/docs",
    redoc_url="/redoc",
)

# Setup CORS middleware
setup_cors(app)

# Register API routes
app.include_router(router)


@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler for uncaught errors."""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "INTERNAL_ERROR",
            "message": "An unexpected error occurred. Please try again.",
        },
    )


@app.get("/", include_in_schema=False)
async def root():
    """Root endpoint redirect to docs."""
    return {
        "message": "Physical AI Textbook RAG API",
        "docs": "/docs",
        "health": "/api/health",
    }


def main():
    """Run the application with uvicorn."""
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info",
    )


if __name__ == "__main__":
    main()
