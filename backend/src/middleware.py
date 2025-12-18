from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .config import get_settings

def setup_cors(app: FastAPI) -> None:
    settings = get_settings()

    # cors_origins is now a List[str] from config
    origins = [origin.strip() for origin in settings.cors_origins if origin.strip()]

    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["*"],
    )
