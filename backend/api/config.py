"""API Configuration settings using Pydantic."""

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # API Settings
    cors_origins: list[str] = [
        "https://physical-ai-textbook-two.vercel.app",
        "http://localhost:3000",
        "http://localhost:8000",
    ]

    # Logging
    log_level: str = "INFO"

    class Config:
        """Pydantic config."""

        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore unknown env vars
