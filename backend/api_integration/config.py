"""
Configuration module for FastAPI API settings.
"""
import os
from typing import List, Optional
from pydantic_settings import BaseSettings
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # API Configuration
    API_TITLE: str = "Humanoid Robotics Chat API"
    API_DESCRIPTION: str = "API for integrating the AI agent backend with the Docusaurus frontend"
    API_VERSION: str = "1.0.0"

    # Server Configuration
    HOST: str = "0.0.0.0"
    PORT: int = 8007
    DEBUG: bool = False

    # CORS Configuration
    # Allow Docusaurus frontend origins
    ALLOWED_ORIGINS: List[str] = [
        "http://localhost:3000",  # Default Docusaurus dev server
        "http://localhost:3001",  # Alternative Docusaurus dev server
        "http://localhost:3002",  # Additional Docusaurus dev server
        "http://localhost:5000",  # Our Docusaurus dev server
        "http://localhost:5001",  # Our Docusaurus dev server
        "http://localhost:5002",  # Our Docusaurus dev server
        "http://localhost:5003",  # Our Docusaurus dev server
        "http://127.0.0.1:3000",
        "http://127.0.0.1:3001",
        "http://127.0.0.1:3002",
        "http://127.0.0.1:5000",
        "http://127.0.0.1:5001",
        "http://127.0.0.1:5002",
        "http://127.0.0.1:5003",
        "http://localhost:4000",  # Additional possible ports
        "http://127.0.0.1:4000",
        "https://javedali735-physical-ai-and-humanoid-robotics.hf.space",  # Hugging Face Space
        "https://*.hf.space",  # Allow any Hugging Face Space subdomain
    ]

    # Rate Limiting Configuration
    RATE_LIMIT_MAX_REQUESTS: int = 100  # Max requests per time window
    RATE_LIMIT_WINDOW_SECONDS: int = 3600  # Time window in seconds (1 hour)

    # AI Agent Configuration
    # Check for GEMINI_API_KEY first, then fallback to OPENAI_API_KEY
    GEMINI_API_KEY: Optional[str] = os.getenv("GEMINI_API_KEY")
    OPENAI_API_KEY: Optional[str] = os.getenv("OPENAI_API_KEY")

    # Qdrant Configuration (from existing config)
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "book_text_embedding")

    # Application Configuration
    TOP_K: int = int(os.getenv("TOP_K", "5"))
    SIMILARITY_THRESHOLD: float = float(os.getenv("SIMILARITY_THRESHOLD", "0.5"))

    # Session Configuration
    SESSION_EXPIRATION_MINUTES: int = 30

    class Config:
        env_file = ".env"
        case_sensitive = True
        extra = "allow"  # Allow extra environment variables not defined in the model

# Create a global settings instance
settings = Settings()