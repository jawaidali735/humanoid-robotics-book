"""
Configuration Module

This module centralizes all configuration settings for the application.
"""
import os
from dotenv import load_dotenv

load_dotenv()

EMBED_MODEL = "embed-english-v3.0"
COHERE_API_KEY = os.getenv("COHERE_API_KEY", "")
COHERE_MODEL = os.getenv("COHERE_MODEL", "embed-english-v3.0")
COHERE_INPUT_TYPE = os.getenv("COHERE_INPUT_TYPE", "search_document")
SITEMAP_URL= os.getenv("SITEMAP_URL")

# API Configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")
GROQ_API_KEY = os.getenv("GROQ_API_KEY", "")  # Note: Corrected from GROK to GROQ as used in the agent.py
GROK_API_KEY = os.getenv("GROK_API_KEY", "")  # Added for GROK API key

# Qdrant Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_book")

# Application Configuration
CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "1000"))
CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", "200"))
TOP_K = int(os.getenv("TOP_K", "20"))
SIMILARITY_THRESHOLD = float(os.getenv("SIMILARITY_THRESHOLD", "0.3"))

# Server Configuration
HOST = os.getenv("HOST", "0.0.0.0")
PORT = int(os.getenv("PORT", "8007"))
DEBUG = os.getenv("DEBUG", "False").lower() == "true"

# CORS Configuration
ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000,http://localhost:5000").split(",")

# Rate Limiting Configuration
RATE_LIMIT_MAX_REQUESTS = int(os.getenv("RATE_LIMIT_MAX_REQUESTS", "100"))
RATE_LIMIT_WINDOW_SECONDS = int(os.getenv("RATE_LIMIT_WINDOW_SECONDS", "3600"))

def validate_config(require_api_keys=True):
    """
    Validates that all required configuration parameters are present.

    Args:
        require_api_keys (bool): Whether to require API keys (set to False for testing)
    """
    errors = []

    # Check if at least one API key is available (for testing, this can be skipped)
    if require_api_keys:
        api_keys = [OPENAI_API_KEY, GEMINI_API_KEY, COHERE_API_KEY, GROQ_API_KEY, GROK_API_KEY]
        if not any(api_keys):
            errors.append("At least one API key is required: OPENAI_API_KEY, GEMINI_API_KEY, COHERE_API_KEY, GROQ_API_KEY, or GROK_API_KEY")

    # Validate chunk parameters
    if CHUNK_SIZE <= 0:
        errors.append("CHUNK_SIZE must be a positive integer")

    if CHUNK_OVERLAP < 0:
        errors.append("CHUNK_OVERLAP must be a non-negative integer")

    if CHUNK_OVERLAP >= CHUNK_SIZE:
        errors.append("CHUNK_OVERLAP must be less than CHUNK_SIZE")

    # Validate Qdrant configuration
    if not QDRANT_URL:
        errors.append("QDRANT_URL environment variable is required")

    if errors:
        raise ValueError("Configuration validation failed: " + "; ".join(errors))

# Validate configuration at startup (set require_api_keys=False for testing environment)
import os
validate_config(require_api_keys=not os.environ.get('TESTING', False))