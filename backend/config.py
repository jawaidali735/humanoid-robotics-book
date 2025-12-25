import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration class to manage environment variables and settings."""

    # API Configuration
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
    # Check for GEMINI_API_KEY as well
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    if not OPENAI_API_KEY and not GEMINI_API_KEY and not COHERE_API_KEY:
        print("Warning: No API key set (OPENAI_API_KEY, GEMINI_API_KEY, or COHERE_API_KEY)")
        # For testing purposes, we'll allow the system to continue with a None value
        # but in production, one of these should be set

    # Qdrant Configuration
    QDRANT_URL = os.getenv("QDRANT_URL")
    if not QDRANT_URL:
        raise ValueError("QDRANT_URL environment variable is required")

    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    if not QDRANT_API_KEY:
        raise ValueError("QDRANT_API_KEY environment variable is required")

    QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_text_embedding")

    # Application Configuration
    DEBUG = os.getenv("DEBUG", "False").lower() == "true"
    LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")

    # Similarity search configuration
    TOP_K = int(os.getenv("TOP_K", "20"))  # Number of results to retrieve
    SIMILARITY_THRESHOLD = float(os.getenv("SIMILARITY_THRESHOLD", "0.3"))