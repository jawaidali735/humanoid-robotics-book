"""
Configuration package for the backend.

This module provides compatibility for both the old Config class-based approach
and the new settings module approach.
"""

# Import the Config class from the root config.py file to maintain compatibility
try:
    from ..config import Config
except ImportError:
    # If the config module doesn't exist, create a dummy Config class
    class Config:
        QDRANT_URL = "http://localhost:6333"
        QDRANT_API_KEY = "dummy_key"
        QDRANT_COLLECTION_NAME = "humanoid_robotics_book"
        TOP_K = 20
        SIMILARITY_THRESHOLD = 0.3
        OPENAI_API_KEY = None
        GEMINI_API_KEY = None
        COHERE_API_KEY = None

# Import settings from the settings.py file to maintain compatibility
try:
    from .settings import *
except ImportError:
    # If settings module doesn't exist, define basic settings
    QDRANT_URL = "http://localhost:6333"
    QDRANT_API_KEY = "dummy_key"
    QDRANT_COLLECTION_NAME = "humanoid_robotics_book"

# Make sure both are available at the package level
__all__ = ['Config', 'QDRANT_URL', 'QDRANT_API_KEY', 'QDRANT_COLLECTION_NAME',
           'EMBED_MODEL', 'COHERE_API_KEY', 'COHERE_MODEL', 'COHERE_INPUT_TYPE',
           'SITEMAP_URL', 'CHUNK_SIZE', 'CHUNK_OVERLAP']