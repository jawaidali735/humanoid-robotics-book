
import logging
import os
import sys

# Add the parent directory to the path to import from config
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
from config.settings import QDRANT_COLLECTION_NAME, QDRANT_URL, QDRANT_API_KEY

logger = logging.getLogger(__name__)

def create_collection(client: QdrantClient = None):
    """
    Creates a Qdrant collection with the specified configuration.

    Args:
        client (QdrantClient): Qdrant client instance. If None, creates a new one.

    Returns:
        bool: True if collection was created, False if it already existed
    """
    if client is None:
        client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY
        )

    try:
        # Check if collection already exists
        collections = client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if QDRANT_COLLECTION_NAME in collection_names:
            logger.info(f"Collection '{QDRANT_COLLECTION_NAME}' already exists, skipping creation")
            return False

        # Create the collection
        client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=VectorParams(
                size=1024,        # Cohere embed-english-v3.0 dimension
                distance=Distance.COSINE
            )
        )

        logger.info(f"Collection '{QDRANT_COLLECTION_NAME}' created successfully")
        return True

    except Exception as e:
        logger.error(f"Error creating Qdrant collection: {str(e)}")
        raise
