from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.exceptions import UnexpectedResponse
from config.settings import QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME
import os
import logging
from data_retrieval.get_query_embedder import get_embedding


# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

load_dotenv()

def retrieve(query):
    """
    Retrieve relevant content from the knowledge base using Qdrant.
    Includes error handling for Qdrant unavailability.
    """
    logger.info(f"Retrieving content for query: {query[:50]}...")

    try:
        qdrant = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY
        )
        qdrant_collection = QDRANT_COLLECTION_NAME

        embedding = get_embedding(query)
        logger.debug(f"Generated embedding for query: {query[:30]}...")

        result = qdrant.query_points(
            collection_name=qdrant_collection,
            query=embedding,
            limit=5
        )

        results = [
            point.payload.get("chunk_content")
            for point in result.points
            if point.payload and "chunk_content" in point.payload
        ]

        logger.info(f"Retrieved {len(results)} results for query: {query[:30]}...")
        return results

    except UnexpectedResponse as e:
        logger.error(f"Qdrant UnexpectedResponse error for query '{query[:30]}...': {e}")
        # Return empty list if Qdrant is unavailable
        return []
    except ConnectionError as e:
        logger.error(f"Qdrant connection error for query '{query[:30]}...': {e}")
        # Return empty list if Qdrant is unavailable
        return []
    except Exception as e:
        logger.error(f"Unexpected error during Qdrant retrieval for query '{query[:30]}...': {e}")
        # Return empty list for any other error
        return []


# Test
if __name__ == "__main__":
    print(retrieve("ROS"))