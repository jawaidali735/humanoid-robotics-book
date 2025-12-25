# """
# Qdrant Client Initialization Module

# This module initializes and returns a Qdrant client.
# """
# from qdrant_client import QdrantClient
# import logging

# from backend.config.settings import QDRANT_URL, QDRANT_API_KEY

# logger = logging.getLogger(__name__)

# def initialize_qdrant_client() -> QdrantClient:
#     """
#     Initializes and returns a Qdrant client.

#     Returns:
#         QdrantClient: An instance of the Qdrant client configured with the settings
#     """
#     try:
#         # Determine if we're using local or cloud Qdrant based on the URL
#         if QDRANT_URL.startswith("http://") or QDRANT_URL.startswith("https://"):
#             # Using HTTP/HTTPS - could be local or cloud
#             if QDRANT_API_KEY:
#                 # Using cloud instance with API key
#                 client = QdrantClient(
#                     url=QDRANT_URL,
#                     api_key=QDRANT_API_KEY,
#                     timeout=10
#                 )
#                 logger.info(f"Initialized Qdrant client for cloud instance at {QDRANT_URL}")
#             else:
#                 # Using local instance without API key
#                 client = QdrantClient(
#                     url=QDRANT_URL,
#                     timeout=10
#                 )
#                 logger.info(f"Initialized Qdrant client for local instance at {QDRANT_URL}")
#         else:
#             # Assume local instance with host and port
#             client = QdrantClient(
#                 host=QDRANT_URL,
#                 timeout=10
#             )
#             logger.info(f"Initialized Qdrant client for local instance at {QDRANT_URL}")

#         # Test the connection
#         try:
#             client.get_collections()
#             logger.info("Successfully connected to Qdrant")
#         except Exception as e:
#             logger.error(f"Failed to connect to Qdrant: {str(e)}")
#             raise

#         return client

#     except Exception as e:
#         logger.error(f"Error initializing Qdrant client: {str(e)}")
#         raise


# def validate_qdrant_connection(client: QdrantClient) -> bool:
#     """
#     Validates the connection to Qdrant by attempting to get collections.

#     Args:
#         client (QdrantClient): The Qdrant client to validate

#     Returns:
#         bool: True if connection is valid, False otherwise
#     """
#     try:
#         client.get_collections()
#         return True
#     except Exception as e:
#         logger.error(f"Qdrant connection validation failed: {str(e)}")
#         return False