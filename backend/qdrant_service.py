from typing import List, Dict
from qdrant_client import QdrantClient
from qdrant_client.http import models
from config import Config
import logging
import os
import cohere

logger = logging.getLogger(__name__)


class QdrantService:
    """Service class to handle Qdrant vector database operations."""

    def __init__(self):
        self.client = QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            prefer_grpc=False
        )
        self.collection_name = Config.QDRANT_COLLECTION_NAME

        # Cohere client (init once)
        api_key = os.getenv("COHERE_API_KEY") or getattr(Config, "COHERE_API_KEY", None)
        if not api_key:
            raise ValueError("COHERE_API_KEY is required")
        self.cohere = cohere.Client(api_key)

    # -------------------------
    # Embedding
    # -------------------------
    def get_embedding(self, text: str) -> List[float]:
        try:
            response = self.cohere.embed(
                texts=[text],
                model="multilingual-22-12",
                input_type="search_query"
            )

            embedding = response.embeddings[0]
            if len(embedding) != 768:
                raise ValueError("Invalid embedding size")

            return embedding

        except Exception as e:
            logger.error(f"Embedding failed: {str(e)}")
            raise

    # -------------------------
    # Retrieval
    # -------------------------
    def retrieve_information(self, query_text: str, top_k: int = None) -> List[Dict]:
        if top_k is None:
            top_k = Config.TOP_K

        if not self.collection_exists():
            logger.warning(f"Collection '{self.collection_name}' does not exist")
            return []

        query_embedding = self.get_embedding(query_text)

        search_result = self.client.query_points(
            collection_name=self.collection_name,
            query=models.NamedVector(
                name="default",
                vector=query_embedding
            ),
            limit=top_k,
            score_threshold=Config.SIMILARITY_THRESHOLD
        )

        results = []
        for point in search_result.points:
            payload = point.payload or {}

            results.append({
                "content_id": str(point.id),
                "content_text": payload.get("chunk_content", ""),
                "source_reference": (
                    f"{payload.get('source_title', '')} - {payload.get('source_url', '')}"
                    if payload.get("source_title") or payload.get("source_url")
                    else payload.get("source_reference", "")
                ),
                "similarity_score": float(point.score),
                "metadata": payload.get("metadata", {})
            })

        logger.info(f"Retrieved {len(results)} results")
        return results

    # -------------------------
    # Utilities
    # -------------------------
    def collection_exists(self) -> bool:
        try:
            collections = self.client.get_collections().collections
            return any(c.name == self.collection_name for c in collections)
        except Exception as e:
            logger.error(str(e))
            return False

    def get_collection_info(self):
        try:
            return self.client.get_collection(self.collection_name)
        except Exception as e:
            logger.error(str(e))
            return None
