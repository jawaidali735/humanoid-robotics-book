import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct

from config.settings import QDRANT_COLLECTION_NAME, QDRANT_API_KEY, QDRANT_URL
from .embedder import embed

qdrant = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
)

def save_chunk_to_qdrant(chunk, chunk_id, url):
    vector = embed(chunk)

    qdrant.upsert(
        collection_name=QDRANT_COLLECTION_NAME,
        points=[
            PointStruct(
                id=int(chunk_id),  # Use integer ID as required by Qdrant
                vector=vector,
                payload={
                    "url": url,
                    "text": chunk,
                    "chunk_id": str(chunk_id)
                }
            )
        ]
    )