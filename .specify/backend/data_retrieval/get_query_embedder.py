from config.settings import COHERE_API_KEY
import cohere
cohere_client = cohere.Client(COHERE_API_KEY)


def get_embedding(text):
    """Get embedding vector from Cohere Embed - using same dimensions as existing vectors (768 dim)"""
    response = cohere_client.embed(
        model="embed-multilingual-v2.0",  # Use multilingual v2.0 model which produces 768-dim vectors to match existing collection
        texts=[text],
    )
    return response.embeddings[0] 