
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from .url_extractor import get_all_urls, extract_text_from_url
from .qdrant_collection import create_collection
from config.settings import SITEMAP_URL
from .chunk_text import chunk_text
from .qdrant_store import save_chunk_to_qdrant
from qdrant_client import QdrantClient
from config.settings import QDRANT_URL, QDRANT_API_KEY



def start_ingestion(urls=None):
    """
    Main ingestion function that takes URLs and runs the complete workflow to save embeddings in Qdrant.

    Args:
        urls (list): List of URLs to process. If None, extracts from SITEMAP_URL
    """
    if urls is None:
        urls = get_all_urls(SITEMAP_URL)

    # Initialize Qdrant client to create collection
    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY
    )
    create_collection(client)

    global_id = 1

    for url in urls:
        print("\nProcessing:", url)
        text = extract_text_from_url(url)

        if not text:
            print(f"[WARNING] No text extracted from: {url}")
            continue

        chunks = chunk_text(text)

        for ch in chunks:
            if ch.strip():  # Only save non-empty chunks
                save_chunk_to_qdrant(ch, global_id, url)
                print(f"Saved chunk {global_id}")
                global_id += 1

    print("\n+ Ingestion completed!")
    print("Total chunks stored:", global_id - 1)


def ingest_from_sitemap():
    """Ingest content from the configured sitemap URL."""
    print("Getting URLs from sitemap...")
    urls = get_all_urls()
    start_ingestion(urls)


if __name__ == "__main__":
    # Run with default sitemap URLs
    start_ingestion()