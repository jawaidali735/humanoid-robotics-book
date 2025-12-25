# Text Extraction & Embedding Pipeline

A single-file implementation to extract text from Docusaurus book URLs, chunk it appropriately, generate embeddings using Cohere, and store them in Qdrant vector database.

## Overview

This pipeline implements a complete RAG (Retrieval-Augmented Generation) data ingestion system that:

1. Crawls Docusaurus-based documentation sites to find all content pages
2. Extracts clean text content while preserving semantic structure
3. Chunks text into appropriately sized segments for embedding models
4. Generates semantic embeddings using Cohere's embedding models
5. Stores embeddings in Qdrant vector database with metadata
6. Tests retrieval functionality with sample queries

## Prerequisites

- Python 3.9+
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. Clone the repository and navigate to the backend directory
2. Install dependencies using UV:
   ```bash
   uv pip install -r requirements.txt
   ```
   Or if using pyproject.toml:
   ```bash
   uv sync
   ```

3. Create a `.env` file with your configuration:
   ```env
   # API Configuration
   COHERE_API_KEY=your_cohere_api_key_here

   # Qdrant Configuration
   QDRANT_HOST=your_qdrant_host_here
   QDRANT_API_KEY=your_qdrant_api_key_here

   # Processing Configuration
   CHUNK_SIZE=300
   CHUNK_OVERLAP=50
   ```

## Usage

### Running the Complete Pipeline

To run the entire pipeline from URL crawling to embedding storage and retrieval testing:

```python
from main import last_main

if __name__ == "__main__":
    last_main()
```

### Using Individual Functions

The pipeline is composed of several individual functions that can be used separately:

```python
from main import get_all_urls, extract_text_from_urls, chunk_text, embed, create_collection, save_chunk_to_qdrant

# Get all URLs from a Docusaurus site
urls = get_all_urls("https://your-docusaurus-site.com")

# Extract text from URLs
text_contents = extract_text_from_urls(urls)

# Chunk text appropriately
for content in text_contents:
    chunks = chunk_text(content['content'])

# Generate embeddings
texts = [chunk['content'] for chunk in chunks]
embeddings = embed(texts)

# Store in Qdrant
client = create_collection("my_collection")
for i, embedding in enumerate(embeddings):
    save_chunk_to_qdrant(client, "my_collection", chunks[i]['content'], embedding)
```

## Configuration

The pipeline can be configured via environment variables in the `.env` file:

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_HOST`: Qdrant Cloud host URL
- `QDRANT_API_KEY`: Qdrant Cloud API key
- `CHUNK_SIZE`: Target number of words per text chunk (default: 300)
- `CHUNK_OVERLAP`: Number of words to overlap between chunks (default: 50)

## Functions

### `get_all_urls(base_url)`
Crawls a Docusaurus site and returns all content page URLs.

### `extract_text_from_urls(urls)`
Extracts clean text content from provided URLs, removing navigation and boilerplate elements.

### `chunk_text(text, chunk_size=None, overlap=None)`
Chunks text into appropriately sized segments with overlap for context preservation.

### `embed(texts)`
Generates embeddings for text chunks using Cohere's embedding model with caching.

### `create_collection(collection_name)`
Creates a Qdrant collection for storing embeddings.

### `save_chunk_to_qdrant(client, collection_name, chunk_content, embedding, metadata=None)`
Saves individual text chunks with embeddings to Qdrant.

### `last_main()`
Orchestrates the complete pipeline from start to finish.

## Architecture

The implementation follows a single-file architecture with the following components:

1. **URL Crawling**: Discovers all content pages on a Docusaurus site
2. **Text Extraction**: Cleans HTML and extracts semantic content
3. **Text Chunking**: Splits content into appropriately sized segments
4. **Embedding Generation**: Creates semantic vectors using Cohere
5. **Storage**: Persists embeddings in Qdrant with metadata
6. **Retrieval Testing**: Validates the stored embeddings with search queries

## Error Handling

The pipeline includes comprehensive error handling:
- Network request failures are retried with exponential backoff
- Embedding API rate limits are handled gracefully
- Qdrant connection issues are caught and logged
- Individual document processing failures don't stop the entire pipeline

## Performance Considerations

- Embedding caching prevents redundant API calls
- Rate limiting prevents overwhelming source sites
- Memory usage is optimized by processing documents sequentially
- Configurable chunk size balances context preservation with embedding model limits