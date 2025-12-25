# Quickstart Guide: Text Extraction & Embedding Pipeline

**Feature**: 001-text-extraction-embedding
**Created**: 2025-12-16

## Overview

This guide will help you set up and run the text extraction and embedding pipeline for processing Docusaurus book URLs. The implementation will be in a single `main.py` file with specific functions as required: `get_all_urls()`, `extract_text_from_urls()`, `chunk_text()`, `embed()`, `create_collection('book_text_embedding')`, `save_chunk_to_qdrant()`, and orchestrated by a `last_main()` function. The pipeline extracts clean text, chunks it appropriately, generates embeddings, and stores them in Qdrant.

## Prerequisites

- Python 3.9 or higher
- UV package manager
- Access to Cohere API (for embeddings)
- Access to Qdrant Cloud (for vector storage)

## Setup

### 1. Install UV Package Manager

If you don't have UV installed:

```bash
# On macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# On Windows (PowerShell)
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### 2. Create Project Directory

```bash
mkdir text-extraction-pipeline
cd text-extraction-pipeline
```

### 3. Initialize Python Project with UV

```bash
uv init
uv venv  # Create virtual environment
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 4. Install Dependencies

Create a `pyproject.toml` file with the required dependencies:

```bash
uv add requests beautifulsoup4 cohere qdrant-client python-dotenv pytest
```

This creates the following `pyproject.toml`:

```toml
[project]
name = "text-extraction-pipeline"
version = "0.1.0"
description = "Text extraction and embedding pipeline for Docusaurus books"
readme = "README.md"
requires-python = ">=3.9"
dependencies = [
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "cohere>=4.0.0",
    "qdrant-client>=1.7.0",
    "python-dotenv>=1.0.0",
    "pytest>=7.4.0",
]
```

## Configuration

### 1. Create Environment File

Create a `.env` file in your project root:

```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_HOST=https://your-cluster.qdrant.tech
QDRANT_API_KEY=your_qdrant_api_key_here

# Optional: Custom collection name (defaults to "book_embeddings")
QDRANT_COLLECTION=humanoid_robotics_book_embeddings

# Optional: Processing configuration
CHUNK_SIZE=300
CHUNK_OVERLAP=50
```

### 2. Load Environment Variables

In your Python code, load the environment variables:

```python
from dotenv import load_dotenv
import os

load_dotenv()

cohere_api_key = os.getenv("COHERE_API_KEY")
qdrant_host = os.getenv("QDRANT_HOST")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
```

## Basic Usage

### 1. Text Extraction Example

```python
import requests
from bs4 import BeautifulSoup

def extract_text_from_docusaurus(url):
    """Extract clean text from a Docusaurus book URL."""
    response = requests.get(url)
    response.raise_for_status()

    soup = BeautifulSoup(response.content, 'html.parser')

    # Remove navigation and other non-content elements
    for element in soup(['nav', 'header', 'footer', 'aside']):
        element.decompose()

    # Find the main content area (Docusaurus typically uses main or article tags)
    main_content = soup.find('main') or soup.find('article') or soup.find(class_='container')

    if main_content:
        # Extract text while preserving paragraph structure
        paragraphs = main_content.find_all(['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li'])
        text_content = '\n\n'.join([p.get_text().strip() for p in paragraphs if p.get_text().strip()])
    else:
        # Fallback: extract all text
        text_content = soup.get_text(separator='\n\n').strip()

    return text_content

# Example usage
url = "https://jawaid.github.io/humanoid-robotics-book/intro"
text = extract_text_from_docusaurus(url)
print(f"Extracted {len(text)} characters from {url}")
```

### 2. Text Chunking Example

```python
def chunk_text(text, chunk_size=300, overlap=50):
    """Chunk text into appropriately sized segments."""
    words = text.split()
    chunks = []

    start_idx = 0
    while start_idx < len(words):
        end_idx = start_idx + chunk_size
        chunk_words = words[start_idx:end_idx]
        chunk_text = ' '.join(chunk_words)

        chunks.append({
            'content': chunk_text,
            'start_idx': start_idx,
            'end_idx': end_idx,
            'token_count': len(chunk_words)
        })

        # Move start index by (chunk_size - overlap) to create overlap
        start_idx = end_idx - overlap if end_idx < len(words) else len(words)

    return chunks

# Example usage
chunks = chunk_text(text, chunk_size=300, overlap=50)
print(f"Created {len(chunks)} chunks from the text")
```

### 3. Embedding Generation Example

```python
import cohere

def generate_embeddings(chunks, cohere_api_key):
    """Generate embeddings for text chunks using Cohere."""
    co = cohere.Client(cohere_api_key)

    # Extract just the content from chunks for embedding
    texts = [chunk['content'] for chunk in chunks]

    # Generate embeddings (batch process up to 96 texts at a time)
    response = co.embed(
        texts=texts,
        model="embed-multilingual-v2.0",  # or "embed-english-v2.0"
        input_type="search_document"
    )

    # Combine chunks with their embeddings
    for i, chunk in enumerate(chunks):
        chunk['embedding'] = response.embeddings[i]

    return chunks

# Example usage
chunks_with_embeddings = generate_embeddings(chunks, cohere_api_key)
print(f"Generated embeddings for {len(chunks_with_embeddings)} chunks")
```

### 4. Qdrant Storage Example

```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

def store_embeddings_in_qdrant(chunks, qdrant_host, qdrant_api_key, collection_name="book_embeddings"):
    """Store embeddings in Qdrant vector database."""
    client = QdrantClient(
        url=qdrant_host,
        api_key=qdrant_api_key,
        prefer_grpc=False
    )

    # Create collection if it doesn't exist
    try:
        client.get_collection(collection_name)
    except:
        # Create collection with appropriate vector size (768 for Cohere multilingual model)
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE)
        )

    # Prepare points for insertion
    points = []
    for i, chunk in enumerate(chunks):
        points.append(
            models.PointStruct(
                id=i,
                vector=chunk['embedding'],
                payload={
                    "chunk_content": chunk['content'],
                    "chunk_index": chunk['start_idx'],
                    "source_text_length": len(chunk['content']),
                    "token_count": chunk['token_count']
                }
            )
        )

    # Upload points to collection
    client.upsert(
        collection_name=collection_name,
        points=points
    )

    return len(points)

# Example usage
stored_count = store_embeddings_in_qdrant(
    chunks_with_embeddings,
    qdrant_host,
    qdrant_api_key,
    os.getenv("QDRANT_COLLECTION", "book_embeddings")
)
print(f"Stored {stored_count} embeddings in Qdrant")
```

## Complete Pipeline Example

Here's a complete implementation following the required function structure in a single `main.py` file:

```python
import os
from dotenv import load_dotenv
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from urllib.parse import urljoin, urlparse
import time

def get_all_urls(base_url="https://jawaidali735.github.io/humanoid-robotics-book/"):
    """Get all Docusaurus book URLs from the provided site."""
    load_dotenv()

    response = requests.get(base_url)
    response.raise_for_status()

    soup = BeautifulSoup(response.content, 'html.parser')

    # Find all links in the navigation and content areas
    links = soup.find_all('a', href=True)

    urls = set()
    base_parsed = urlparse(base_url)
    base_netloc = base_parsed.netloc
    base_scheme = base_parsed.scheme

    for link in links:
        href = link['href']
        # Convert relative URLs to absolute URLs
        absolute_url = urljoin(base_url, href)

        # Only include URLs from the same domain that are part of the book
        parsed = urlparse(absolute_url)
        if parsed.netloc == base_netloc and parsed.scheme in ['http', 'https']:
            if absolute_url.endswith(('.html', '/')) and base_url in absolute_url:
                urls.add(absolute_url)

    # Also check for sitemap or other navigation elements that might contain more URLs
    # Look for common Docusaurus navigation patterns
    nav_links = soup.find_all('nav')
    for nav in nav_links:
        nav_a_tags = nav.find_all('a', href=True)
        for link in nav_a_tags:
            href = link['href']
            absolute_url = urljoin(base_url, href)
            parsed = urlparse(absolute_url)
            if parsed.netloc == base_netloc and parsed.scheme in ['http', 'https']:
                if absolute_url.endswith(('.html', '/')) and base_url in absolute_url:
                    urls.add(absolute_url)

    return list(urls)

def extract_text_from_urls(urls):
    """Extract clean text from the provided URLs while preserving semantic structure."""
    load_dotenv()

    cohere_api_key = os.getenv("COHERE_API_KEY")
    text_contents = []

    for url in urls:
        try:
            print(f"Processing URL: {url}")
            response = requests.get(url)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove navigation and other non-content elements typical in Docusaurus
            for element in soup(['nav', 'header', 'footer', 'aside', 'div[class*="docSidebarContainer"]']):
                element.decompose()

            # Find the main content area (Docusaurus typically uses main or article tags or specific classes)
            main_content = (soup.find('main') or
                          soup.find('article') or
                          soup.find(class_='container') or
                          soup.find(class_='docItemContainer') or
                          soup.find(class_='theme-doc-markdown'))

            if main_content:
                # Extract text while preserving some structure
                paragraphs = main_content.find_all(['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'td', 'th'])
                text_content = '\n\n'.join([p.get_text().strip() for p in paragraphs if p.get_text().strip()])
            else:
                # Fallback: extract all text
                text_content = soup.get_text(separator='\n\n').strip()

            # Clean up the text content
            text_content = '\n'.join([line.strip() for line in text_content.split('\n') if line.strip()])

            text_contents.append({
                'url': url,
                'content': text_content,
                'title': soup.title.string if soup.title else url
            })

            # Add a small delay to be respectful to the server
            time.sleep(0.5)

        except Exception as e:
            print(f"Error processing {url}: {str(e)}")
            continue

    return text_contents

def chunk_text(text, chunk_size=300, overlap=50):
    """Chunk text into appropriately sized segments."""
    words = text.split()
    chunks = []

    start_idx = 0
    while start_idx < len(words):
        end_idx = start_idx + chunk_size
        chunk_words = words[start_idx:end_idx]
        chunk_text = ' '.join(chunk_words)

        chunks.append({
            'content': chunk_text,
            'start_idx': start_idx,
            'end_idx': end_idx,
            'token_count': len(chunk_words)
        })

        # Move start index by (chunk_size - overlap) to create overlap
        start_idx = end_idx - overlap if end_idx < len(words) else len(words)

    return chunks

def embed(texts):
    """Generate embeddings for text chunks using Cohere."""
    load_dotenv()

    cohere_api_key = os.getenv("COHERE_API_KEY")
    co = cohere.Client(cohere_api_key)

    # Generate embeddings (process in batches if there are many texts)
    response = co.embed(
        texts=texts,
        model="embed-multilingual-v2.0",  # or "embed-english-v2.0"
        input_type="search_document"
    )

    return response.embeddings

def create_collection(collection_name="book_text_embedding"):
    """Create a Qdrant collection for storing embeddings."""
    load_dotenv()

    qdrant_host = os.getenv("QDRANT_HOST")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    client = QdrantClient(
        url=qdrant_host,
        api_key=qdrant_api_key,
        prefer_grpc=False
    )

    # Create collection if it doesn't exist
    try:
        client.get_collection(collection_name)
        print(f"Collection '{collection_name}' already exists")
    except:
        # Create collection with appropriate vector size (768 for Cohere multilingual model)
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE)
        )
        print(f"Created collection '{collection_name}' with 768-dimensional vectors")

    return client

def save_chunk_to_qdrant(client, collection_name, chunk_content, embedding, metadata=None):
    """Save individual text chunk with embedding to Qdrant."""
    if metadata is None:
        metadata = {}

    # Generate a unique ID for this chunk (could be based on content hash or UUID)
    import uuid
    point_id = str(uuid.uuid4())

    # Prepare the point to be inserted
    point = models.PointStruct(
        id=point_id,
        vector=embedding,
        payload={
            "chunk_content": chunk_content,
            **metadata  # Include any additional metadata
        }
    )

    # Insert the point into the collection
    client.upsert(
        collection_name=collection_name,
        points=[point]
    )

    return point_id

def last_main():
    """Main function to orchestrate the complete pipeline and test retrieval."""
    load_dotenv()

    print("Starting the text extraction and embedding pipeline...")

    # Step 1: Get all URLs from the Docusaurus site
    print("Step 1: Getting all URLs from the Docusaurus site...")
    base_url = "https://jawaidali735.github.io/humanoid-robotics-book/"
    urls = get_all_urls(base_url)
    print(f"Found {len(urls)} URLs to process")

    # For testing purposes, limit to a few URLs initially
    urls = urls[:5]  # Adjust this number based on how many pages you want to process

    # Step 2: Extract text from URLs
    print("Step 2: Extracting text from URLs...")
    text_contents = extract_text_from_urls(urls)
    print(f"Extracted text from {len(text_contents)} pages")

    # Step 3: Process each text content
    print("Step 3: Processing text contents...")
    collection_name = "book_text_embedding"

    # Create the Qdrant collection
    client = create_collection(collection_name)

    total_chunks_processed = 0
    for i, text_data in enumerate(text_contents):
        print(f"Processing content {i+1}/{len(text_contents)}: {text_data['title']}")

        # Chunk the text
        chunks = chunk_text(text_data['content'], chunk_size=300, overlap=50)
        print(f"Created {len(chunks)} chunks for {text_data['title']}")

        # Process each chunk
        for j, chunk in enumerate(chunks):
            print(f"Processing chunk {j+1}/{len(chunks)}")

            # Generate embedding for the chunk
            embeddings = embed([chunk['content']])
            embedding_vector = embeddings[0]

            # Prepare metadata
            metadata = {
                'source_url': text_data['url'],
                'source_title': text_data['title'],
                'chunk_index': j,
                'total_chunks': len(chunks),
                'token_count': chunk['token_count']
            }

            # Save to Qdrant
            point_id = save_chunk_to_qdrant(
                client,
                collection_name,
                chunk['content'],
                embedding_vector,
                metadata
            )

            total_chunks_processed += 1
            print(f"Saved chunk to Qdrant with ID: {point_id}")

    print(f"Pipeline completed! Processed and stored {total_chunks_processed} chunks.")

    # Step 4: Test retrieval from Qdrant and log outcomes
    print("Step 4: Testing retrieval from Qdrant...")

    # Perform a sample search to test retrieval
    try:
        # Search for a sample query
        sample_query = embed(["humanoid robotics"])
        search_results = client.search(
            collection_name=collection_name,
            query_vector=sample_query[0],
            limit=3  # Get top 3 results
        )

        print("Sample search results:")
        for idx, result in enumerate(search_results):
            print(f"Result {idx+1}:")
            print(f"  Content preview: {result.payload['chunk_content'][:100]}...")
            print(f"  Source: {result.payload['source_title']}")
            print(f"  Similarity score: {result.score}")
            print()

        print("Retrieval test completed successfully!")

    except Exception as e:
        print(f"Error during retrieval test: {str(e)}")

    print("Complete pipeline execution finished!")

# Example usage
if __name__ == "__main__":
    last_main()
```

## Testing

Create a `tests/` directory and add basic tests:

```python
# tests/test_extraction.py
import pytest
from src.text_extractor import extract_text_from_docusaurus

def test_extract_text_from_docusaurus():
    # This would require a mock or test URL
    # For now, we'll just test that the function exists and doesn't crash
    pass

def test_chunk_text():
    from src.text_chunker import chunk_text

    text = "This is a test sentence. " * 50  # Create a longer text
    chunks = chunk_text(text, chunk_size=20, overlap=5)

    assert len(chunks) > 0
    assert all(len(chunk['content']) > 0 for chunk in chunks)
    assert all(chunk['token_count'] <= 20 for chunk in chunks)
```

Run tests with:
```bash
uv run pytest
```

## Running the Application

Once you have your complete implementation, run it with:

```bash
python -m src.main  # or whatever your main module is called
```

## Troubleshooting

### Common Issues

1. **Rate Limiting**: If you encounter rate limits from Cohere API, implement retry logic with exponential backoff.

2. **Memory Issues**: For very large documents, process content in streaming fashion rather than loading everything into memory.

3. **Qdrant Connection**: Ensure your Qdrant Cloud URL and API key are correct in the environment variables.

4. **Text Extraction Quality**: If text extraction isn't working well for a specific Docusaurus site, inspect the HTML structure and adjust the selectors accordingly.