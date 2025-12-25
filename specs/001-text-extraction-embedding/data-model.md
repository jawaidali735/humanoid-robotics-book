# Data Model: Text Extraction & Embedding Pipeline

**Feature**: 001-text-extraction-embedding
**Created**: 2025-12-16
**Status**: Draft

## Entities

### TextContent
Represents the extracted content from a Docusaurus book URL (in single-file implementation)

**Fields**:
- `url` (string, required): Source URL of the Docusaurus book page
- `title` (string, required): Title of the document/page
- `content` (string, required): Clean text content extracted from the page
- `metadata` (object, optional): Additional information like headings, sections

**Validation Rules**:
- URL must be a valid, accessible Docusaurus page URL
- Content must be non-empty after extraction

### TextChunk
A segmented portion of text content, optimized for embedding generation (in single-file implementation)

**Fields**:
- `content` (string, required): Chunked text content
- `chunk_index` (integer, required): Position of the chunk in the content sequence
- `token_count` (integer, required): Number of tokens in the chunk
- `source_url` (string, required): URL of the source document
- `source_title` (string, required): Title of the source document

**Validation Rules**:
- content must be non-empty
- chunk_index must be non-negative
- token_count must be within embedding model limits

### EmbeddingRecord
A record containing an embedding vector with associated metadata (in single-file implementation)

**Fields**:
- `chunk_content` (string, required): Original chunked text content
- `vector` (array, required): The embedding vector (array of floats)
- `vector_dimensions` (integer, required): Number of dimensions in the vector
- `collection` (string, required): Qdrant collection name
- `metadata` (object, required): Associated metadata for retrieval
  - `source_url` (string): Original URL of the content
  - `source_title` (string): Title of the source document
  - `chunk_index` (integer): Position of chunk in document
  - `total_chunks` (integer): Total number of chunks from the document
  - `token_count` (integer): Number of tokens in the chunk

**Validation Rules**:
- vector must be a valid array of floats
- vector_dimensions must match the embedding model specification (768 for Cohere multilingual model)
- collection name must be valid for Qdrant

## Processing Flow

### Single-File Implementation Flow
1. `get_all_urls()` retrieves all URLs from the Docusaurus site
2. `extract_text_from_urls()` processes each URL to extract clean text content
3. For each text content:
   - `chunk_text()` segments the content into appropriately sized chunks
   - Each chunk is processed with `embed()` to generate embeddings
   - `save_chunk_to_qdrant()` stores the chunk with its embedding in Qdrant
4. `last_main()` orchestrates the complete pipeline and tests retrieval