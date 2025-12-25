# Data Model: Text Embedding & Retrieval System

## Entity: Text Chunk
- **Attributes**:
  - chunk_id: string (unique identifier)
  - content: string (the actual text content)
  - source_url: string (URL where text was extracted from)
  - position: integer (position of chunk in original document)
  - embedding: vector (Cohere-generated embedding vector, 1024 dimensions)
  - created_at: timestamp (when chunk was processed)
  - processed: boolean (whether embedding has been generated)

- **Validation Rules**:
  - content must not be empty
  - source_url must be a valid URL format
  - chunk_id must be unique within system
  - embedding vector must have 1024 dimensions (for Cohere model)

- **Relationships**:
  - Belongs to one source_url
  - Part of one Document entity (conceptual, not stored)

## Entity: Embedding Vector
- **Attributes**:
  - vector_id: string (Qdrant point ID, same as chunk_id)
  - vector: array<float> (1024-dimensional embedding from Cohere)
  - metadata: object (contains chunk_id, source_url, and other metadata)
  - payload: object (Qdrant-specific payload with searchable fields)

- **Validation Rules**:
  - vector must have exactly 1024 dimensions
  - vector_id must match chunk_id format
  - metadata must contain required fields (chunk_id, source_url)

- **Relationships**:
  - Maps 1:1 with Text Chunk
  - Stored in Qdrant collection "humanoid_robotics_book"

## Entity: Qdrant Collection
- **Attributes**:
  - name: string (fixed as "humanoid_robotics_book")
  - vector_size: integer (fixed as 1024 for Cohere embeddings)
  - distance: string (fixed as "COSINE")
  - status: string (active, creating, error)

- **Validation Rules**:
  - name must be "humanoid_robotics_book"
  - vector_size must match Cohere embedding dimensions (1024)
  - distance must be "COSINE" as specified

- **Relationships**:
  - Contains multiple Embedding Vector entities
  - Associated with single Cohere embedding model

## Entity: Document Processing Job
- **Attributes**:
  - job_id: string (unique identifier for processing job)
  - source_urls: array<string> (list of URLs to process)
  - status: string (pending, processing, completed, failed)
  - created_at: timestamp (when job was created)
  - completed_at: timestamp (when job was completed)
  - processed_count: integer (number of chunks successfully processed)
  - failed_count: integer (number of chunks that failed)

- **Validation Rules**:
  - source_urls must contain valid URLs
  - status must be one of the defined values
  - processed_count and failed_count must be non-negative

- **Relationships**:
  - Creates multiple Text Chunk entities
  - Associated with multiple Embedding Vector entities

## State Transitions

### Document Processing Job
- Initial state: pending
- Transitions to: processing (when URL extraction begins)
- Transitions to: completed (when all URLs processed successfully)
- Transitions to: failed (when errors occur during processing)

### Text Chunk
- Initial state: extracted (text extracted from URL)
- Transitions to: chunked (when text is split into chunks)
- Transitions to: embedded (when Cohere embedding is generated)
- Transitions to: stored (when vector is saved to Qdrant)

## Data Flow Validation

### Ingestion Pipeline
1. URL → Text Extraction → Raw Text
2. Raw Text → Chunking → Text Chunk entities
3. Text Chunk → Embedding → Embedding Vector entities
4. Embedding Vector → Storage → Qdrant Collection

### Retrieval Pipeline
1. Query String → Embedding → Query Vector
2. Query Vector → Similarity Search → Matching Embedding Vectors
3. Matching Vectors → Content Retrieval → Relevant Text Chunks
4. Relevant Chunks → Ranked Results → Final Response

## Schema Constraints

### Qdrant Collection Schema
- Vector dimension: 1024 (fixed for Cohere embeddings)
- Distance metric: COSINE (as specified)
- Payload fields:
  - chunk_id: keyword (indexed for fast lookup)
  - source_url: keyword (indexed for filtering)
  - content: text (for full-text search capability)
  - created_at: integer (timestamp for temporal queries)

### Configuration Requirements
- All configuration parameters must be validated at startup
- API keys must be present before processing begins
- Qdrant connection must be verified before collection operations
- Cohere model availability must be confirmed before embedding operations