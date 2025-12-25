# Feature Specification: Text Embedding & Retrieval System

**Feature Branch**: `1-text-embedding-retrieval`
**Created**: 2025-12-21
**Status**: Draft
**Input**: User description: "Backend Embedding & Retrieval Prompt

## Role
You are a senior backend AI engineer working strictly under *Spec-Kit Plus (spec-driven development)* rules.

## Goal
Build a *Python backend system* for a *Humanoid Robotics Book* that performs:
- URL text extraction
- Text chunking
- Embedding using *Cohere*
- *Explicit Qdrant collection creation*
- Vector storage in *Qdrant*
- Semantic retrieval using the *same Cohere model*

Backend only. No UI. No shortcuts

---

## Mandatory Folder Structure (STRICT)

backend/
├── data_embedding/
│   ├── url_extractor.py
│   ├── text_chunker.py
│   ├── embedder.py
│   ├── qdrant_client.py
│   ├── qdrant_collection.py
│   ├── qdrant_store.py
│   ├── ingestion.py
│
├── data_retrieval/
│   ├── get_query_embedder.py
│   ├── retriever.py
│
├── config/
│   ├── settings.py
│
├── main.py

---

## Data Embedding Specifications

### 1. URL Text Extraction
- Create a function that takes a URL from (https://jawaidali735.github.io/humanoid-robotics-book/sitemap.xml)
- Extract clean readable text
- No chunking here
- Output: raw text string

---

### 2. Text Chunking
- Create a separate function
- Split extracted text into chunks
- Chunk size must be configurable
- Output: list of text chunks

---

### 3. Embedding Generation (Cohere)
- Create a function that:
  - Uses *Cohere embedding model*
  - Converts text chunks into vectors
- The *same Cohere model* must be reused everywhere
- No hardcoding

---

### 4. Qdrant Client Initialization
- Create a dedicated client initializer
- Support local or cloud Qdrant
- No collection logic here

---

### 5. Qdrant Collection Creation (MANDATORY & SEPARATE)
- Create a *separate function* ONLY for collection creation
- Collection name MUST be: humanoid_robotics_book
- Vector size must match Cohere embedding output
- Distance metric: COSINE
- If collection already exists:
  - Do NOT recreate
  - Safely skip creation

---

### 6. Store Data in Qdrant
- Create a function to store embeddings
- Store vectors ONLY after collection exists
- Each vector must include metadata:
  - chunk_id
  - source_url

---

### 7. Ingestion Controller
- Create a main ingestion function
- It must execute steps in order:
  1. URL → text extraction
  2. Text → chunks
  3. Chunks → embeddings
  4. Initialize Qdrant client
  5. *Create Qdrant collection*
  6. Save embeddings
- If any step fails → stop execution
- Log success/failure clearly

---

## Data Retrieval Specifications

### 8. Query Embedding
- Create a function that embeds user query
- Must use the *same Cohere model* as ingestion

---

### 9. Vector Retrieval
- Create a retrieval function
- Perform similarity search from Qdrant
- Return top relevant text chunks

---

## Configuration Rules
- All keys, URLs, model names must live in config/settings.py
- No hard-coded values anywhere

---

## Quality Rules (NON-NEGOTIABLE)
- One function = one responsibility
- Collection creation must NEVER be mixed with storage
- Modular, testable code
- Clean docstrings
- Backend only
- P"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Text Ingestion Pipeline (Priority: P1)

System administrators need to ingest text content from URLs into a vector database so that users can perform semantic searches against the content. The system must extract text from URLs, chunk it, generate embeddings, and store them in Qdrant.

**Why this priority**: This is the foundational functionality that enables all subsequent search capabilities.

**Independent Test**: Can be fully tested by providing a URL, verifying that text is extracted, chunked, embedded, and stored in Qdrant with proper metadata.

**Acceptance Scenarios**:

1. **Given** a valid URL with text content, **When** the ingestion process is triggered, **Then** the text is successfully extracted, chunked, embedded, and stored in Qdrant with proper metadata
2. **Given** an invalid URL or URL with no text content, **When** the ingestion process is triggered, **Then** the system handles the error gracefully with appropriate logging

---

### User Story 2 - Semantic Text Retrieval (Priority: P2)

Users need to search through the ingested text content using natural language queries so that they can find relevant information about humanoid robotics topics. The system must convert queries to embeddings and retrieve semantically similar text chunks.

**Why this priority**: This provides the core search functionality that users will interact with.

**Independent Test**: Can be fully tested by providing a query string and verifying that semantically relevant text chunks are returned from the vector database.

**Acceptance Scenarios**:

1. **Given** a user query about humanoid robotics, **When** the retrieval process is triggered, **Then** the system returns the most relevant text chunks from the stored content
2. **Given** a query that doesn't match any stored content, **When** the retrieval process is triggered, **Then** the system returns an appropriate response indicating no relevant results

---

### User Story 3 - Configuration Management (Priority: P3)

System administrators need to configure API keys, model parameters, and service endpoints through a centralized configuration so that the system can be deployed in different environments without code changes.

**Why this priority**: This enables flexible deployment and management of the system.

**Independent Test**: Can be fully tested by verifying that configuration values are properly loaded and used throughout the system without hardcoding.

**Acceptance Scenarios**:

1. **Given** configuration parameters in settings.py, **When** the system initializes, **Then** all services use the configured values for API keys, endpoints, and model parameters

---

### Edge Cases

- What happens when a URL is inaccessible or returns an error?
- How does the system handle extremely large text documents that exceed memory limits?
- What happens when the Qdrant collection already exists?
- How does the system handle malformed URLs or non-text content?
- What happens when the Cohere API is unavailable or returns an error?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST extract clean readable text from provided URLs
- **FR-002**: The system MUST chunk extracted text with configurable chunk size parameters
- **FR-003**: The system MUST generate embeddings using the Cohere embedding model
- **FR-004**: The system MUST initialize and connect to a Qdrant vector database
- **FR-005**: The system MUST create a Qdrant collection named "humanoid_robotics_book" with COSINE distance metric
- **FR-006**: The system MUST store embeddings with metadata (chunk_id and source_url) in Qdrant
- **FR-007**: The system MUST embed user queries using the same Cohere model as the stored content
- **FR-008**: The system MUST perform semantic similarity search against stored vectors in Qdrant
- **FR-009**: The system MUST return the most relevant text chunks based on query similarity
- **FR-010**: The system MUST handle errors gracefully and provide appropriate logging
- **FR-011**: All configuration parameters MUST be stored in config/settings.py without hardcoding
- **FR-012**: Collection creation MUST be a separate function from storage operations
- **FR-013**: The system MUST check if a Qdrant collection exists before attempting creation

### Key Entities *(include if feature involves data)*

- **Text Chunk**: A segment of extracted text with associated metadata (chunk_id, source_url)
- **Embedding Vector**: A numerical representation of text content generated by the Cohere model
- **Qdrant Collection**: A container for storing embedding vectors with metadata in the vector database
- **Query**: A text string provided by users for semantic search against stored content

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The system successfully extracts text from 95% of valid URLs provided
- **SC-002**: The system processes and stores text embeddings with less than 5% failure rate
- **SC-003**: Semantic search returns relevant results within 2 seconds for 90% of queries
- **SC-004**: The system handles configuration parameters without requiring code changes
- **SC-005**: All system components are modular with single responsibility and clear interfaces
- **SC-006**: The system properly manages Qdrant collections without conflicts or duplicate creation