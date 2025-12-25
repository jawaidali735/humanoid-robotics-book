# Implementation Plan: Text Extraction & Embedding Pipeline

**Feature**: 001-text-extraction-embedding
**Created**: 2025-12-16
**Status**: Draft
**Author**: Claude

## Technical Context

### Architecture Overview
The text extraction and embedding pipeline will be implemented as a single Python file (`main.py`) that processes Docusaurus book URLs to extract clean text, chunk it appropriately, generate embeddings, and store them in a vector database. The system will be designed with specific functions as outlined in the requirements: `get_all_urls()`, `extract_text_from_urls()`, `chunk_text()`, `embed()`, `create_collection()`, `save_chunk_to_qdrant()`, and orchestrated by a `last_main()` function.


### Technology Stack
- **Backend**: Python with appropriate libraries for web scraping, text processing, and API integration
- **Text Extraction**: Libraries for parsing HTML and extracting clean text content
- **Embedding Generation**: Integration with embedding model APIs (e.g., Cohere, OpenAI, or open-source alternatives)
- **Vector Database**: Qdrant for storing and retrieving embeddings
- **Package Management**: UV for dependency management
- **Configuration**: Environment variables for API keys and service configuration
- **Target Site**: https://jawaidali735.github.io/humanoid-robotics-book/
- **SiteMAp URL**: https://jawaidali735.github.io/humanoid-robotics-book/sitemap.xml

### System Components
1. **get_all_urls()**: Function to retrieve all Docusaurus book URLs from the provided site
2. **extract_text_from_urls()**: Function to extract clean text from the provided URLs while preserving semantic structure
3. **chunk_text()**: Function to segment text into appropriately sized chunks for embedding
4. **embed()**: Function to create semantic embeddings from text chunks using embedding models
5. **create_collection()**: Function to create a Qdrant collection named 'book_text_embedding'
6. **save_chunk_to_qdrant()**: Function to save individual text chunks with embeddings to Qdrant
7. **last_main()**: Orchestrator function that executes the complete pipeline and tests retrieval

### Unknowns
- [NEEDS CLARIFICATION] What specific embedding model API will be used (Cohere, OpenAI, or open-source)?
- [NEEDS CLARIFICATION] What is the exact URL of the target Docusaurus book for testing?
- [NEEDS CLARIFICATION] What are the specific Qdrant Cloud configuration details?
- [NEEDS CLARIFICATION] What are the exact token limits for the embedding model being used?

## Constitution Check

### Educational Clarity
- The implementation will include clear documentation and comments explaining each component
- Code will be structured to serve as a learning example for AI engineers
- Each module will include usage examples and error handling explanations

### Technical Accuracy
- The implementation will follow current best practices for web scraping, text processing, and vector databases
- Proper error handling and rate limiting will be implemented
- Security considerations for API keys and data handling will be addressed

### Practical Outcomes
- Each component will be tested with real Docusaurus book URLs
- Performance metrics will be included to verify processing speed and accuracy
- A complete working example will be provided for end-to-end testing

### Ethical Responsibility
- Proper rate limiting will be implemented to avoid overloading source servers
- API usage will follow terms of service for embedding services
- Data privacy considerations will be addressed for any stored content

### Original and Traceable Content
- All code will be original with proper attribution where applicable
- Dependencies will be clearly documented and managed through UV
- Implementation will follow established patterns from the literature

## Gate Evaluation

### Compliance Check
- ✅ Educational Clarity: Implementation will be well-documented with clear examples
- ✅ Technical Accuracy: Will use current best practices for each component
- ✅ Practical Outcomes: Each module will be tested with real data
- ✅ Ethical Responsibility: Will implement proper rate limiting and API usage
- ✅ Original Content: All code will be original

## Phase 0: Research & Resolution of Unknowns

### Research Tasks

#### 1. Embedding Model API Selection
**Task**: Research and select the appropriate embedding model API
**Considerations**:
- Cost implications for API usage
- Performance and accuracy of different models
- Ease of integration with Python
- Token limits and rate limiting policies

#### 2. Docusaurus Book URL Identification
**Task**: Identify the specific Docusaurus book URL for testing
**Considerations**:
- Publicly accessible URL for testing
- Proper content structure for text extraction
- Appropriate size for testing purposes

#### 3. Qdrant Cloud Configuration
**Task**: Determine specific Qdrant Cloud configuration details
**Considerations**:
- Cloud endpoint URL
- API key management
- Collection naming conventions
- Vector dimension settings

#### 4. Token Limit Research
**Task**: Determine the exact token limits for the selected embedding model
**Considerations**:
- Maximum input length for the model
- Optimal chunk size for semantic coherence
- Overlap strategies between chunks

## Phase 1: Design & Contracts

### Data Model Design

#### Text Document Entity
- **id**: Unique identifier for the document
- **url**: Source URL of the Docusaurus book page
- **content**: Clean text content extracted from the page
- **metadata**: Additional information (title, headings, etc.)
- **created_at**: Timestamp of extraction
- **status**: Processing status (pending, processed, failed)

#### Text Chunk Entity
- **id**: Unique identifier for the chunk
- **document_id**: Reference to the parent document
- **content**: Chunked text content
- **chunk_index**: Position of the chunk in the document
- **token_count**: Number of tokens in the chunk
- **created_at**: Timestamp of chunking

#### Embedding Record Entity
- **id**: Unique identifier for the embedding record
- **chunk_id**: Reference to the text chunk
- **vector**: The embedding vector (array of floats)
- **metadata**: Associated metadata for retrieval
- **collection**: Qdrant collection name
- **created_at**: Timestamp of embedding creation

### API Contracts

#### Text Extraction Service
```
POST /extract
{
  "url": "https://example-docusaurus-book.com/page"
}
Response:
{
  "document_id": "uuid",
  "content": "clean text content",
  "metadata": {...}
}
```

#### Text Chunking Service
```
POST /chunk
{
  "document_id": "uuid",
  "content": "text content to be chunked",
  "options": {
    "chunk_size": 512,
    "overlap": 50
  }
}
Response:
{
  "chunks": [
    {
      "chunk_id": "uuid",
      "content": "chunked text",
      "index": 0
    }
  ]
}
```

#### Embedding Generation Service
```
POST /embed
{
  "chunks": [
    {
      "chunk_id": "uuid",
      "content": "text to embed"
    }
  ]
}
Response:
{
  "embeddings": [
    {
      "chunk_id": "uuid",
      "vector": [0.1, 0.2, ...],
      "dimension": 1536
    }
  ]
}
```

#### Storage Service
```
POST /store
{
  "embeddings": [
    {
      "chunk_id": "uuid",
      "vector": [0.1, 0.2, ...],
      "metadata": {...}
    }
  ],
  "collection": "book_embeddings"
}
Response:
{
  "status": "success",
  "stored_count": 5
}
```

### Quickstart Guide

#### Setup
1. Install UV package manager
2. Create a new Python project directory
3. Set up virtual environment with UV
4. Install required dependencies
5. Create a single `main.py` file with all required functions

#### Configuration
1. Set environment variables for API keys and service endpoints
2. Configure Qdrant Cloud connection details
3. Set up embedding model API credentials

#### Usage
1. Run the `last_main()` function to execute the complete pipeline
2. Monitor the processing progress
3. Verify embeddings are stored in Qdrant and test retrieval

## Phase 2: Implementation Plan

### Implementation Tasks
1. Set up project structure and dependencies with UV
2. Implement `get_all_urls()` function to retrieve all URLs from the Docusaurus site (https://jawaidali735.github.io/humanoid-robotics-book/)
3. Implement `extract_text_from_urls()` function to extract clean text from provided URLs
4. Implement `chunk_text()` function to segment text into appropriately sized chunks
5. Implement `embed()` function to generate embeddings using Cohere API
6. Implement `create_collection()` function to create 'book_text_embedding' collection in Qdrant
7. Implement `save_chunk_to_qdrant()` function to store chunks with embeddings in Qdrant
8. Implement `last_main()` function to orchestrate the complete pipeline
9. Add retrieval testing and logging in `last_main()` function
10. Add error handling and logging throughout all functions
11. Write tests for each function

### Dependencies to Install
- requests (for URL fetching)
- beautifulsoup4 (for HTML parsing)
- cohere (for embeddings) or equivalent
- qdrant-client (for vector database)
- python-dotenv (for configuration)
- pytest (for testing)

## Re-evaluation of Constitution Check Post-Design

### Updated Compliance Check
- ✅ Educational Clarity: API contracts and data models are clearly defined
- ✅ Technical Accuracy: Design follows established patterns for each component
- ✅ Practical Outcomes: Each service endpoint has a clear purpose and usage
- ✅ Ethical Responsibility: API usage and rate limiting are considered in design
- ✅ Original Content: Architecture is original and well-documented