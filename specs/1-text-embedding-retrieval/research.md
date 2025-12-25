# Research: Text Embedding & Retrieval System

## Decision: Cohere Embedding Model Selection
**Rationale**: Cohere provides stable embedding dimensions and strong semantic performance for technical text. The `embed-english-v3.0` model is recommended for its balance of performance and cost.
**Alternatives considered**: OpenAI embeddings, Hugging Face models, Gemini embeddings. Cohere was chosen for its specific focus on semantic search and consistent performance with technical documentation.

## Decision: Qdrant Vector Database
**Rationale**: Qdrant offers explicit collection control and open-source flexibility. It provides native support for cosine similarity which is required by the specification.
**Alternatives considered**: Pinecone, Weaviate, Vespa. Qdrant was chosen for its explicit collection management capabilities and open-source nature.

## Decision: Text Extraction Method
**Rationale**: Using requests + BeautifulSoup4 for clean text extraction from web pages. This combination provides reliable extraction of readable text while filtering out HTML noise.
**Alternatives considered**: Newspaper3k, Trafilatura, readability. BeautifulSoup4 was chosen for its simplicity and effectiveness with structured documentation sites.

## Decision: Text Chunking Strategy
**Rationale**: Fixed-size configurable chunks ensure deterministic ingestion and simpler testing. A chunk size of 512 tokens (approximately 384 words) balances retrieval accuracy with context preservation.
**Alternatives considered**: Semantic chunking, recursive splitting. Fixed-size chunking was chosen for its predictability and easier parameter tuning.

## Decision: Qdrant Collection Creation Strategy
**Rationale**: Explicit collection creation function that checks for existence before creation prevents duplicate collections and ensures proper schema setup.
**Alternatives considered**: Auto-create on insert. Explicit creation was chosen to match the specification requirement for separate collection creation function.

## Technical Specifications Resolved

### Cohere Embedding Dimensions
- Model: `embed-english-v3.0`
- Output dimensions: 1024 (for text type embeddings)
- This matches the specification requirement for vector size to match Cohere output

### Qdrant Configuration
- Distance metric: COSINE (as required by specification)
- Collection name: `humanoid_robotics_book`
- Vector size: 1024 (matching Cohere embedding dimensions)

### URL Processing
- Source: https://jawaidali735.github.io/humanoid-robotics-book/sitemap.xml
- Text extraction method: requests + BeautifulSoup4 for clean text
- Chunk size: Configurable, default 1000 characters with 200 character overlap

### Metadata Requirements
- chunk_id: Sequential identifier for each text chunk
- source_url: Original URL where the text was found
- Additional metadata: Process timestamp, chunk position in original document

## Dependencies and Setup

### Required Python Packages
- cohere: For embedding generation
- qdrant-client: For vector database operations
- requests: For URL content retrieval
- beautifulsoup4: For text extraction from HTML
- python-dotenv: For configuration management

### Configuration Parameters
- COHERE_API_KEY: API key for Cohere service
- QDRANT_URL: URL for Qdrant instance (local or cloud)
- QDRANT_API_KEY: API key for Qdrant (if using cloud)
- CHUNK_SIZE: Size of text chunks in characters
- CHUNK_OVERLAP: Overlap between chunks to preserve context
- COHERE_MODEL: Name of the Cohere embedding model to use