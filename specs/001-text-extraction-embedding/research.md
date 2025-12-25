# Research Document: Text Extraction & Embedding Pipeline

**Feature**: 001-text-extraction-embedding
**Created**: 2025-12-16
**Status**: Complete

## Research Findings

### 1. Embedding Model API Selection

**Decision**: Cohere embedding models (recommended: embed-multilingual-v2.0 or embed-english-v2.0)
**Rationale**:
- Cohere provides high-quality embeddings with good semantic understanding
- Well-documented API with Python SDK
- Reasonable pricing for development use
- Supports up to 512 tokens per input
- Good performance for technical documentation content

**Alternatives considered**:
- OpenAI embeddings: Higher cost, 8192 token limit, good quality
- Sentence Transformers (open-source): Free but requires more computational resources
- Google's text embeddings: Good quality but less familiar ecosystem

### 2. Docusaurus Book URL for Testing

**Decision**: Use the specified humanoid robotics book deployed at the GitHub Pages URL
**Rationale**:
- The book is already deployed and accessible
- Contains technical content appropriate for RAG system testing
- Properly structured with Docusaurus navigation and content
- Publicly accessible for testing purposes

**Note**: The target URL is: https://jawaidali735.github.io/humanoid-robotics-book/

### 3. Qdrant Cloud Configuration

**Decision**: Use Qdrant Cloud with the following configuration details
**Rationale**:
- Qdrant is a purpose-built vector database with excellent performance
- Cloud offering provides managed service with high availability
- Good Python client library with comprehensive documentation
- Supports metadata filtering and efficient similarity search

**Configuration details needed**:
- QDRANT_HOST: Cloud endpoint URL (e.g., "https://your-cluster.qdrant.tech")
- QDRANT_API_KEY: Secure API key for authentication
- Collection name: "humanoid_robotics_book_embeddings" (or similar)
- Vector dimensions: 768 (for Cohere embed-multilingual-v2.0) or 1024 (for embed-english-v2.0)

### 4. Token Limit and Chunking Strategy

**Decision**: Use 300-400 token chunks with 50-token overlap for optimal balance
**Rationale**:
- Cohere embed-multilingual-v2.0 supports up to 512 tokens per input
- 300-400 token chunks provide good semantic coherence
- 50-token overlap helps preserve context across chunks
- Allows for metadata and special tokens in the embedding process

**Alternative strategies considered**:
- Character-based chunking: Less semantic awareness but simpler
- Sentence-based chunking: Better semantic boundaries but variable lengths
- Recursive chunking: More sophisticated but potentially over-engineered

## Best Practices Identified

### Web Scraping Best Practices
- Respect robots.txt and rate limiting
- Use appropriate user agents
- Implement retry logic with exponential backoff
- Handle different content types appropriately

### Text Processing Best Practices
- Preserve semantic structure (headings, lists, code blocks)
- Remove navigation elements and boilerplate content
- Handle special formatting (math formulas, code snippets)
- Maintain document hierarchy in metadata

### Embedding Generation Best Practices
- Batch requests to optimize API usage
- Implement proper error handling for API failures
- Cache embeddings to avoid redundant API calls
- Monitor token usage for cost optimization

### Vector Database Best Practices
- Use appropriate indexing strategies for the query patterns
- Implement proper metadata schema for filtering
- Monitor collection size and implement retention policies
- Use efficient distance metrics for similarity search