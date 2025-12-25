# Data Model: Retrieval-Enabled Agent

## Entity: Query
**Description**: Natural language question submitted by the user requesting information from the book
**Fields**:
- query_text (string): The user's question in natural language
- query_id (string): Unique identifier for the query
- timestamp (datetime): When the query was submitted
- user_context (object): Optional context about the user or session

**Validation rules**:
- query_text must be non-empty
- query_text must be less than 1000 characters to prevent abuse

## Entity: Retrieved Content
**Description**: Relevant book passages retrieved from Qdrant vector store based on semantic similarity to the query
**Fields**:
- content_id (string): Unique identifier for the content chunk
- content_text (string): The actual text content retrieved
- similarity_score (float): Score representing how similar this content is to the query
- source_reference (string): Reference to the original book section
- metadata (object): Additional metadata about the content source

**Validation rules**:
- content_text must be non-empty
- similarity_score must be between 0 and 1
- source_reference must be present and valid

## Entity: Grounded Response
**Description**: AI-generated answer that is strictly based on the retrieved content with proper attribution
**Fields**:
- response_id (string): Unique identifier for the response
- query_id (string): Reference to the original query
- response_text (string): The AI-generated response text
- source_attributions (array): List of source references used in the response
- confidence_level (float): Confidence level of the response accuracy
- timestamp (datetime): When the response was generated

**Validation rules**:
- response_text must be non-empty
- response must only contain information from retrieved content
- source_attributions must be non-empty when content is provided
- confidence_level must be between 0 and 1

## Entity: Vector Embedding
**Description**: Mathematical representation of book content segments stored in Qdrant for semantic search
**Fields**:
- embedding_id (string): Unique identifier for the embedding
- content_id (string): Reference to the original content
- vector_data (array): The actual vector representation
- metadata (object): Additional information about the embedding
- created_at (datetime): When the embedding was created

**Validation rules**:
- vector_data must have consistent dimensions
- content_id must reference a valid content item

## Relationships
- Query → Retrieved Content: One-to-many (one query can retrieve multiple content chunks)
- Retrieved Content → Grounded Response: One-to-many (multiple content chunks can be used in one response)
- Query → Grounded Response: One-to-one (each query generates one response)