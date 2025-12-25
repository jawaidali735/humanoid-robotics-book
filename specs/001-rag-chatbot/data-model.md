# Data Model: RAG Textbook Chatbot Backend

## Entities

### UserQuery
**Description**: Text input from users asking questions about the textbook content
**Fields**:
- query_id: string (unique identifier)
- content: string (the actual question text)
- timestamp: datetime (when the query was submitted)
- user_context: object (optional user session information)

### TextbookContentChunk
**Description**: Segments of extracted text from the textbook pages that have been processed and embedded for vector search
**Fields**:
- chunk_id: string (unique identifier)
- content: string (the text content of the chunk)
- source_url: string (URL of the original textbook page)
- page_title: string (title of the source page)
- embedding: array (vector embedding of the content)
- metadata: object (additional information about the chunk)

### VectorEmbedding
**Description**: Numerical representation of text content created using Google Gemini embeddings
**Fields**:
- embedding_id: string (unique identifier)
- vector: array (768-dimensional vector)
- content_id: string (reference to the source content chunk)
- model: string (embedding model used)

### RetrievedContext
**Description**: Relevant content segments retrieved from the vector database based on query similarity
**Fields**:
- retrieval_id: string (unique identifier)
- query_embedding: array (the embedding of the user query)
- matched_chunks: array (list of matching content chunks)
- similarity_scores: array (scores for each matched chunk)
- retrieval_timestamp: datetime (when retrieval was performed)

### GeneratedResponse
**Description**: AI-generated answer to user questions that incorporates retrieved context
**Fields**:
- response_id: string (unique identifier)
- query_id: string (reference to the original query)
- content: string (the generated answer)
- source_citations: array (list of source URLs used)
- generation_timestamp: datetime (when response was generated)
- confidence_score: number (confidence in the response accuracy)

### SourceCitation
**Description**: URLs and page references pointing to the original textbook content
**Fields**:
- citation_id: string (unique identifier)
- url: string (source URL)
- title: string (page title)
- snippet: string (relevant text snippet)
- relevance_score: number (how relevant this source is to the query)

## Relationships

- UserQuery → GeneratedResponse (one-to-many: one query can generate one response)
- TextbookContentChunk → VectorEmbedding (one-to-one: each chunk has one embedding)
- UserQuery → RetrievedContext (one-to-one: each query gets one retrieval result)
- RetrievedContext → TextbookContentChunk (many-to-many: retrieval can match multiple chunks)
- GeneratedResponse → SourceCitation (one-to-many: response can cite multiple sources)

## Validation Rules

- UserQuery.content must be non-empty string
- TextbookContentChunk.content must be non-empty and less than 4000 tokens
- SourceCitation.url must be a valid URL format
- VectorEmbedding.vector must have exactly 768 dimensions
- GeneratedResponse must include at least one source citation