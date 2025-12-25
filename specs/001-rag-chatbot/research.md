# Research Document: RAG Textbook Chatbot Backend

## Decision: Architecture Pattern
**Rationale**: Selected a microservice architecture with separate modules for ingestion, embedding, retrieval, and chat functionality to ensure clean separation of concerns and maintainability.
**Alternatives considered**: Monolithic approach vs. microservices vs. serverless functions. Microservices provide better scalability and maintainability for the RAG pipeline.

## Decision: Technology Stack
**Rationale**: Using Python 3.11 with FastAPI provides excellent async support, automatic API documentation, and strong typing. Google Gemini with LangChain provides the required embedding and generation capabilities.
**Alternatives considered**:
- Alternative 1: Using OpenAI models vs. Google Gemini - Chosen Gemini due to project constraints
- Alternative 2: Different vector databases (Pinecone, Weaviate vs. Qdrant) - Chosen Qdrant due to free tier availability and LangChain integration
- Alternative 3: Different web frameworks (Flask vs. FastAPI) - Chosen FastAPI for better performance and built-in OpenAPI docs

## Decision: Embedding Model
**Rationale**: Using LangChain Gemini embeddings (text-embedding-004, 768 dims) as specified in technical requirements to maintain consistency with generation model.
**Alternatives considered**: Other embedding dimensions/models - 768 dimensions provide good balance of accuracy and performance.

## Decision: Deployment Strategy
**Rationale**: Docker container deployment to Hugging Face Spaces on port 7860 meets project constraints for free tier deployment.
**Alternatives considered**: Various cloud platforms vs. Hugging Face Spaces - Chosen Hugging Face due to project constraints.

## Decision: Content Processing Pipeline
**Rationale**: Sitemap crawling → HTML extraction → chunking → embedding approach provides systematic processing of textbook content.
**Alternatives considered**: Direct PDF processing vs. HTML crawling - HTML crawling chosen as textbook is web-based.

## Decision: API Design
**Rationale**: Standard REST endpoints for /chat and /health with proper error handling and response formatting.
**Alternatives considered**: GraphQL vs. REST - Chosen REST for simplicity and broad compatibility.