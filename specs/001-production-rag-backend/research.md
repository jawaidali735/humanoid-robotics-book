# Research: Production RAG Backend (FastAPI + OpenAI Agents SDK + Qdrant)

## Overview
This document captures research findings for implementing a production-ready RAG backend using FastAPI, OpenAI Agents SDK, Qdrant, and Gemini API.

## Decision: OpenAI Agents SDK Integration Pattern
**Rationale**: Using the official OpenAI Agents SDK provides a clean separation between agent logic and tool execution. The SDK's `@function_tool` decorator allows existing retrieval functions to be easily exposed to the agent.

**Alternatives considered**:
- LangChain: More complex and would violate the constraint of only using OpenAI Agents SDK
- Custom agent implementation: Would require more development time and maintenance

## Decision: Gemini API via AsyncOpenAI Client
**Rationale**: The custom AsyncOpenAI client with Google endpoint allows using Gemini API through the OpenAI Agents SDK, meeting the requirement to use the official SDK while accessing Google's model.

**Implementation**:
```python
api_key = os.getenv("GEMINI_API_KEY")
external_client = AsyncOpenAI(
    api_key=api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)
model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=external_client
)
```

## Decision: Async FastAPI Endpoints
**Rationale**: Asynchronous endpoints are essential for handling multiple concurrent users and maintaining good performance, especially when waiting for external API calls (Qdrant and Gemini).

**Alternatives considered**:
- Synchronous endpoints: Would block on I/O operations and limit concurrency

## Decision: Existing Retrieval Integration
**Rationale**: The existing `data_retrieval` directory already contains working retrieval functionality that should be leveraged via the `@function_tool` decorator to create a retrieval tool for the agent.

**Implementation**: Create a wrapper function in `agents/tools.py` that calls the existing retrieval function from `data_retrieval`.

## Decision: Pydantic Schemas for API
**Rationale**: Using Pydantic schemas ensures type safety, automatic validation, and clear API contracts between frontend and backend.

**Schema Structure**:
- ChatRequest: Contains user message and optional conversation context
- ChatResponse: Contains generated response and optional metadata

## Decision: Error Handling Strategy
**Rationale**: Robust error handling is essential for production systems to maintain reliability and provide good user experience during failures.

**Approach**:
- Handle Qdrant connection failures gracefully
- Manage Gemini API rate limits and errors
- Provide meaningful error messages to the frontend

## Technology Best Practices

### FastAPI
- Use dependency injection for configuration and services
- Implement proper response models with Pydantic
- Use middleware for logging and error handling

### OpenAI Agents SDK
- Define clear agent instructions for the QA use case
- Implement tools with proper type hints and documentation
- Use async runners for non-blocking execution

### Qdrant Integration
- Leverage existing vector search capabilities
- Handle different embedding dimensions consistently
- Implement proper error handling for database operations

### Async Programming
- Use async/await consistently throughout the stack
- Implement proper connection pooling for database access
- Handle timeouts appropriately for external API calls