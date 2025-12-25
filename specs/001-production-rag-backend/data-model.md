# Data Model: Production RAG Backend (FastAPI + OpenAI Agents SDK + Qdrant)

## Overview
This document defines the data models for the RAG chatbot backend system, including API request/response schemas and internal data structures.

## API Request/Response Models

### ChatRequest
**Purpose**: Represents a user's chat message sent to the backend

**Fields**:
- `message` (str): The user's question or message (required)
- `conversation_id` (Optional[str]): ID to maintain conversation context (optional)
- `history` (Optional[List[Dict]]): Previous conversation history (optional)

**Validation**:
- `message` must be 1-2000 characters
- `conversation_id` must be a valid UUID format if provided
- `history` items must have 'role' and 'content' fields

### ChatResponse
**Purpose**: Represents the AI agent's response to the user

**Fields**:
- `response` (str): The generated answer from the AI agent (required)
- `conversation_id` (str): ID for the conversation context (required)
- `sources` (List[str]): List of source documents used in the response (optional)
- `timestamp` (datetime): When the response was generated (required)

**Validation**:
- `response` must not be empty
- `sources` URLs must be valid if provided

## Internal Data Models

### Query
**Purpose**: Internal representation of a user query after processing

**Fields**:
- `text` (str): The original query text
- `embedding` (List[float]): Vector representation of the query
- `conversation_context` (Optional[str]): Context from previous interactions

### RetrievedContext
**Purpose**: Represents relevant content retrieved from the knowledge base

**Fields**:
- `content` (str): The text content retrieved from Qdrant
- `source_url` (str): URL where the content originated
- `similarity_score` (float): How closely the content matches the query (0.0-1.0)
- `metadata` (Dict): Additional information about the retrieved content

### AgentResponse
**Purpose**: Internal representation of the agent's response before formatting

**Fields**:
- `content` (str): The main response content
- `confidence` (float): Agent's confidence in the response (0.0-1.0)
- `sources_used` (List[RetrievedContext]): Content used to generate the response
- `tool_calls` (List[Dict]): Record of tools called during response generation

## Qdrant Schema
The system will use the existing Qdrant collection structure from the data_retrieval module:
- Collection name: `book_text_embedding`
- Vector size: 768 (from existing embeddings)
- Payload fields: `chunk_content`, `source_url`, `source_title`

## Validation Rules

### From Requirements
- FR-001: Request/response must be in JSON format
- FR-004: Retrieved content must come from Qdrant based on semantic similarity
- FR-005: Responses must be grounded in source content
- FR-009: Responses should include source citations when possible

### Additional Validation
- All string inputs should be sanitized to prevent injection attacks
- Query length should be limited to prevent abuse
- Response time should be tracked and logged for performance monitoring