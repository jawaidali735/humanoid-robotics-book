# Data Model: Backend–Frontend Integration via FastAPI

**Feature**: 1-fastapi-integration
**Model Version**: 1.0
**Created**: 2025-12-18

## Entity Definitions

### ChatQuery
Represents a user's chat query submitted to the AI agent.

**Fields**:
- `id` (string, UUID) - Unique identifier for the query
- `content` (string, 1-1000 characters) - The user's query text
- `timestamp` (datetime) - When the query was submitted
- `session_id` (string, nullable) - Optional session identifier for conversation tracking
- `user_context` (object, nullable) - Additional context from the user session

**Validation Rules**:
- `content` must be 1-1000 characters
- `timestamp` must be in ISO 8601 format
- `session_id` must be a valid UUID if provided

**Relationships**:
- One-to-many with APIResponse (one query generates one response)

### SelectedTextQuery
Represents a query with selected text context from the frontend.

**Fields**:
- `id` (string, UUID) - Unique identifier for the query
- `selected_text` (string, 1-5000 characters) - The text selected by the user
- `query` (string, 1-1000 characters) - The user's question about the selected text
- `document_context` (object, nullable) - Context about where text was selected
- `timestamp` (datetime) - When the query was submitted

**Validation Rules**:
- `selected_text` must be 1-5000 characters
- `query` must be 1-1000 characters
- `timestamp` must be in ISO 8601 format

**Relationships**:
- One-to-many with APIResponse (one query generates one response)

### APIResponse
Represents the response from the AI agent backend.

**Fields**:
- `id` (string, UUID) - Unique identifier for the response
- `content` (string) - The AI-generated response text
- `sources` (array of SourceAttribution) - References to book content used
- `confidence_level` (number, 0-1) - Confidence score for the response
- `timestamp` (datetime) - When the response was generated
- `error` (object, nullable) - Error information if processing failed

**Validation Rules**:
- `confidence_level` must be between 0 and 1
- `timestamp` must be in ISO 8601 format
- `sources` must be an array of SourceAttribution objects

**Relationships**:
- Many-to-one with ChatQuery or SelectedTextQuery (response to one query)

### SourceAttribution
Represents attribution to specific book content used in the response.

**Fields**:
- `content_id` (string) - ID from Qdrant for the content chunk
- `reference` (string) - Human-readable reference (e.g., "Chapter 1, Section 1.1")
- `text` (string) - Excerpt from the source content
- `similarity_score` (number, 0-1) - Relevance score of the source

**Validation Rules**:
- `similarity_score` must be between 0 and 1
- All fields are required

**Relationships**:
- Belongs to one APIResponse

### SessionContext
Represents user session information for conversation tracking.

**Fields**:
- `session_id` (string, UUID) - Unique identifier for the session
- `user_id` (string, nullable) - Optional user identifier
- `created_at` (datetime) - When the session was created
- `last_activity` (datetime) - When the session was last used
- `conversation_history` (array of objects) - Previous conversation turns

**Validation Rules**:
- `session_id` must be a valid UUID
- `created_at` and `last_activity` must be in ISO 8601 format

**Relationships**:
- One-to-many with ChatQuery (one session can have multiple queries)

### ErrorInfo
Represents structured error information for failed requests.

**Fields**:
- `code` (string) - Error code identifier
- `message` (string) - Human-readable error message
- `details` (object, nullable) - Additional error details
- `timestamp` (datetime) - When the error occurred

**Validation Rules**:
- `code` and `message` are required
- `timestamp` must be in ISO 8601 format

**Relationships**:
- Belongs to one APIResponse (when error occurs)

## State Transitions

### ChatQuery States
1. **PENDING** - Query received, waiting for processing
2. **PROCESSING** - AI agent is processing the query
3. **COMPLETED** - Response generated successfully
4. **FAILED** - Error occurred during processing

### APIResponse States
1. **GENERATING** - Response being created by AI agent
2. **FORMATTING** - Response being formatted for frontend
3. **READY** - Response ready for frontend consumption
4. **ERROR** - Error occurred during generation

## Data Validation Requirements

### Input Validation
- Query content: 1-1000 characters
- Selected text: 1-5000 characters
- Confidence scores: 0-1 range
- Timestamps: ISO 8601 format
- UUIDs: Valid UUID format

### Data Integrity
- All required fields must be present
- Foreign key relationships must be valid
- Timestamps must be consistent
- Confidence scores must be normalized

### Security Validation
- Input sanitization to prevent injection
- Size limits to prevent resource exhaustion
- Format validation to prevent processing errors
- Content filtering for sensitive information

## API Schema Definitions

### Request Schemas
```json
{
  "ChatQueryRequest": {
    "type": "object",
    "properties": {
      "query": {"type": "string", "minLength": 1, "maxLength": 1000},
      "session_id": {"type": ["string", "null"], "format": "uuid"}
    },
    "required": ["query"]
  },
  "SelectedTextRequest": {
    "type": "object",
    "properties": {
      "selected_text": {"type": "string", "minLength": 1, "maxLength": 5000},
      "query": {"type": "string", "minLength": 1, "maxLength": 1000},
      "document_context": {"type": "object"}
    },
    "required": ["selected_text", "query"]
  }
}
```

### Response Schemas
```json
{
  "APIResponse": {
    "type": "object",
    "properties": {
      "success": {"type": "boolean"},
      "data": {
        "type": "object",
        "properties": {
          "response_id": {"type": "string", "format": "uuid"},
          "answer": {"type": "string"},
          "sources": {
            "type": "array",
            "items": {
              "type": "object",
              "properties": {
                "content_id": {"type": "string"},
                "reference": {"type": "string"},
                "text": {"type": "string"},
                "similarity_score": {"type": "number", "minimum": 0, "maximum": 1}
              },
              "required": ["content_id", "reference", "text", "similarity_score"]
            }
          },
          "confidence_level": {"type": "number", "minimum": 0, "maximum": 1},
          "timestamp": {"type": "string", "format": "date-time"}
        },
        "required": ["response_id", "answer", "sources", "confidence_level", "timestamp"]
      },
      "error": {
        "type": ["object", "null"],
        "properties": {
          "code": {"type": "string"},
          "message": {"type": "string"},
          "details": {"type": "object"}
        }
      }
    },
    "required": ["success", "data"]
  }
}
```