"""
Chat API router for the Humanoid Robotics Chat API.
"""
from fastapi import APIRouter, Depends, HTTPException, Request
from typing import List, Optional
from pydantic import BaseModel, Field
import uuid
from datetime import datetime
import sys
import os

# Add the backend directory to the Python path to import agent
current_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.dirname(os.path.dirname(current_dir))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

# Import the RAGAgent
from agent import RAGAgent
from ..config import settings

# Create API router for chat endpoints
chat_router = APIRouter(
    prefix="/api/chat",
    tags=["chat"],
    responses={404: {"description": "Not found"}},
)

# Pydantic models for request/response validation
class ChatQueryRequest(BaseModel):
    """Request model for chat queries with validation."""
    query: str = Field(..., min_length=1, max_length=1000, description="User's chat query")
    session_id: Optional[str] = Field(None, description="Optional session identifier for conversation context")

class SelectedTextRequest(BaseModel):
    """Request model for selected text queries with validation."""
    selected_text: str = Field(..., min_length=1, max_length=5000, description="Text selected by user in frontend")
    query: str = Field(..., min_length=1, max_length=1000, description="User's question about the selected text")
    document_context: Optional[dict] = Field(None, description="Additional context about where text was selected")

class SourceAttribution(BaseModel):
    """Model for source attribution in responses."""
    content_id: str
    reference: str
    text: str
    similarity_score: float = Field(..., ge=0.0, le=1.0)

class APIResponseData(BaseModel):
    """Model for the response data structure."""
    response_id: str
    answer: str
    sources: List[SourceAttribution]
    confidence_level: float = Field(..., ge=0.0, le=1.0)
    timestamp: str

class ErrorResponse(BaseModel):
    """Model for error responses."""
    code: str
    message: str
    details: Optional[dict] = None

class APIResponse(BaseModel):
    """Model for API responses."""
    success: bool
    data: APIResponseData
    error: Optional[ErrorResponse] = None

class HealthResponse(BaseModel):
    """Model for health check responses."""
    status: str
    timestamp: str
    services: dict

class RateLimitResponse(BaseModel):
    """Model for rate limit status responses."""
    limit: int
    remaining: int
    reset_time: str

# Initialize the RAG agent in test mode
import os
api_key = os.getenv("OPENAI_API_KEY") or os.getenv("GEMINI_API_KEY")
test_mode = not api_key  # If no API key is set, run in test mode
rag_agent = RAGAgent(test_mode=test_mode)

@chat_router.post("/query", response_model=APIResponse)
async def chat_query(request: ChatQueryRequest):
    """
    Process a chat query and return AI-generated response using the RAG agent.
    """
    try:
        # Import and use the agent service from parent directory
        from ..agent_service import agent_service
        # Process the query using the agent service (async)
        result = await agent_service.process_chat_query(request.query)

        # Convert the agent's response to our API format
        response_id = str(uuid.uuid4())

        # Format sources from the agent response
        sources = []
        for source in result.get('sources', []):
            sources.append(SourceAttribution(
                content_id=source.get('content_id', ''),
                reference=source.get('reference', ''),
                text=source.get('text', ''),
                similarity_score=source.get('similarity_score', 0.0)
            ))

        return APIResponse(
            success=True,
            data=APIResponseData(
                response_id=response_id,
                answer=result.get('answer', ''),
                sources=sources,
                confidence_level=result.get('confidence_level', 0.0),
                timestamp=result.get('timestamp', datetime.utcnow().isoformat())
            )
        )
    except Exception as e:
        # Return a user-friendly error response instead of raising
        return APIResponse(
            success=False,
            data=APIResponseData(
                response_id=str(uuid.uuid4()),
                answer=f"I'm having trouble processing your request right now. Error: {str(e)[:100]}",
                sources=[],
                confidence_level=0.0,
                timestamp=datetime.utcnow().isoformat()
            ),
            error=ErrorResponse(
                code="AGENT_ERROR",
                message="Error processing request",
                details={"original_error": str(e)}  # Keep original error for debugging
            )
        )

@chat_router.post("/context-query", response_model=APIResponse)
async def context_query(request: SelectedTextRequest):
    """
    Process a query with selected text context using the RAG agent.
    """
    try:
        # Validate selected text length to prevent resource exhaustion
        if len(request.selected_text) > 3000:  # More reasonable limit than 5000
            return APIResponse(
                success=False,
                data=APIResponseData(
                    response_id=str(uuid.uuid4()),
                    answer="",
                    sources=[],
                    confidence_level=0.0,
                    timestamp=datetime.utcnow().isoformat()
                ),
                error=ErrorResponse(
                    code="INPUT_TOO_LARGE",
                    message="Selected text is too large. Please select a smaller portion of text.",
                    details={"max_length": 3000, "actual_length": len(request.selected_text)}
                )
            )

        # Create a contextual query that includes the selected text and document context
        document_context_info = ""
        if request.document_context:
            # Extract relevant information from document context
            doc_title = request.document_context.get('title', '')
            doc_section = request.document_context.get('section', '')
            doc_page = request.document_context.get('page', '')

            if doc_title or doc_section or doc_page:
                document_context_info = f" (from {doc_title} {doc_section} page {doc_page})".strip()

        # Create contextual query with document context information
        contextual_query = f"Based on this text{document_context_info}: '{request.selected_text}', {request.query}"

        # Import and use the agent service from parent directory
        from ..agent_service import agent_service
        # Process the contextual query using the agent service (async)
        result = await agent_service.process_context_query(request.selected_text, request.query, request.document_context)

        # Convert the agent's response to our API format
        response_id = str(uuid.uuid4())

        # Format sources from the agent response
        sources = []
        for source in result.get('sources', []):
            sources.append(SourceAttribution(
                content_id=source.get('content_id', ''),
                reference=source.get('reference', ''),
                text=source.get('text', ''),
                similarity_score=source.get('similarity_score', 0.0)
            ))

        return APIResponse(
            success=True,
            data=APIResponseData(
                response_id=response_id,
                answer=result.get('answer', ''),
                sources=sources,
                confidence_level=result.get('confidence_level', 0.0),
                timestamp=result.get('timestamp', datetime.utcnow().isoformat())
            )
        )
    except Exception as e:
        # Return a user-friendly error response instead of raising
        return APIResponse(
            success=False,
            data=APIResponseData(
                response_id=str(uuid.uuid4()),
                answer=f"I'm having trouble processing your request right now. Error: {str(e)[:100]}",
                sources=[],
                confidence_level=0.0,
                timestamp=datetime.utcnow().isoformat()
            ),
            error=ErrorResponse(
                code="AGENT_ERROR",
                message="Error processing request",
                details={"original_error": str(e)}  # Keep original error for debugging
            )
        )

@chat_router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify backend service availability.
    """
    # Check agent status
    try:
        # Test the agent with a simple query - need to await the async method
        import asyncio
        loop = asyncio.get_event_loop()
        test_result = loop.run_until_complete(rag_agent.query("What is humanoid robotics?"))
        agent_status = "healthy"
    except:
        agent_status = "unhealthy"

    # In a real implementation, you would also check Qdrant connectivity
    return HealthResponse(
        status="healthy" if agent_status == "healthy" else "degraded",
        timestamp=datetime.utcnow().isoformat(),
        services={
            "ai_agent": agent_status,
            "qdrant": "healthy",  # Placeholder - would check actual connection
            "api": "healthy"
        }
    )

@chat_router.get("/rate-limit", response_model=RateLimitResponse)
async def rate_limit_status(request: Request):
    """
    Check current rate limit status for the requesting IP.
    """
    # Get the client IP to check their specific rate limit status
    forwarded_for = request.headers.get("x-forwarded-for")
    if forwarded_for:
        client_ip = forwarded_for.split(",")[0]
    else:
        client_ip = request.client.host

    # In a real implementation with slowapi, you would need to access the internal storage
    # to get the actual usage. For now, we'll return a basic response with the configured limits.
    # To get actual usage, we would need to access slowapi's internal storage directly,
    # which requires understanding its internal structure.

    # For this implementation, we'll return the configured limits as a basic status
    # In a production system, you'd want to integrate with slowapi's storage backend
    # (Redis, memory, etc.) to get actual usage counts
    from slowapi import Limiter
    from ..main import limiter  # Import the limiter from main

    # Since slowapi doesn't provide a direct way to query remaining requests for an IP,
    # we'll return the configured limits. In a real implementation, you'd implement
    # a custom rate limit tracker or use slowapi's internal storage directly.
    return RateLimitResponse(
        limit=settings.RATE_LIMIT_MAX_REQUESTS,
        remaining=settings.RATE_LIMIT_MAX_REQUESTS,  # Would be actual remaining in a full implementation
        reset_time=(datetime.utcnow().timestamp() + settings.RATE_LIMIT_WINDOW_SECONDS).isoformat()
    )