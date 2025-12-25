"""
Agent service module for handling chat queries with the RAG agent.
"""
import asyncio
from typing import Dict, Any
import os
from datetime import datetime


class AgentService:
    """Service class to handle agent interactions."""

    def __init__(self):
        """Initialize the agent service."""
        # Import the RAG agent
        from agent import RAGAgent

        # Check for API keys to determine test mode
        api_key = os.getenv("OPENAI_API_KEY") or os.getenv("GEMINI_API_KEY") or os.getenv("COHERE_API_KEY")
        test_mode = not api_key  # If no API key is set, run in test mode
        self.rag_agent = RAGAgent(test_mode=test_mode)

    async def process_chat_query(self, query: str) -> Dict[str, Any]:
        """Process a chat query using the RAG agent."""
        try:
            # Use the RAG agent to process the query
            result = await self.rag_agent.query(query)
            return result
        except Exception as e:
            # Return a fallback response if the agent fails
            return {
                "query": query,
                "answer": f"Test response: I received your query '{query}' but the agent is not fully configured. This is a test response.",
                "sources": [],
                "confidence_level": 0.5,
                "timestamp": datetime.utcnow().isoformat(),
            }

    async def process_context_query(self, selected_text: str, query: str, document_context: dict = None) -> Dict[str, Any]:
        """Process a contextual query using the RAG agent."""
        try:
            # Combine the selected text and query for contextual processing
            full_query = f"Based on: '{selected_text}', {query}"
            result = await self.rag_agent.query(full_query)
            return result
        except Exception as e:
            # Return a fallback response if the agent fails
            return {
                "query": query,
                "answer": f"Test response: I received your contextual query but the agent is not fully configured. This is a test response.",
                "sources": [],
                "confidence_level": 0.5,
                "timestamp": datetime.utcnow().isoformat(),
            }


# Create a global instance of the agent service
agent_service = AgentService()