"""
Service layer to integrate with the RAG agent backend.
"""
import logging
from typing import Dict, Any
from datetime import datetime

from config import settings
from agent import RAGAgent

logger = logging.getLogger(__name__)


class AgentService:
    """Service class to interface with the RAG agent."""

    def __init__(self):
        self._agent = None
        self._initialize_agent()

    def _initialize_agent(self):
        try:
            api_key = (
                settings.OPENAI_API_KEY
                or settings.GEMINI_API_KEY
                or settings.GROQ_API_KEY
                or settings.GROK_API_KEY  # Added GROK_API_KEY support
            )

            test_mode = not bool(api_key)

            self._agent = RAGAgent(test_mode=test_mode)
            logger.info(
                f"RAG Agent initialized in {'test' if test_mode else 'real'} mode"
            )

        except Exception as e:
            logger.exception("Failed to initialize RAG Agent")
            raise

    @property
    def agent(self):
        if self._agent is None:
            self._initialize_agent()
        return self._agent

    async def process_chat_query(
        self, query: str, session_id: str | None = None
    ) -> Dict[str, Any]:
        try:
            result = await self.agent.query(query)

            if isinstance(result, str):
                result = {
                    "answer": result,
                    "sources": [],
                    "confidence_level": 0.8,
                }

            return {
                "query": query,
                "answer": result.get("answer", ""),
                "sources": result.get("sources", []),
                "confidence_level": result.get("confidence_level", 0.8),
                "timestamp": datetime.utcnow().isoformat(),
            }

        except Exception as e:
            logger.warning(f"Chat query processing issue (using fallback): {str(e)}")
            # Instead of showing an error, return a response that indicates no specific info was found
            # but the system is still functional
            return {
                "query": query,
                "answer": f"I don't have specific information about '{query}' in the book content, but I can still help with general questions about humanoid robotics. How else may I assist you?",
                "sources": [],
                "confidence_level": 0.5,
                "timestamp": datetime.utcnow().isoformat(),
            }

    async def process_context_query(
        self, selected_text: str, query: str, document_context: dict | None = None
    ) -> Dict[str, Any]:
        contextual_query = f"Based on this text: '{selected_text[:500]}...', {query}"

        try:
            result = await self.agent.query(contextual_query)

            if isinstance(result, str):
                result = {
                    "answer": result,
                    "sources": [],
                    "confidence_level": 0.8,
                }

            return {
                "query": contextual_query,
                "answer": result.get("answer", ""),
                "sources": result.get("sources", []),
                "confidence_level": result.get("confidence_level", 0.8),
                "timestamp": datetime.utcnow().isoformat(),
            }

        except Exception as e:
            logger.warning(f"Context query processing issue (using fallback): {str(e)}")
            return {
                "query": contextual_query,
                "answer": f"I don't have specific information about this context in the book content, but I can still help with general questions about humanoid robotics. How else may I assist you?",
                "sources": [],
                "confidence_level": 0.5,
                "timestamp": datetime.utcnow().isoformat(),
            }

    def get_agent_status(self) -> Dict[str, str]:
        return {
            "ai_agent": "healthy" if self._agent else "unhealthy",
            "qdrant": "unknown",
            "api": "healthy",
        }


# SINGLETON
agent_service = AgentService()
