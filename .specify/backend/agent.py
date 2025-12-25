from agents import (
    Agent,
    Runner,
    AsyncOpenAI,
    OpenAIChatCompletionsModel,
    RunConfig,
    function_tool,
)
from dotenv import load_dotenv, find_dotenv
import os
from datetime import datetime
from typing import Dict, List, Any
from tools import retrieval_tool
load_dotenv(find_dotenv())



# -------------------------
# Model / Client Setup
# -------------------------
def build_agent():
    """
    Build Agent + RunConfig based on available API key
    """

    # ---- GROK API (Highest Priority) ----
    if os.getenv("GROK_API_KEY"):
        # For now, treat GROK API key similar to GROQ (OpenAI-compatible)
        # This may need to be adjusted based on the actual GROK API requirements
        client = AsyncOpenAI(
            api_key=os.getenv("GROK_API_KEY"),
            base_url="https://api.groq.com/openai/v1",  # May need adjustment
        )

        model = OpenAIChatCompletionsModel(
            model="llama-3.3-70b-versatile",  # Using a supported model for GROK/GROQ
            openai_client=client,
        )

        config = RunConfig(
            model=model,
            model_provider=client,
            tracing_disabled=True,
        )

    # ---- GROQ (OpenAI-compatible) ----
    elif os.getenv("GROQ_API_KEY"):
        client = AsyncOpenAI(
            api_key=os.getenv("GROQ_API_KEY"),
            base_url="https://api.groq.com/openai/v1",
        )

        model = OpenAIChatCompletionsModel(
            model="llama-3.3-70b-versatile",
            openai_client=client,
        )

        config = RunConfig(
            model=model,
            model_provider=client,
            tracing_disabled=True,
        )

    # ---- GEMINI API ----
    elif os.getenv("GEMINI_API_KEY"):
        # For GEMINI, we might need a different approach since it's not OpenAI compatible
        # For now, we'll set up the configuration for testing mode
        client = None
        config = None
        print("GEMINI API key detected but not yet implemented with proper client")

    # ---- OpenAI ----
    elif os.getenv("OPENAI_API_KEY"):
        client = AsyncOpenAI(
            api_key=os.getenv("OPENAI_API_KEY"),
        )

        model = OpenAIChatCompletionsModel(
            model="gpt-4o",
            openai_client=client,
        )

        config = RunConfig(
            model=model,
            model_provider=client,
            tracing_disabled=True,
        )

    # ---- No key ----
    else:
        client = None
        config = None

    agent = Agent(
        name="Humanoid Robotics Assistant",
        instructions=(
            "You are an expert humanoid robotics assistant.\n"
            "Whenever the user asks about humanoid robotics, ROS, "
            "Isaac Sim, physical AI, simulation, or book-related topics, "
            "try to use retrieval_tool to get relevant information.\n"
            "If retrieval_tool is successful, incorporate that information into your response.\n"
            "If retrieval_tool fails or returns no relevant information, respond using your general knowledge.\n"
            "Always provide helpful and accurate responses regardless of whether you can access the book content.\n"
            "When using book content, cite references from retrieved results."
        ),
        tools=[retrieval_tool],
    )

    return agent, config


AGENT, RUN_CONFIG = build_agent()


# -------------------------
# RAG Agent Wrapper
# -------------------------
class RAGAgent:
    def __init__(self, test_mode: bool = False):
        self.test_mode = test_mode
        self.agent = AGENT
        self.config = RUN_CONFIG

    async def query(self, user_query: str) -> Dict[str, Any]:
        if not user_query or not user_query.strip():
            raise ValueError("Query cannot be empty")

        if len(user_query) > 1000:
            raise ValueError("Query too long")

        # -------------------------
        # Test Mode
        # -------------------------
        if self.test_mode:
            # Use the same retrieval path as the main tools
            from tools import retrieval_tool
            results = retrieval_tool(user_query)

            return {
                "query": user_query,
                "answer": f"[TEST MODE] Retrieved {len(results)} chunks using the tools path.",
                "sources": [{"content": result, "source": "retrieval_tool"} for result in results],
                "confidence_level": 0.7,
                "timestamp": datetime.utcnow().isoformat(),
            }

        # -------------------------
        # Agent Run with tool failure handling
        # -------------------------
        try:
            if self.config:
                result = await Runner.run(
                    starting_agent=self.agent,
                    input=user_query,
                    run_config=self.config,
                )
            else:
                result = await Runner.run(
                    starting_agent=self.agent,
                    input=user_query,
                )

            return {
                "query": user_query,
                "answer": result.final_output,
                "sources": result.tool_outputs if hasattr(result, "tool_outputs") else [],
                "confidence_level": 1.0,
                "timestamp": datetime.utcnow().isoformat(),
            }

        except RuntimeError:
            # Fallback for running event loop
            if self.config:
                result = Runner.run_sync(
                    starting_agent=self.agent,
                    input=user_query,
                    run_config=self.config,
                )
            else:
                result = Runner.run_sync(
                    starting_agent=self.agent,
                    input=user_query,
                )

            return {
                "query": user_query,
                "answer": result.final_output,
                "sources": result.tool_outputs if hasattr(result, "tool_outputs") else [],
                "confidence_level": 1.0,
                "timestamp": datetime.utcnow().isoformat(),
            }

        except Exception as e:
            # Handle tool calling failures and other agent errors gracefully
            # Instead of failing completely, return a response using general knowledge
            error_msg = str(e)
            if "tool" in error_msg.lower() or "function" in error_msg.lower() or "Failed to call a function" in error_msg:
                # If it's a tool-related error, return a helpful response
                return {
                    "query": user_query,
                    "answer": f"I understand you're asking about '{user_query}'. I don't have access to the specific book content right now, but I can provide general information about this topic based on my knowledge.",
                    "sources": [],
                    "confidence_level": 0.7,
                    "timestamp": datetime.utcnow().isoformat(),
                }
            else:
                # For other types of errors, raise the exception
                raise e

    # -------------------------
    # Direct Retrieval (Optional) - Use data retrieval directly
    # -------------------------
    def retrieve_information(self, query_text: str, top_k: int = None) -> list:
        # Call data retrieval directly to maintain the proper flow
        from data_retrieval.retrieval import retrieve
        return retrieve(query_text)
