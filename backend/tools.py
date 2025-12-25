from agents import function_tool
from typing import List
import sys
import os

# Add the backend directory to sys.path to properly import data_retrieval
backend_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from data_retrieval.retrieval import retrieve


@function_tool
def retrieval_tool(query: str) -> str:
    """
    Retrieve relevant content from the humanoid robotics knowledge base using semantic search.

    Args:
        query: The user's question or search query

    Returns:
        String containing relevant text chunks from the knowledge base
    """
    # Call the existing retrieval function from data_retrieval
    results = retrieve(query)

    # Return the retrieved content as a string (join the list if needed)
    if isinstance(results, list):
        return " ".join(results) if results else "No relevant information found."
    else:
        return str(results) if results else "No relevant information found."