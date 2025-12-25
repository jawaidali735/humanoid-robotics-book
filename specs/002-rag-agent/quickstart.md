# Quickstart Guide: Retrieval-Enabled Agent

## Prerequisites

- Python 3.11 or higher
- OpenAI API key
- Qdrant instance (cloud or self-hosted)
- Qdrant API key and connection details

## Setup

1. **Install dependencies**:
   ```bash
   pip install openai-agents-python qdrant-client python-dotenv
   ```

2. **Create environment file**:
   Create a `.env` file if not exists in your project root with the following:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_URL=your_qdrant_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_COLLECTION_NAME=your_collection_name_here
   ```

3. **Initialize the agent**:
   ```python
   from backend.agent import RAGAgent

   # Initialize the agent with configuration
   agent = RAGAgent()

   # Query the agent
   response = agent.query("Your question about humanoid robotics here")
   print(response)
   ```

## Key Components

### 1. Configuration (`backend/config.py`)
Handles environment variables and connection parameters for OpenAI and Qdrant.

### 2. Qdrant Client (`backend/qdrant_client.py`)
Manages connection to Qdrant and performs similarity searches to retrieve relevant content.

### 3. Main Agent (`backend/agent.py`)
The core implementation that combines OpenAI Agents SDK with custom retrieval tools.

### 4. Utilities (`backend/utils/`)
Helper functions for response formatting and input validation.

## Basic Usage

```python
# Import the agent
from backend.agent import RAGAgent

# Initialize
rag_agent = RAGAgent()

# Ask a question
question = "What are the key principles of humanoid robotics?"
response = rag_agent.query(question)

print(response.answer)
print(response.sources)  # Source attributions
```

## Testing

Run the tests to ensure everything works correctly:
```bash
pytest backend/tests/
```

## Development Workflow

1. Set up your Qdrant instance with embedded book content
2. Configure your API keys in the `.env` file
3. Initialize the RAG agent
4. Test with sample queries
5. Verify that responses include proper source attribution