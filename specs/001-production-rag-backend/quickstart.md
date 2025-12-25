# Quickstart: Production RAG Backend (FastAPI + OpenAI Agents SDK + Qdrant)

## Overview
This guide provides a quick setup and run guide for the RAG chatbot backend.

## Prerequisites
- Python 3.11+
- pip package manager
- Access to Qdrant vector database
- Gemini API key
- Existing knowledge base already embedded in Qdrant

## Setup Instructions

### 1. Clone and Navigate to Backend Directory
```bash
cd backend
```

### 2. Install Dependencies
```bash
pip install fastapi uvicorn openai-agents pydantic qdrant-client python-dotenv cohere
```

### 3. Set Up Environment Variables
Create a `.env` file in the backend directory:
```env
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_text_embedding
COHERE_API_KEY=your_cohere_api_key
```

### 4. Verify Existing Data
Ensure the `data_retrieval` directory contains the retrieval functions that connect to your Qdrant instance with the humanoid robotics knowledge base.

## Running the Application

### 1. Start the Server
```bash
cd backend
uvicorn app.main:app --reload --port 8000
```

### 2. Test the API
```bash
curl -X POST "http://localhost:8000/api/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is ROS2 in humanoid robotics?"
  }'
```

## Architecture Overview

### Components
1. **FastAPI Application** (`app/main.py`): Main entry point and API router
2. **Chat Endpoint** (`app/api/chat.py`): Handles incoming chat requests
3. **QA Agent** (`app/agents/qa_agent.py`): AI agent that processes queries
4. **Retrieval Tool** (`app/agents/tools.py`): Tool that fetches relevant content from Qdrant
5. **Schemas** (`app/schemas/chat.py`): Request/response data models
6. **Configuration** (`app/config/setting.py`): Environment configuration

### Flow
1. User sends a query to the `/api/chat` endpoint
2. FastAPI validates the request using Pydantic schemas
3. The QA agent is instantiated with Gemini model and retrieval tool
4. Agent processes the query, potentially calling the retrieval tool
5. Retrieved content is used to generate a grounded response
6. Response is formatted and returned to the user

## Key Features
- Asynchronous processing for high concurrency
- Integration with existing data retrieval functions
- Proper error handling and logging
- Type-safe API with Pydantic models
- Configurable through environment variables

## Troubleshooting
- If Qdrant connection fails, verify the URL and API key in environment variables
- If Gemini API returns errors, check the API key and quota limits
- If responses seem irrelevant, verify that the knowledge base is properly embedded in Qdrant