# Quickstart Guide: Backend–Frontend Integration via FastAPI

**Feature**: 1-fastapi-integration
**Version**: 1.0
**Created**: 2025-12-18

## Overview

This guide provides step-by-step instructions to set up the FastAPI backend integration with the Docusaurus frontend for the humanoid robotics chatbot. Follow these steps to get the integrated system running locally.

## Prerequisites

Before starting, ensure you have:

- **Backend**: Python 3.9 or higher
- **Frontend**: Node.js 16 or higher, npm or yarn
- **Database**: Qdrant vector database running (local or remote)
- **API Keys**: Valid OpenAI (or Gemini) API key and Qdrant credentials
- **Tools**: Git, pip, virtual environment manager (optional but recommended)

## Backend Setup

### 1. Clone and Navigate to Backend Directory

```bash
cd backend
```

### 2. Set Up Python Environment (Recommended)

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Configure Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
# API Keys
GEMINI_API_KEY=your_gemini_api_key_here
# OR if using OpenAI:
# OPENAI_API_KEY=your_openai_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_content

# Application Configuration
DEBUG=false
LOG_LEVEL=INFO
TOP_K=5
SIMILARITY_THRESHOLD=0.5
```

### 4. Start the FastAPI Server

```bash
# From the backend directory
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`.

## Frontend Setup

### 1. Navigate to Frontend Directory

```bash
cd docs  # or wherever your Docusaurus project is located
```

### 2. Install Dependencies

```bash
npm install
# or
yarn install
```

### 3. Configure API Endpoint

Update your Docusaurus configuration to include the API endpoint. In `docusaurus.config.js`:

```javascript
// Add to your docusaurus.config.js
module.exports = {
  // ... other config
  themeConfig: {
    // ... other theme config
    apiEndpoint: process.env.API_ENDPOINT || 'http://localhost:8000'
  }
};
```

### 4. Create Chat Component

Create a chat component in `/src/components/ChatInterface/`:

```bash
mkdir -p src/components/ChatInterface
```

Create `src/components/ChatInterface/index.js` with the chat interface implementation that connects to the FastAPI endpoints.

## API Endpoints

### Chat Query
- **Endpoint**: `POST /api/chat/query`
- **Description**: Process a chat query and return AI-generated response
- **Example**:
```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is humanoid robotics?"}'
```

### Context Query (Selected Text)
- **Endpoint**: `POST /api/chat/context-query`
- **Description**: Process a query with selected text context
- **Example**:
```bash
curl -X POST http://localhost:8000/api/chat/context-query \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "Humanoid robotics is a branch of robotics...",
    "query": "Can you explain the key challenges?"
  }'
```

### Health Check
- **Endpoint**: `GET /api/health`
- **Description**: Check backend service availability
- **Example**:
```bash
curl http://localhost:8000/api/health
```

### Rate Limit Status
- **Endpoint**: `GET /api/rate-limit`
- **Description**: Check current rate limit status
- **Example**:
```bash
curl http://localhost:8000/api/rate-limit
```

## Frontend Integration

### 1. Add CORS Configuration

The backend is configured to allow requests from `http://localhost:3000` (common Docusaurus dev server) and `http://localhost:3001`. If your setup uses different ports, update the CORS configuration in the FastAPI app.

### 2. Implement Chat Component

Create a React component that:

1. Makes POST requests to `/api/chat/query` for regular queries
2. Makes POST requests to `/api/chat/context-query` for selected text queries
3. Displays responses with source attribution
4. Handles loading states and errors appropriately

### 3. State Management

The chat component should manage:

- Conversation history
- Loading states during API calls
- Error states for failed requests
- Session context for maintaining conversation flow

## Testing the Integration

### 1. Verify Backend Health

```bash
curl http://localhost:8000/api/health
```

Expected response: `{"status": "healthy", ...}`

### 2. Test Chat Endpoint

```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "Hello"}'
```

### 3. Test Frontend Connection

Start the Docusaurus development server:

```bash
npm run start
# or
yarn start
```

Visit `http://localhost:3000` and test the chat interface.

## Common Issues and Troubleshooting

### CORS Errors
- Ensure the backend allows requests from your frontend origin
- Check that the frontend is making requests to the correct backend URL

### API Key Issues
- Verify that API keys are correctly set in the `.env` file
- Check that the API provider (OpenAI/Gemini) is properly configured

### Qdrant Connection Issues
- Verify Qdrant URL and API key are correct
- Ensure Qdrant service is running and accessible
- Check that the specified collection exists

### Rate Limiting
- Default rate limits may be too restrictive for development
- Adjust rate limiting parameters in the backend configuration as needed

## Production Deployment

### Backend Deployment
- Use a production ASGI server like Gunicorn
- Set up proper logging and monitoring
- Configure SSL/TLS for secure connections
- Set up proper environment variables for production

### Frontend Deployment
- Build the Docusaurus site for production: `npm run build`
- Configure the production API endpoint
- Ensure CORS settings allow requests from your production domain

## Next Steps

1. Customize the chat interface UI to match your design requirements
2. Implement additional features like conversation history persistence
3. Add analytics to track usage and improve the chatbot
4. Set up monitoring and alerting for the API endpoints
5. Consider implementing caching for frequently asked questions