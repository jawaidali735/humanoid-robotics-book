# Backend Deployment

This document provides instructions for deploying the backend service using Docker.

## Prerequisites

- Docker installed on your system
- Docker Compose installed
- API keys for your chosen AI provider (OpenAI, GEMINI, GROQ, or GROK)

## Environment Variables

Create a `.env` file in the root directory with the following variables:

```bash
# AI API Keys (at least one is required)
OPENAI_API_KEY=your_openai_api_key
GEMINI_API_KEY=your_gemini_api_key
GROQ_API_KEY=your_groq_api_key
GROK_API_KEY=your_grok_api_key

# Qdrant Configuration (for vector database)
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=your_collection_name

# Cohere Configuration (for embeddings)
COHERE_API_KEY=your_cohere_api_key

# Database Configuration (if applicable)
DATABASE_URL=your_database_url

# Application Settings
PORT=8007
```

## Docker Deployment

### 1. Build and Run with Docker Compose (Development)

```bash
# From the project root directory
docker-compose up --build
```

### 2. Build and Run with Docker Compose (Production)

```bash
# From the project root directory
docker-compose -f docker-compose.prod.yml up --build -d
```

### 3. Build and Run Backend Only

```bash
# Navigate to backend directory
cd backend

# Build the Docker image
docker build -t humanoid-robotics-backend .

# Run the container
docker run -d -p 8007:8007 --env-file ../.env humanoid-robotics-backend
```

## Deploying to Cloud Platforms

### Heroku

1. Install Heroku CLI
2. Create a Heroku app: `heroku create your-app-name`
3. Set config vars: `heroku config:set OPENAI_API_KEY=your_key`
4. Deploy: `git push heroku main`

### Render

1. Create a new Web Service on Render
2. Connect your GitHub repository
3. Set the build command: `cd backend && pip install -r requirements.txt`
4. Set the start command: `uvicorn api_integration.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables in Render dashboard

### AWS Elastic Beanstalk, Google Cloud Run, etc.

Use the provided Dockerfile to build and deploy to your preferred cloud platform.

## API Endpoints

After deployment, your API will be available at:
- `http://your-server-ip:8007` (for direct deployment)
- Health check: `http://your-server-ip:8007/api/chat/health`

## Frontend Configuration

When deploying the frontend, set the environment variable:
```bash
REACT_APP_CHATBOT_API_ENDPOINT=http://your-backend-url:8007
```

## Troubleshooting

1. **Port already in use**: Change the port mapping in docker-compose.yml
2. **API keys not working**: Verify environment variables are set correctly
3. **Qdrant connection issues**: Check QDRANT_URL and QDRANT_API_KEY
4. **Health check failing**: Ensure all required environment variables are set