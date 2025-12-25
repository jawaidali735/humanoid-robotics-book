# Quickstart Guide: RAG Textbook Chatbot Backend

## Prerequisites

- Python 3.11
- Docker
- Google Gemini API key
- Qdrant Cloud account (free tier)

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Set up environment variables**
   Create a `.env` file with the following variables:
   ```env
   GEMINI_API_KEY=your_google_gemini_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

## Running Locally

1. **Start the services**
   ```bash
   # Run directly with Python
   python -m uvicorn src.main:app --host 0.0.0.0 --port 7860

   # Or with Docker
   docker build -t rag-chatbot .
   docker run -p 7860:7860 rag-chatbot
   ```

2. **Run the ingestion pipeline**
   ```bash
   python src/services/ingestion.py
   ```

3. **Test the API**
   ```bash
   curl -X POST http://localhost:7860/chat \
     -H "Content-Type: application/json" \
     -d '{"question": "What are the key components of a humanoid robot??"}'
   ```

## API Endpoints

- `POST /chat` - Submit a question and get an answer with citations
- `GET /health` - Check service health status

## Configuration

The system can be configured via environment variables:

- `GEMINI_MODEL`: Gemini model to use (default: gemini-2.0-flash)
- `EMBEDDING_MODEL`: Embedding model to use (default: text-embedding-004)
- `QDRANT_COLLECTION`: Collection name for vector storage (default: textbook_content)
- `CHUNK_SIZE`: Size of text chunks for embedding (default: 1000 characters)

## Testing

Run the test suite:
```bash
pytest tests/
```

Run specific test types:
```bash
pytest tests/unit/      # Unit tests
pytest tests/integration/  # Integration tests
pytest tests/contract/  # Contract tests
```