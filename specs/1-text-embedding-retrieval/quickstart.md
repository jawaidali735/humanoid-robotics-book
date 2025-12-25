# Quickstart: Text Embedding & Retrieval System

## Prerequisites

- Python 3.9 or higher
- Cohere API key (sign up at https://cohere.ai)
- Qdrant instance (local Docker or cloud)
- Git for version control

## Setup Instructions

### 1. Clone and Navigate to Backend Directory

```bash
# Create the backend directory structure
mkdir -p backend/{data_embedding,data_retrieval,config}
```

### 2. Install Dependencies

```bash
# Create requirements.txt
cat > backend/requirements.txt << 'EOF'
cohere==4.0.5
qdrant-client==1.7.0
requests==2.31.0
beautifulsoup4==4.12.2
python-dotenv==1.0.0
pytest==7.4.3
EOF

# Install dependencies
cd backend
pip install -r requirements.txt
```

### 3. Set Up Configuration

Create the configuration file:

```bash
# Create settings file
cat > backend/config/settings.py << 'EOF'
import os
from dotenv import load_dotenv

load_dotenv()

# Cohere Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY", "")
COHERE_MODEL = os.getenv("COHERE_MODEL", "embed-english-v3.0")
COHERE_INPUT_TYPE = os.getenv("COHERE_INPUT_TYPE", "search_document")

# Qdrant Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_book")

# Text Processing Configuration
CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "1000"))
CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", "200"))

# Validation
if not COHERE_API_KEY:
    raise ValueError("COHERE_API_KEY environment variable is required")
EOF
```

### 4. Environment Variables

Create a `.env` file in the backend directory:

```bash
cat > backend/.env << 'EOF'
# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here
COHERE_MODEL=embed-english-v3.0
COHERE_INPUT_TYPE=search_document

# Qdrant Configuration
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=
QDRANT_COLLECTION_NAME=humanoid_robotics_book

# Text Processing Configuration
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
EOF
```

### 5. Start Qdrant (if using local instance)

```bash
# Using Docker
docker run -d --name qdrant-container -p 6333:6333 qdrant/qdrant

# Or if you have qdrant installed locally
# qdrant
```

### 6. Verify Setup

Create a simple test to verify all components work:

```bash
cat > backend/test_setup.py << 'EOF'
from config.settings import COHERE_API_KEY, QDRANT_URL
import cohere
from qdrant_client import QdrantClient

def test_connections():
    print("Testing Cohere connection...")
    try:
        co = cohere.Client(COHERE_API_KEY)
        response = co.embed(
            texts=["test"],
            model="embed-english-v3.0"
        )
        print(f"✓ Cohere connection successful, embedding dimensions: {len(response.embeddings[0])}")
    except Exception as e:
        print(f"✗ Cohere connection failed: {e}")
        return False

    print("Testing Qdrant connection...")
    try:
        client = QdrantClient(url=QDRANT_URL)
        collections = client.get_collections()
        print(f"✓ Qdrant connection successful, collections: {[c.name for c in collections.collections]}")
    except Exception as e:
        print(f"✗ Qdrant connection failed: {e}")
        return False

    return True

if __name__ == "__main__":
    success = test_connections()
    if success:
        print("\n✓ All connections verified successfully!")
    else:
        print("\n✗ Some connections failed. Please check your configuration.")
EOF

# Run the test
python backend/test_setup.py
```

## Next Steps

1. Implement the modules as specified in the architecture:
   - `backend/data_embedding/url_extractor.py`
   - `backend/data_embedding/text_chunker.py`
   - `backend/data_embedding/embedder.py`
   - `backend/data_embedding/qdrant_client.py`
   - `backend/data_embedding/qdrant_collection.py`
   - `backend/data_embedding/qdrant_store.py`
   - `backend/data_embedding/ingestion.py`
   - `backend/data_retrieval/get_query_embedder.py`
   - `backend/data_retrieval/retriever.py`

2. Test the ingestion pipeline with sample URLs

3. Test the retrieval functionality with sample queries

## Troubleshooting

- **Cohere API Error**: Verify your API key in the `.env` file
- **Qdrant Connection Error**: Ensure Qdrant is running and URL is correct
- **Module Import Errors**: Ensure all dependencies are installed via `pip install -r requirements.txt`