# API Contracts: Vector Retrieval and Semantic Search Validation Pipeline

## Query Service

### Endpoint: POST /query
Query Qdrant Cloud collection using vector similarity search.

#### Request
```json
{
  "query_text": "search query text",
  "collection_name": "book_text_embedding",
  "limit": 10,
  "similarity_threshold": 0.75
}
```

#### Response
```json
{
  "query_id": "uuid",
  "results": [
    {
      "chunk_id": "uuid",
      "content": "retrieved content text",
      "similarity_score": 0.85,
      "source_url": "original source URL",
      "source_title": "original document title",
      "chunk_index": 1,
      "token_count": 150
    }
  ],
  "execution_time": 1.2
}
```

#### Error Response
```json
{
  "error": "error message",
  "code": "error code"
}
```

## Validation Service

### Endpoint: POST /validate
Validate retrieval quality and accuracy against expected results.

#### Request
```json
{
  "query_id": "uuid",
  "query_text": "search query text",
  "expected_results": ["chunk_id1", "chunk_id2", "..."],
  "retrieved_chunks": [
    {
      "chunk_id": "uuid",
      "content": "retrieved content text",
      "similarity_score": 0.85
    }
  ]
}
```

#### Response
```json
{
  "validation_result": {
    "id": "uuid",
    "query_id": "uuid",
    "precision": 0.8,
    "recall": 0.75,
    "relevance_score": 0.82,
    "execution_time": 1.2,
    "status": "pass",
    "thresholds": {
      "precision": 0.85,
      "recall": 0.80
    }
  }
}
```

## Report Service

### Endpoint: GET /report
Generate comprehensive validation report.

#### Response
```json
{
  "validation_report": {
    "id": "uuid",
    "start_time": "2025-12-17T10:00:00Z",
    "end_time": "2025-12-17T10:02:30Z",
    "total_queries": 100,
    "pass_count": 95,
    "pass_rate": 0.95,
    "avg_precision": 0.85,
    "avg_recall": 0.82,
    "avg_relevance": 0.81,
    "execution_time": 150.0,
    "status": "pass",
    "details": [
      {
        "query_id": "uuid",
        "precision": 0.8,
        "recall": 0.75,
        "status": "pass"
      }
    ]
  }
}
```