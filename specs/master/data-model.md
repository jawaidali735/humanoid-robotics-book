# Data Model: Vector Retrieval and Semantic Search Validation Pipeline

## Overview

This document describes the data models used in the vector retrieval and semantic search validation pipeline. These models represent the core entities that flow through the system during validation operations.

## Core Entities

### Query Vector
Represents a search query that has been converted to vector form for similarity search.

- **id**: String (UUID) - Unique identifier for the query
- **content**: String - Original text of the search query
- **embedding**: Array of Float - Vector representation of the query text (768-dimensional for Cohere)
- **expected_results**: Array of String - List of content IDs that should be considered relevant
- **created_at**: DateTime - Timestamp when the query was created
- **query_type**: String - Classification of query type (factual, conceptual, procedural, etc.)

### Retrieved Chunk
Represents a content chunk retrieved from Qdrant Cloud that matches a query.

- **id**: String (UUID) - Unique identifier for the retrieved chunk
- **content**: String - Text content of the retrieved chunk
- **similarity_score**: Float - Vector similarity score between query and this chunk (0.0 to 1.0)
- **source_url**: String - Original URL where this content was extracted from
- **source_title**: String - Title of the original document
- **chunk_index**: Integer - Position of this chunk within the original document
- **token_count**: Integer - Number of tokens in the chunk
- **retrieved_at**: DateTime - Timestamp when chunk was retrieved
- **collection**: String - Name of the Qdrant collection where chunk is stored

### Validation Result
Captures the results of validating a single query against expected outcomes.

- **id**: String (UUID) - Unique identifier for the validation result
- **query_id**: String - Reference to the source query
- **retrieved_chunks**: Array of RetrievedChunk - List of chunks returned for the query
- **precision**: Float - Precision score for this query's results (0.0 to 1.0)
- **recall**: Float - Recall score for this query's results (0.0 to 1.0)
- **relevance_score**: Float - Overall relevance assessment (0.0 to 1.0)
- **execution_time**: Float - Time taken to execute validation in seconds
- **status**: String - Validation status ("pass", "fail", "partial")
- **evaluated_at**: DateTime - Timestamp when validation was performed
- **thresholds**: Object - Validation thresholds used for this assessment

### Validation Report
Aggregated results from running the complete validation pipeline.

- **id**: String (UUID) - Unique identifier for the validation report
- **start_time**: DateTime - When validation pipeline started
- **end_time**: DateTime - When validation pipeline completed
- **total_queries**: Integer - Number of queries validated
- **pass_count**: Integer - Number of queries that passed validation
- **pass_rate**: Float - Overall pass rate (0.0 to 1.0)
- **avg_precision**: Float - Average precision across all queries
- **avg_recall**: Float - Average recall across all queries
- **avg_relevance**: Float - Average relevance score across all queries
- **execution_time**: Float - Total time for validation pipeline in seconds
- **details**: Array of ValidationResult - Individual results for each query
- **status**: String - Overall validation status ("pass", "fail", "partial")

### Test Dataset
Curated collection of queries and expected results for validation.

- **id**: String (UUID) - Unique identifier for the test dataset
- **name**: String - Name/description of the test dataset
- **description**: String - Detailed description of the dataset's purpose
- **queries**: Array of QueryVector - List of queries with expected results
- **coverage**: Object - Coverage metrics for different topic areas
- **created_at**: DateTime - When dataset was created
- **author**: String - Creator of the test dataset

## Relationships

- A single `Query Vector` can result in multiple `Retrieved Chunk` objects during a search
- Each `Validation Result` connects one `Query Vector` with its corresponding `Retrieved Chunk` results
- Multiple `Validation Result` objects are aggregated into a single `Validation Report`
- A `Test Dataset` contains multiple `Query Vector` objects for comprehensive validation

## Data Flow

1. `Test Dataset` provides `Query Vector` objects to the validation pipeline
2. Each `Query Vector` is used to search in Qdrant Cloud, returning multiple `Retrieved Chunk` objects
3. `Validation Result` objects are created by comparing `Retrieved Chunk` results with expected outcomes
4. `Validation Report` aggregates all `Validation Result` objects into a comprehensive assessment

## Storage Considerations

- Query vectors and validation results should be stored temporarily for analysis
- Validation reports should be persisted for historical tracking
- Large test datasets may need to be loaded incrementally to manage memory usage
- Qdrant Cloud collection schema should match the expected content structure