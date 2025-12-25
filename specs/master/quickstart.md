# Quickstart Guide: Vector Retrieval and Semantic Search Validation Pipeline

## Getting Started

This guide provides step-by-step instructions to set up and run the vector retrieval and semantic search validation pipeline.

### Prerequisites

- Python 3.9 or higher
- UV package manager
- Access to Qdrant Cloud with 'book_text_embedding' collection
- Cohere API key for embedding generation
- Git for version control

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. Navigate to the validation feature directory:
   ```bash
   cd specs/master
   ```

3. Install dependencies using UV:
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\\Scripts\\activate
   uv pip install -r requirements.txt
   ```

### Configuration

1. Create a `.env` file in the project root with the following variables:
   ```
   QDRANT_HOST=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   COHERE_API_KEY=your_cohere_api_key
   VALIDATION_THRESHOLD_PRECISION=0.85
   VALIDATION_THRESHOLD_RECALL=0.80
   ```

2. Ensure your Qdrant Cloud collection 'book_text_embedding' exists and contains embedded content.

### Running Validation

1. Execute the complete validation pipeline:
   ```bash
   python validation.py
   ```

2. Or run specific validation components:
   ```bash
   # Run only query validation
   python validation.py --component query

   # Run only quality assessment
   python validation.py --component quality

   # Run with custom test dataset
   python validation.py --dataset path/to/test_dataset.json
   ```

### Output

The validation pipeline will generate:
- A comprehensive validation report in `validation_report.json`
- Detailed logs in `validation.log`
- Quality metrics summary in `metrics_summary.txt`

## Common Tasks

### Running a Single Query Test
```bash
python validation.py --query "What is humanoid robotics?"
```

### Validating Specific Metrics
```bash
python validation.py --metric precision --limit 10
```

### Generating Test Reports
```bash
python validation.py --report full
```

## Troubleshooting

### Connection Issues
- Verify Qdrant Cloud credentials in `.env`
- Check network connectivity to Qdrant Cloud
- Ensure API keys have appropriate permissions

### Embedding Generation Failures
- Verify Cohere API key in `.env`
- Check API rate limits
- Ensure query text is within embedding model limits

### Validation Threshold Failures
- Review test dataset for quality
- Adjust validation thresholds in configuration if needed
- Verify that embedded content matches expected topics