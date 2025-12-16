# RAG Retrieval Pipeline Validation Tool

This tool is designed to validate the retrieval component of a Retrieval-Augmented Generation (RAG) pipeline. It utilizes Cohere for embeddings and Qdrant as a vector database to retrieve relevant text chunks based on a given query. The tool also exposes a FastAPI endpoint for programmatic access and includes performance testing capabilities.

## Features

-   **Retrieval Accuracy**: Core logic to query a vector database and return relevant chunks.
-   **Metadata Integrity**: Ensures associated metadata with retrieved chunks is accurate and complete.
-   **Performance Assessment**: FastAPI endpoint for load testing and performance measurement.
-   **API Key Authentication**: Secure access to the retrieval endpoint.
-   **Exponential Backoff**: Robust API calls with retry mechanisms.
-   **Comprehensive Logging**: Detailed logs for monitoring and debugging.

## Project Structure

-   `backend/src/retrieval/`: Contains the core retrieval logic, data models, services for Cohere and Qdrant, and configuration.
-   `backend/src/api/retrieval_api.py`: FastAPI application exposing the retrieval functionality.
-   `backend/requirements.txt`: Python dependencies.
-   `backend/tests/test_retrieval.py`: Unit and integration tests for retrieval accuracy and metadata integrity.
-   `tests/performance/test_retrieval_performance.py`: Load testing script using Locust.

## Setup and Installation

1.  **Clone the repository**:
    ```bash
    git clone <your-repository-url>
    cd Physical_AI_And_Robotics
    ```

2.  **Create and activate a virtual environment**:
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows use `venv\Scripts\activate`
    ```

3.  **Install dependencies**:
    ```bash
    pip install -r backend/requirements.txt
    pip install "fastapi[all]" # For the FastAPI endpoint
    pip install uvicorn locust # For running the API and load tests
    ```

4.  **Environment Variables**:
    Create a `.env` file in the project root and populate it with the following:
    ```
    COHERE_API_KEY="your_cohere_api_key"
    QDRANT_URL="your_qdrant_url"
    QDRANT_API_KEY="your_qdrant_api_key"
    QDRANT_COLLECTION_NAME="rag_validation" # Or your preferred collection name
    ```
    *Replace `"your_cohere_api_key"`, `"your_qdrant_url"`, and `"your_qdrant_api_key"` with your actual credentials.*

## Usage

### 1. Run the FastAPI Application

To start the retrieval API server:
```bash
uvicorn backend.src.api.retrieval_api:app --host 0.0.0.0 --port 8000 --reload
```
The API will be available at `http://0.0.0.0:8000`. You can access the OpenAPI documentation at `http://0.0.0.0:8000/docs`.

### 2. Testing Retrieval Accuracy (Pytest)

Run the unit and integration tests:
```bash
pytest backend/tests/test_retrieval.py
```

### 3. Running Performance Tests (Locust)

To run the load tests:
1.  Ensure the FastAPI application is running (see step 1).
2.  Start Locust from the project root:
    ```bash
    locust -f tests/performance/test_retrieval_performance.py
    ```
3.  Open your browser to `http://localhost:8089` (Locust UI).
4.  Enter the host (e.g., `http://localhost:8000`) and start the test.

### 4. Direct Retrieval (Python Script)

You can also use the `RetrievalMain` class directly in a Python script for testing:

```python
import os
from backend.src.retrieval.main import RetrievalMain
from backend.src.retrieval.models import Query
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

retrieval_tool = RetrievalMain()
test_query = Query(query="What is a neural network?", top_k=3)
results = retrieval_tool.retrieve_chunks(test_query)

for chunk in results.retrieved_chunks:
    print(f"Score: {chunk.score}, Text: {chunk.text[:100]}...")
    print(f"  Metadata: {chunk.metadata}")
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

[MIT](https://choosealicense.com/licenses/mit/)
