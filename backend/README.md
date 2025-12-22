# RAG Agent with Gemini Backend

This project implements a Retrieval-Augmented Generation (RAG) agent using the OpenAI Agents SDK, leveraging a Gemini model (via an OpenAI-compatible API) as the backend LLM. It interacts with a Qdrant vector database (populated with Cohere embeddings) for content retrieval, ensuring answers are strictly grounded in retrieved information. A FastAPI endpoint exposes the agent for interaction.

## Features

-   **Grounded Responses**: Agent formulates answers strictly based on retrieved information.
-   **Fallback for Unanswerable Questions**: Agent responds with "I don't know" when no relevant information is found.
-   **Qdrant Integration**: Utilizes Qdrant for efficient vector search and retrieval.
-   **Cohere Embeddings**: Generates embeddings for user queries using Cohere.
-   **FastAPI Endpoint**: Exposes the RAG agent functionality via a high-performance HTTP API.
-   **Environment Variable Configuration**: Securely manages API keys and other configurations.
-   **Comprehensive Logging**: Detailed logs for monitoring and debugging.

## Project Structure

-   `backend/src/api/`: FastAPI endpoint definitions (`main.py`).
-   `backend/src/models/`: Pydantic models, data structures.
-   `backend/src/services/`: Business logic, agent orchestration.
-   `backend/src/retrieval/`: Qdrant client, Cohere embeddings.
-   `backend/src/agent/`: OpenAI Agents SDK configuration and agent definition.
-   `backend/src/utils/`: Utility functions (e.g., logging, config).
-   `backend/requirements.txt`: Python dependencies.
-   `backend/tests/`: Unit and integration tests.

## Setup and Installation

1.  **Clone the repository**:
    ```bash
    git clone <your-repository_url>
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
    ```

4.  **Configure Environment Variables**:
    Create a `.env` file in the `backend/` directory (or in the project root if preferred, ensuring it's loaded) and populate it with the following. You can use `backend/.env.example` as a template. For local development, you can use `backend/.env.development`.
    ```
    GEMINI_API_KEY="your_gemini_api_key"
    GEMINI_API_BASE_URL="your_gemini_api_base_url"
    QDRANT_URL="your_qdrant_url"
    QDRANT_API_KEY="your_qdrant_api_key"
    QDRANT_COLLECTION_NAME="your_qdrant_collection_name"
    COHERE_API_KEY="your_cohere_api_key"
    
    # Google OAuth Configuration (for social login)
    GOOGLE_CLIENT_ID="YOUR_GOOGLE_CLIENT_ID"
    GOOGLE_CLIENT_SECRET="YOUR_GOOGLE_CLIENT_SECRET"
    GOOGLE_REDIRECT_URI="http://localhost:9000/api/v1/auth/google/callback"
    ```
    *Replace placeholder values with your actual credentials and configurations.*

## Running the FastAPI Service

1.  **Start the FastAPI application**:
    ```bash
    uvicorn backend.src.api.main:app --host 0.0.0.0 --port 8000 --reload
    ```
    The API will be available at `http://localhost:8000`. You can access the OpenAPI documentation at `http://localhost:8000/docs`.

## Interacting with the RAG Agent

-   Once the FastAPI service is running, send `POST` requests to `/chat` with your questions.
-   The agent will retrieve relevant information from Qdrant and respond with a grounded answer or "I don't know" if no relevant content is found.

Example `curl` request:
```bash
curl -X POST "http://localhost:8000/chat" \
         -H "Content-Type: application/json" \
         -d '{"query": "What is ROS 2?"}'
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

[MIT](https://choosealicense.com/licenses/mit/)