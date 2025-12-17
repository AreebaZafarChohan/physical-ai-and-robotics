# Quickstart: RAG Agent with Gemini Backend

This guide provides a quick overview of how to set up and interact with the RAG Agent with Gemini Backend.

## Prerequisites

-   Python 3.9+ installed.
-   `pip` for package management.
-   Access to a Qdrant instance (local or remote) with a populated collection (as per Spec-2).
-   A Gemini API key and an OpenAI-compatible endpoint URL for Gemini.

## Setup

1.  **Clone the repository (if you haven't already):**
    ```bash
    git clone [repository_url]
    cd Physical_AI_And_Robotics
    git checkout 006-rag-agent-gemini
    ```

2.  **Create and activate a virtual environment:**
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
    ```

3.  **Install dependencies:**
    ```bash
    pip install -r backend/requirements.txt
    ```
    *(Note: `backend/requirements.txt` will need to be updated with necessary packages like `fastapi`, `uvicorn`, `qdrant-client`, `openai-agents-sdk`, `async-openai`.)*

4.  **Configure Environment Variables:**
    Set the following environment variables:
    -   `GEMINI_API_KEY`: Your Gemini API key.
    -   `GEMINI_API_BASE_URL`: The URL of the OpenAI-compatible Gemini API endpoint.
    -   `QDRANT_URL`: The URL of your Qdrant instance.
    -   `QDRANT_API_KEY`: (Optional) API key for Qdrant if authentication is enabled.
    -   `QDRANT_COLLECTION_NAME`: The name of the Qdrant collection to query.
    -   `COHERE_API_KEY`: Your Cohere API key (used for embeddings, assume embedding service is available).

## Running the FastAPI Service

1.  **Start the FastAPI application:**
    ```bash
    uvicorn backend.src.api.main:app --host 0.0.0.0 --port 8000 --reload
    ```
    *(Note: The exact path to the FastAPI app might differ based on final project structure.)*

2.  **Access the API:**
    The API will be available at `http://localhost:8000`. You can test the `/chat` endpoint using `curl` or a tool like Postman/Insomnia.

    Example `curl` request:
    ```bash
    curl -X POST "http://localhost:8000/chat" \
         -H "Content-Type: application/json" \
         -d '{"query": "What is ROS 2?"}'
    ```

## Interacting with the RAG Agent

-   Once the FastAPI service is running, send `POST` requests to `/chat` with your questions.
-   The agent will retrieve relevant information from Qdrant and respond with a grounded answer or "I don't know" if no relevant content is found.
