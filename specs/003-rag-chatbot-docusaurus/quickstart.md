# Quickstart Guide: RAG Chatbot for Docusaurus Book

This guide outlines the high-level steps for setting up and running the RAG Chatbot locally for development purposes.

## 1. Clone the repository

First, clone the main project repository to your local machine:

```bash
git clone [your-repository-url]
cd Physical_AI_And_Robotics # Adjust if your repository has a different name
```

## 2. Frontend Setup (Docusaurus)

The chatbot widget will be integrated into the Docusaurus frontend.

1.  Navigate to the `frontend/` directory:
    ```bash
    cd frontend/
    ```
2.  Install the necessary Node.js dependencies:
    ```bash
    npm install
    ```
3.  Start the Docusaurus development server:
    ```bash
    npm start
    ```
    This will typically open a browser window to `http://localhost:3000` where you can see the Docusaurus website.

## 3. Backend Setup (FastAPI)

The chatbot's API is powered by a FastAPI backend.

1.  Navigate to the `backend/` directory (you might need to create this directory and project structure during implementation):
    ```bash
    cd ../backend/
    ```
2.  Create and activate a Python virtual environment:
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows, use `.\venv\Scripts\activate`
    ```
3.  Install Python dependencies:
    ```bash
    pip install -r requirements.txt # This file should now exist
    ```
4.  Configure Environment Variables:
    Create a `.env` file in the `backend/` directory and add the following (replace placeholders with your actual keys/URLs). This file was automatically generated.
    ```
    OPENAI_API_KEY="your_openai_api_key"
    NEON_DB_URL="your_neon_postgres_connection_string"
    QDRANT_API_KEY="your_qdrant_api_key"
    QDRANT_CLUSTER_URL="your_qdrant_cluster_url"
    JWT_SECRET_KEY="a_strong_secret_key_for_jwt"
    ```
5.  Start the FastAPI development server:
    ```bash
    uvicorn main:app --reload
    ```
    The backend API will typically run on `http://localhost:8000`.

## 4. Database Initialization

1.  **Neon Postgres**:
    *   Ensure your Neon Postgres database is provisioned and accessible via the `NEON_DB_URL` configured above.
    *   Run any database migrations to set up the `users`, `chat_sessions`, `messages`, `feedback`, and `book_metadata` tables. (Migration scripts will be developed as part of the implementation).
    *   **Action**: To create tables initially, you might run a script like `python -c "from backend.app.database import create_db_and_tables; create_db_and_tables()"` (once `create_db_and_tables` is fully implemented).
2.  **Qdrant Cloud**:
    *   Ensure your Qdrant Cloud instance is accessible via the `QDRANT_API_KEY` and `QDRANT_CLUSTER_URL`.
    *   The `book_chapters` collection will be created automatically upon the first embedding ingestion, or can be created manually.
    *   Ingest initial book and course content using the `/embed` API endpoint (details for this process will be provided in the Data Pipeline documentation).
    *   **Action**: To create the Qdrant collections initially, you can make a call to the `/embed` endpoint or manually create them if needed.

## 5. Access the Chatbot

Once both the frontend and backend services are running, open the Docusaurus website in your browser (e.g., `http://localhost:3000`). The floating chatbot widget should be visible in the bottom-right corner, and you can begin interacting with it.
