# Quickstart: Qdrant Embedding Pipeline & AI-Native RAG Chatbot System

## 1. Overview

This repository contains the infrastructure and code for the "Physical AI & Humanoid Robotics" textbook, including:
- An **AI-Native RAG Chatbot System** for interactive learning and querying the book content.
- A **Qdrant Embedding Pipeline** (`ingest_book.py`) to populate the Qdrant Cloud database with the textbook's content.

## 2. Qdrant Embedding Pipeline

This guide provides the steps to run the `ingest_book.py` script. This script will populate your Qdrant Cloud database with the content of the "Physical AI & Humanoid Robotics" textbook.

### 2.1. Prerequisites

- Python 3.10 or higher.
- A Qdrant Cloud account and API key.
- A Cohere account and API key.

### 2.2. Setup

### 2.2.1. Environment Variables

Create a `.env` file in the `data_ingestion/` directory or set the following environment variables in your system:

```
QDRANT_URL="<YOUR_QDRANT_CLOUD_URL>"
QDRANT_API_KEY="<YOUR_QDRANT_API_KEY>"
COHERE_API_KEY="<YOUR_COHERE_API_KEY>"
```

### 2.2.2. Install Dependencies

Install the required Python libraries:

```bash
cd data_ingestion
pip install -r requirements.txt
# (Assuming requirements.txt contains requests "qdrant-client[fastembed]" cohere trafilatura)
```

### 2.3. Running the Script

Execute the script from your terminal:

```bash
cd data_ingestion
python ingest_book.py
```

### 2.4. Expected Output

The script will provide logs indicating its progress. You should see output similar to the following:

```
INFO:root:Found 25 URLs in the sitemap.
INFO:root:Processing URL 1/25: https://physical-ai-and-robotics-five.vercel.app/docs/intro
INFO:root:Extracted 5 chunks from the URL.
INFO:root:Upserted 5 points to Qdrant collection 'ai_robotics_book'.
...
INFO:root:Processing URL 25/25: https://physical-ai-and-robotics-five.vercel.app/docs/capstone-project/autonomous-humanoid
...
INFO:root:Successfully ingested 150 chunks into the Qdrant collection.
INFO:root:Validation complete: Found 150 points in the collection.
```

## 3. AI-Native RAG Chatbot System

This system provides an interactive chatbot integrated into the Docusaurus frontend, powered by FastAPI backend, Qdrant vector search, and Neon Postgres for user personalization.

### 3.1. Features

*   **Normal RAG**: Answers questions using the full book content.
*   **Selected-Text RAG**: Answers questions specifically about highlighted text on a Docusaurus page.
*   **User Profile Management**: Secure account creation, login, and personalization preferences.
*   **Personalized Responses**: Chatbot tailors responses based on user's background and preferences.

### 3.2. Setup and Deployment

For detailed setup instructions, including environment variables, and deployment guides for both the FastAPI backend (Railway) and the Docusaurus frontend (Vercel), please refer to:

[docs/deployment.md](docs/deployment.md)

## 4. Verification

After the script completes, you can log in to your Qdrant Cloud dashboard to verify that the `ai_robotics_book` collection has been created and populated with the correct number of points.
