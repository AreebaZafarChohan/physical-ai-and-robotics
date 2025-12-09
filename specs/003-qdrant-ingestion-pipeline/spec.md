---
title: "Qdrant Embedding Pipeline for Humanoid AI Book"
description: "A complete ingestion pipeline for the Physical AI & Humanoid Robotics textbook."
date: 2025-12-09
feature:
  - "[[003-qdrant-ingestion-pipeline]]"
branch:
  - "[[003-qdrant-ingestion-pipeline]]"
---

## 1. Overview

We need to create a complete ingestion pipeline that takes all pages of our Docusaurus-based Physical AI & Humanoid Robotics textbook, extracts text, generates vector embeddings using Cohere’s embed-english-v3.0 model, and stores all chunks into a Qdrant Cloud collection. This is Step 1 of building the RAG chatbot.

## 2. Clarifications

### Session 2025-12-09

- Q: The spec says to "recreate" the Qdrant collection if it already exists. This is a destructive action. What should be the behavior? → A: Prompt the user for confirmation before deleting an existing collection.
- Q: If a single page URL from the sitemap fails to download (e.g., 404 error), should the script halt or continue? → A: Skip the failed URL and continue with the rest.
- Q: What should be the retry policy for failed requests to external services (Cohere, Qdrant, website URLs)? → A: Implement a 3-retry policy with exponential backoff.

## 3. User Scenarios & Experience

- **Actor**: Panaversity Hackathon Participant
- **Goal**: To populate a Qdrant vector database with the entire content of the online textbook to enable a RAG-based chatbot.
- **Scenario**:
  1. The user obtains the ingestion script.
  2. They set up their environment with the required API keys for Cohere and Qdrant.
  3. They run the script `ingest_book.py`.
  4. The script fetches the sitemap from the live textbook URL, processes each page, and ingests the content into their Qdrant collection.
  5. The user can then verify in their Qdrant Cloud dashboard that the collection has been created and populated with data.

## 4. Functional Requirements

### 4.1. Data Ingestion

- The system must fetch the `sitemap.xml` from the provided book URL.
- The system must parse all URLs from the sitemap.
- For each URL, the system must fetch the HTML content.
- The system must extract clean, readable text from the HTML.

### 4.2. Data Processing

- The extracted text must be chunked into smaller segments (approx. 1200 characters).
- Chunking logic must attempt to preserve sentence boundaries.
- The system must generate vector embeddings for each text chunk using the Cohere `embed-english-v3.0` model.

### 4.3. Data Storage

- The system must connect to a Qdrant Cloud instance.
- If the collection already exists, the script must prompt the user for confirmation before recreating it.
- The collection must be configured with a vector size of 1024 and a COSINE distance metric.
- Each vector must be stored with a corresponding payload containing:
  - `url`: The source URL of the page.
  - `text`: The original text chunk.
  - `chunk_id`: A unique identifier for the chunk within the page.

### 4.4. Execution & Environment

- The entire process must be encapsulated in a single Python script: `ingest_book.py`.
- The script must be runnable from the command line.
- The script must use environment variables for API keys (Cohere, Qdrant).
- The script must provide clear logging for each major step of the process.
- The script must handle and log errors gracefully.

### 4.5. Error Handling and Resilience

- For any failed network requests (fetching URLs, Cohere API, Qdrant API), the script will retry up to 3 times with exponential backoff.
- If a URL from the sitemap fails to process after all retries, the script will log the error and continue to the next URL.

## 5. Success Criteria

- **Completeness**: 100% of the pages listed in the `sitemap.xml` are processed and ingested, gracefully skipping any permanently inaccessible URLs.
- **Data Integrity**: No text chunks are missing or empty after processing.
- **Verification**: The script successfully upserts all generated vectors and payloads into the Qdrant collection, verified by querying the collection count.
- **Portability**: The Python script runs without modification on different machines, provided the correct environment variables are set.
- **Usability**: The script provides clear, actionable logs that allow a user to understand its progress and troubleshoot errors.

## 6. Scope

### 6.1. In Scope

- A single, executable Python script for data ingestion.
- Fetching and parsing a Docusaurus `sitemap.xml`.
- Text extraction from HTML.
- Text chunking and embedding generation with Cohere.
- Storing vectors and metadata in a Qdrant Cloud collection.

### 6.2. Out of Scope

- The RAG chatbot or any query-side logic.
- A FastAPI or other web service endpoint.
- User authentication or content personalization.
- The use of any embedding models other than Cohere's `embed-english-v3.0`.
- Creation or management of multiple Qdrant collections.

## 7. Assumptions

- The user has active API keys for both Qdrant Cloud and Cohere.
- The Docusaurus site at the provided URL has a publicly accessible `sitemap.xml`.
- The Python environment will have the allowed libraries installed (`requests`, `xml.etree.ElementTree`, `trafilatura`, `qdrant-client`, `cohere`).
