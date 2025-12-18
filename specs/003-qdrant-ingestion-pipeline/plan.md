---
title: "Implementation Plan: Qdrant Embedding Pipeline"
description: "A detailed plan to create the ingestion pipeline for the Physical AI & Humanoid Robotics textbook."
date: 2025-12-09
feature: "[[003-qdrant-ingestion-pipeline]]"
branch: "[[003-qdrant-ingestion-pipeline]]"
---

## 1. Technical Context

This plan outlines the creation of a Python script, `ingest_book.py`, to build a vector store for a RAG chatbot. The script will fetch content from a Docusaurus website, process it, and store it in a Qdrant Cloud collection.

- **Primary Language**: Python 3.10+
- **Key Libraries**: `requests`, `xml.etree.ElementTree`, `trafilatura`, `qdrant-client`, `cohere`
- **Vector Database**: Qdrant Cloud
- **Embedding Model**: Cohere `embed-english-v3.0` (1024 dimensions)
- **Target Environment**: A command-line interface where Python is installed.

## 2. Constitution Check

- **[✅] Alignment**: The plan aligns with the constitution's goal of building an AI-native textbook with a RAG chatbot.
- **[✅] Stack**: The chosen technologies (Python, Qdrant, Cohere) are consistent with the "Chatbot Integration Stack" defined in the constitution.
- **[✅] Scope**: The plan is tightly focused on the ingestion pipeline, which is the first step of the chatbot feature.

## 3. High-Level Phases

1.  **Setup & Initialization**: Configure the script's environment and constants.
2.  **Sitemap Parsing**: Fetch and parse the website's sitemap.
3.  **Text Extraction**: Download and clean text from each page.
4.  **Chunking Strategy**: Break down the text into manageable pieces for embedding.
5.  **Embedding Generation**: Convert text chunks into vector embeddings.
6.  **Qdrant Storage Pipeline**: Store the vectors and their metadata in Qdrant.
7.  **Script Assembly & Testing**: Combine all components and test the end-to-end pipeline.
8.  **Final Deliverables Packaging**: Prepare the script and documentation for distribution.

## 4. Detailed Plan

### Phase 1 — Setup & Initialization

- **Task 1.1**: Define the project structure for the script.
- **Task 1.2**: Document required environment variables (`QDRANT_API_KEY`, `COHERE_API_KEY`, `QDRANT_URL`).
- **Task 1.3**: Import all required libraries at the top of the script.
- **Task 1.4**: Configure global constants:
    - `SITEMAP_URL`: "https://physical-ai-and-robotics-five.vercel.app/sitemap.xml"
    - `COLLECTION_NAME`: "ai_robotics_book"
    - `COHERE_MODEL`: "embed-english-v3.0"
    - `VECTOR_SIZE`: 1024

### Phase 2 — Sitemap Parsing

- **Task 2.1**: Implement a function `fetch_sitemap_urls(url)` to download and parse the `sitemap.xml`.
- **Task 2.2**: Extract all `<loc>` text content into a list of URLs.
- **Task 2.3**: Add logging to print the number of URLs found.
- **Task 2.4**: Add a validation check to exit if no URLs are found.

### Phase 3 — Text Extraction

- **Task 3.1**: Implement a function `extract_text_from_url(url)` that:
    - Downloads the HTML content using `requests`.
    - Extracts clean text using `trafilatura.extract()`.
    - Implements a retry mechanism (3 retries with exponential backoff) for failed downloads.
- **Task 3.2**: If extraction returns no text, log a warning and return `None`.

### Phase 4 — Chunking Strategy

- **Task 4.1**: Implement a function `chunk_text(text, chunk_size=1200)` that:
    - Splits the text into chunks of approximately `chunk_size` characters.
    - Prioritizes splitting along sentence boundaries (`. `).
    - Returns a list of text chunks.

### Phase 5 — Embedding Generation

- **Task 5.1**: Implement a function `get_embedding(text, model)` that:
    - Calls the Cohere client to generate an embedding for the given text.
    - Uses `input_type="search_query"`.
    - Implements a retry mechanism for API failures.
    - Returns the embedding vector.

### Phase 6 — Qdrant Storage Pipeline

- **Task 6.1**: Initialize the Qdrant client.
- **Task 6.2**: Implement logic to check if the collection exists. If it does, prompt the user for confirmation before deleting and recreating it with `recreate_collection()`.
    - `vector_size`: `VECTOR_SIZE`
    - `distance`: `models.Distance.COSINE`
- **Task 6.3**: Loop through each URL and its extracted text chunks.
- **Task 6.4**: For each chunk, generate an embedding and prepare a `PointStruct` with:
    - `id`: A unique, incrementing integer.
    - `vector`: The generated embedding.
    - `payload`: `{"url": url, "text": chunk, "chunk_id": chunk_index}`.
- **Task 6.5**: Upsert points to Qdrant in batches to improve efficiency.

### Phase 7 — Script Assembly & Testing

- **Task 7.1**: Create a `main()` function to orchestrate the entire pipeline from sitemap fetching to storage.
- **Task 7.2**: Add `if __name__ == "__main__":` to execute the `main` function.
- **Task 7.3**: Add logging to indicate progress at each phase.
- **Task 7.4**: Implement a final validation step that queries the Qdrant collection and prints the total number of points to verify success.

### Phase 8 — Final Deliverables Packaging

- **Task 8.1**: Finalize the `ingest_book.py` script with all functions and logic.
- **Task 8.2**: Create a `quickstart.md` file that includes:
    - Instructions on how to set up the environment and install dependencies.
    - The required environment variables.
    - An example of how to run the script: `python ingest_book.py`.
    - A sample of the expected console output.

## 5. ADR (Architectural Decision Record) Suggestions

No major architectural decisions are required for this plan, as it follows a standard ingestion pipeline pattern. The technology stack is pre-defined by the project constitution.