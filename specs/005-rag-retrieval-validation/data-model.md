# Data Model for RAG Retrieval Pipeline Validation

This document outlines the data structures used in the RAG retrieval pipeline validation tool.

## Entities

### Query

Represents the input from the user.

-   **query**: `string` - The natural language query.
-   **top_k**: `integer` - The number of results to return.

### RetrievedChunk

Represents a single chunk of text retrieved from the vector database.

-   **text**: `string` - The content of the chunk.
-   **score**: `float` - The similarity score of the chunk to the query.
-   **metadata**: `object` - A nested object containing source information.

### Metadata

Provides context for a `RetrievedChunk`.

-   **source_url**: `string` - The URL of the source document.
-   **section**: `string` - The section of the document the chunk is from.
-   **heading**: `string` - The heading of the section.
-   **chunk_index**: `integer` - The index of the chunk within the document.

## API Contracts

### POST /retrieve

**Request Body:**

```json
{
    "query": "What is ROS 2?",
    "top_k": 5
}
```

**Response Body (200 OK):**

```json
{
    "query": "What is ROS 2?",
    "embedding_model": "embed-english-v3.0",
    "top_k": 5,
    "retrieved_chunks": [
        {
            "text": "ROS 2 is a set of software libraries and tools that help you build robot applications.",
            "score": 0.95,
            "metadata": {
                "source_url": "https://example.com/ros2/intro",
                "section": "Introduction",
                "heading": "What is ROS 2?",
                "chunk_index": 0
            }
        }
    ]
}
```
