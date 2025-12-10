# Data Model for AI-Native RAG Chatbot System

This document outlines the key entities and their attributes, including relationships and validation rules.

## Entities

### User Profile

Represents a user's background and preferences, stored in Neon Serverless Postgres.

-   **Table Name**: `users`
-   **Attributes**:
    -   `user_id`: UUID (Primary Key, unique identifier), auto-generated.
    -   `email`: VARCHAR(255), UNIQUE, NOT NULL (for authentication).
    -   `hashed_password`: VARCHAR(255), NOT NULL (stored securely).
    -   `background_summary`: TEXT (User's self-described hardware/software background).
    -   `preferred_difficulty`: VARCHAR(50), DEFAULT 'Beginner' (Allowed values: 'Beginner', 'Intermediate', 'Advanced', 'Expert').
    -   `preferred_language`: VARCHAR(10), DEFAULT 'English' (Allowed values: 'English', 'Urdu').
    -   `personalization_enabled`: BOOLEAN, DEFAULT FALSE (Flag to enable/disable personalization).
    -   `created_at`: TIMESTAMP WITH TIME ZONE, DEFAULT CURRENT_TIMESTAMP.
    -   `updated_at`: TIMESTAMP WITH TIME ZONE, DEFAULT CURRENT_TIMESTAMP.

### Chat Query

Represents a user's input query to the chatbot. This is a transient data structure used for API requests.

-   **Attributes**:
    -   `query_text`: STRING (The actual question from the user).
    -   `selected_text`: STRING, OPTIONAL (Text highlighted by the user for selected-text RAG mode).
    -   `user_id`: UUID (Used for retrieving personalization settings).

### Chat Response

Represents the chatbot's output, streamed back to the frontend. This is a transient data structure for API responses.

-   **Attributes**:
    -   `response_text`: STRING (The generated answer from the AI model).
    -   `citations`: LIST of STRING (List of chunk IDs from Qdrant used as citations).
    -   `chunks_used`: INTEGER (Number of chunks used to generate the response).
    -   `personalization_applied`: BOOLEAN (Indicates if personalization was applied to the response).