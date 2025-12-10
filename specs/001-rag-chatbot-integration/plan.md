# Implementation Plan: AI-Native RAG Chatbot System

**Feature Branch**: `001-rag-chatbot-integration`  
**Created**: 2025-12-10  
**Status**: Draft  
**Specification**: [./spec.md](./spec.md)

## High-Level Summary

The RAG Chatbot System will be a Docusaurus-embedded chatbot powered by the OpenAI Agents SDK with a Gemini model. It will leverage Qdrant Cloud for vector retrieval and Neon Postgres for user personalization. The chatbot will support both standard RAG (full book context) and selected-text RAG modes, delivering streamed responses. User authentication will be handled via JWT. The frontend will provide clear UI feedback for loading and errors.

## System Architecture

```
+---------------------+      +------------------------+      +-------------------+
|  Docusaurus Frontend  |<---->|   ChatKit UI Component |<---->| FastAPI Backend   |
+----------^----------+      +------------------------+      +---------^---------+
           |                                                       |
           | HTTP/S                                                | HTTP/S (API Calls)
           |                                                       |
+----------+----------+      +------------------------+      +---------+---------+
|  User Browser/Client |<---->|    Auth Layer (JWT)    |<---->| OpenAI Agents SDK |
+---------------------+      +------------------------+      +---------^---------+
                                                                 |
                                                                 | Function Calls
                                                                 |
                     +-------------------------------------------------------------+
                     |                                                             |
                     |  +--------------------+  +----------------------+  +---------------------+
                     +->| qdrant_search_tool |  | selected_text_tool   |  | neon_preferences_tool |
                        +--------------------+  +----------------------+  +---------------------+
                               |                           |                           |
                               | gRPC/REST                 | Direct Text Input         | SQL/REST
                               |                           |                           |
                     +---------+---------+       +---------+---------+       +---------+---------+
                     | Qdrant Cloud Vector DB|<----->| FastAPI Backend (Internal) |<----->| Neon Postgres User DB |
                     +---------------------+       +------------------------+       +---------------------+
```

## Data Flow (End-to-End)

1.  **Normal RAG query:**
    *   User inputs query in Docusaurus-embedded ChatKit UI.
    *   ChatKit UI sends query to FastAPI `/api/v1/chat` endpoint (authenticated with JWT).
    *   FastAPI backend passes query to OpenAI Agents SDK.
    *   Agent uses `qdrant_search_tool` to query Qdrant Cloud with user's query.
    *   Qdrant returns top-k relevant text chunks.
    *   FastAPI backend injects retrieved context (chunks) and user personalization (if enabled and available) into the Agent's prompt.
    *   Agent (Gemini model) generates a response based on the context.
    *   FastAPI streams the response back to ChatKit UI.
    *   ChatKit UI renders markdown response, including citations (chunk IDs).

2.  **Selected-text mode query:**
    *   User highlights text in Docusaurus, and "Ask AI about this" button appears.
    *   User clicks button, ChatKit UI sends selected text and query (if any) to FastAPI `/api/v1/selected-chat` endpoint (authenticated with JWT).
    *   FastAPI backend passes selected text to OpenAI Agents SDK.
    *   Agent uses `selected_text_tool` which bypasses vector search.
    *   FastAPI backend injects selected text and strong "only answer using provided text" instruction into the Agent's prompt, along with user personalization.
    *   Agent (Gemini model) generates a response *only* based on the selected text.
    *   FastAPI streams the response back to ChatKit UI.
    *   ChatKit UI renders markdown response.

3.  **User preference loading:**
    *   Upon user login/session start, ChatKit UI (or FastAPI on its behalf) calls FastAPI `/api/v1/user/profile` endpoint.
    *   FastAPI backend queries Neon Postgres via `neon_preferences_tool` to fetch `user_id`, `background_summary`, `preferred_difficulty`, `preferred_language`, `personalization_enabled`.
    *   FastAPI stores these preferences in session or passes them to the Agent for prompt modification.

4.  **Response streaming:**
    *   FastAPI uses Server-Sent Events (SSE) or WebSockets to stream partial responses from the Gemini model to the ChatKit frontend.
    *   ChatKit UI incrementally renders the markdown output as it arrives.

5.  **Storing user settings:**
    *   User updates preferences in ChatKit UI.
    *   ChatKit UI sends update request to FastAPI `/api/v1/user/update` endpoint (authenticated with JWT).
    *   FastAPI backend updates the corresponding user profile in Neon Postgres.

## Backend Services

-   `/agents/`: Directory for OpenAI Agents SDK related setup and model instantiation.
    -   `__init__.py`: Initialization for agents module.
    -   `gemini_agent.py`: Configures `gemini-2.0-flash` via `OpenAIChatCompletionsModel` and orchestrates tool use.
-   `/tools/`: Directory for custom function tools used by the Agent.
    -   `__init__.py`: Initialization for tools module.
    -   `qdrant_tool.py`: Implements `qdrant_search_tool` using `@function_tool` decorator, handles Qdrant Cloud interaction.
    -   `selected_text_tool.py`: Implements `selected_text_tool` using `@function_tool` decorator, processes direct text input.
    -   `neon_user_profile_tool.py`: Implements `neon_preferences_tool` using `@function_tool` decorator, interacts with Neon Postgres for user profiles.
-   `/routes/`: FastAPI router definitions.
    -   `__init__.py`: Initialization for routes module.
    -   `chat.py`: Defines chat-related endpoints (`/api/v1/chat`, `/api/v1/selected-chat`).
    -   `user.py`: Defines user-related endpoints (`/api/v1/auth/signup`, `/api/v1/auth/signin`, `/api/v1/user/profile`, `/api/v1/user/update`).
-   `/db/`: Neon Postgres connector and SQL models.
    -   `__init__.py`: Initialization for db module.
    -   `connection.py`: Establishes and manages connection to Neon Postgres.
    -   `models.py`: Defines SQL models for User Profile.
    -   `crud.py`: Implements Create, Read, Update, Delete operations for User Profile.
-   `/vector/`: Qdrant connector.
    -   `__init__.py`: Initialization for vector module.
    -   `qdrant_client.py`: Configures Qdrant client, handles connection to `ai_robotics_book` collection.
-   `/utils/`: Utility functions.
    -   `__init__.py`: Initialization for utils module.
    -   `logging.py`: Centralized logging configuration.
    -   `errors.py`: Custom exception handling and error response schemas.
    -   `security.py`: JWT encoding/decoding, password hashing.
    -   `cors.py`: CORS middleware configuration for FastAPI.
    -   `prompts.py`: Manages system prompt templates.

## FastAPI Endpoints

1.  **POST `/api/v1/auth/signup`**
    *   **Request Body**: `{"username": "string", "email": "string", "password": "string", "background_summary": "string"}`
    *   **Response Schema**: `{"message": "User created successfully"}` (or JWT token on success if directly logging in)
    *   **Tools Called**: `neon_preferences_tool` (for user creation)
    *   **Streaming Enabled**: No
    *   **Personalization Applied**: No

2.  **POST `/api/v1/auth/signin`**
    *   **Request Body**: `{"email": "string", "password": "string"}`
    *   **Response Schema**: `{"access_token": "string", "token_type": "bearer"}`
    *   **Tools Called**: `neon_preferences_tool` (for user authentication)
    *   **Streaming Enabled**: No
    *   **Personalization Applied**: No

3.  **GET `/api/v1/user/profile`**
    *   **Request Body**: None (requires valid JWT in header)
    *   **Response Schema**: `{"user_id": "UUID", "background_summary": "string", "preferred_difficulty": "string", "preferred_language": "string", "personalization_enabled": "boolean"}`
    *   **Tools Called**: `neon_preferences_tool`
    *   **Streaming Enabled**: No
    *   **Personalization Applied**: No

4.  **PUT `/api/v1/user/update`**
    *   **Request Body**: `{"background_summary": "string", "preferred_difficulty": "string", "preferred_language": "string", "personalization_enabled": "boolean"}` (partial updates allowed)
    *   **Response Schema**: `{"message": "Profile updated successfully"}`
    *   **Tools Called**: `neon_preferences_tool`
    *   **Streaming Enabled**: No
    *   **Personalization Applied**: No

5.  **POST `/api/v1/chat`**
    *   **Request Body**: `{"query": "string"}` (requires valid JWT in header)
    *   **Response Schema**: `StreamingResponse` (JSON stream with `response`, `citations`, `chunks_used`, `personalization_applied`)
    *   **Tools Called**: `qdrant_search_tool`, `neon_preferences_tool`
    *   **Streaming Enabled**: Yes
    *   **Personalization Applied**: Yes

6.  **POST `/api/v1/chat/selected`**
    *   **Request Body**: `{"query": "string", "selected_text": "string"}` (requires valid JWT in header)
    *   **Response Schema**: `StreamingResponse` (JSON stream with `response`, `citations`, `chunks_used`, `personalization_applied`)
    *   **Tools Called**: `selected_text_tool`, `neon_preferences_tool`
    *   **Streaming Enabled**: Yes
    *   **Personalization Applied**: Yes

## Agents + Tools Design

-   **Model**: `gemini-2.0-flash` via `OpenAIChatCompletionsModel` (using OpenAI-style wrapper).
-   **System prompt template for RAG mode**:
    ```
    "You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
    Answer the user's question based ONLY on the provided context.
    If the answer is not found in the context, state that you cannot answer from the provided information.
    Provide citations (chunk IDs) for each piece of information you use.

    User Profile: {user_profile_summary}
    Personalization Style: {personalization_style_description}
    ```
-   **System prompt template for Selected-Text mode**:
    ```
    "You are an AI assistant. Answer the user's question based ONLY on the provided text.
    If the answer is not found in the provided text, state that you cannot answer from the provided text.
    DO NOT use any external knowledge.

    User Profile: {user_profile_summary}
    Personalization Style: {personalization_style_description}
    ```
-   **Tool schema for Qdrant search (`qdrant_search_tool`)**:
    ```python
    @function_tool
    def qdrant_search_tool(query: str, collection_name: str = "ai_robotics_book", top_k: int = 5) -> List[Dict]:
        """
        Searches the Qdrant Cloud vector store for relevant text chunks.
        Args:
            query: The user's query string.
            collection_name: The name of the Qdrant collection to search. Defaults to "ai_robotics_book".
            top_k: The number of top relevant chunks to retrieve.
        Returns:
            A list of dictionaries, where each dictionary represents a chunk with 'content' and 'id'.
        """
        # Internal implementation to call Qdrant API
        pass
    ```
-   **Tool schema for Selected text (`selected_text_tool`)**:
    ```python
    @function_tool
    def selected_text_tool(selected_text: str, query: str = None) -> str:
        """
        Processes user-selected text as the primary context for answering a query.
        This tool bypasses vector search and uses the provided text directly.
        Args:
            selected_text: The text highlighted by the user.
            query: The user's question (optional, if query is implied by selection).
        Returns:
            The selected text itself, which will be injected into the model prompt.
        """
        # Internal implementation to return the selected_text
        pass
    ```
-   **Tool schema for Neon preferences loader (`neon_preferences_tool`)**:
    ```python
    @function_tool
    def neon_preferences_tool(user_id: str) -> Dict:
        """
        Fetches user personalization preferences from Neon Postgres.
        Args:
            user_id: The UUID of the user.
        Returns:
            A dictionary containing user preferences like background_summary, preferred_difficulty,
            preferred_language, and personalization_enabled status.
        """
        # Internal implementation to query Neon DB
        pass
    ```
-   **Tool arbitration behavior**: The Agent will prioritize `selected_text_tool` if `selected_text` is provided in the chat request. Otherwise, it will use `qdrant_search_tool` for general RAG queries. `neon_preferences_tool` will be called proactively to enrich the system prompt.
-   **Hallucination guardrails**: Strict instructions in the system prompts ("ONLY on the provided context/text", "DO NOT use any external knowledge"). Post-processing of model output to ensure citations are present and valid (for RAG mode).

## Retrieval Logic

-   **Embedding model**: `text-embedding-004`
-   **Vector size**: 768 (Confirmed for `text-embedding-004` model)
-   **top_k retrieval**: Default to 5 relevant chunks for RAG mode. Configurable via API parameter.
-   **Metadata filters**: Potential future enhancement for filtering by chapter, module, etc., if metadata is stored in Qdrant.
-   **How citations are mapped back to chunk IDs**: Qdrant results will include original chunk IDs which will be returned in the FastAPI response. Frontend will display these.
-   **Selected-text bypass logic**: Handled by `selected_text_tool` which directly injects the provided text into the prompt, bypassing Qdrant search entirely. If selected text exceeds a predefined token limit (e.g., 2000 tokens), it will be truncated from the middle to preserve context from both ends.

## Neon Personalization Logic

-   **User profile schema**:
    ```sql
    CREATE TABLE users (
        user_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        email VARCHAR(255) UNIQUE NOT NULL,
        hashed_password VARCHAR(255) NOT NULL,
        background_summary TEXT,
        preferred_difficulty VARCHAR(50) DEFAULT 'Beginner', -- 'Beginner', 'Intermediate', 'Advanced', 'Expert'
        preferred_language VARCHAR(10) DEFAULT 'English', -- 'English', 'Urdu'
        personalization_enabled BOOLEAN DEFAULT FALSE,
        created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
        updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
    );
    ```
-   **How background modifies the system prompt**: The `background_summary` will be included in the system prompt as `{user_profile_summary}`.
-   **How preferred difficulty modifies the system prompt**: The `preferred_difficulty` will be used to construct `{personalization_style_description}` in the system prompt (e.g., "Explain concepts in a beginner-friendly manner.").
-   **Fallbacks when profile missing**: If `personalization_enabled` is `FALSE` or profile data is missing, the system prompt will revert to a neutral, general-purpose style, or omit the personalization-specific parts.
-   **Optional difficulty tuning**: Handled by modifying the system prompt based on `preferred_difficulty`.
-   **Urdu/English output switching**: The `preferred_language` will instruct the Gemini model to respond in the chosen language.

## Frontend Integration Plan (ChatKit + Docusaurus)

-   **Component structure**:
    -   `<FloatingChatbot />`: Main ChatKit UI component, always present.
    -   `<AskFromSelection />`: A small, temporary UI element that appears when text is highlighted, containing the "Ask AI about this" button.
-   **Event flow: "highlight text → ask AI"**:
    1.  User highlights text on a Docusaurus page.
    2.  Docusaurus/JavaScript listener detects text selection.
    3.  `<AskFromSelection />` component becomes visible near the selected text.
    4.  User clicks "Ask AI about this" button.
    5.  The selected text (and potentially a default query like "Explain this.") is sent to the FastAPI `/api/v1/chat/selected` endpoint.
    6.  Chatbot UI opens (if not already open) and displays the query and incoming streamed response.
-   **How streaming is handled in ChatKit**: ChatKit SDK natively supports streamed responses. The frontend will establish an SSE or WebSocket connection to the FastAPI endpoint and update the UI incrementally as data arrives.
-   **How to display citations**: Citations (chunk IDs) returned in the JSON stream will be displayed as clickable links or footnotes in the ChatKit UI, linking to the corresponding section of the book (future enhancement: actual linking to text anchors).
-   **How to switch between the two modes**:
    -   Default mode is Normal RAG.
    -   Selected-Text RAG mode is triggered specifically by the "Ask AI about this" interaction.
    -   The ChatKit UI will default to Normal RAG. Selected-Text RAG is activated by highlight-to-chat. A clear indication of the active mode will be displayed. Explicit manual switching for general queries after highlighting is deferred to future iterations.

## DevOps & Deployment

-   **Environment variables needed**:
    -   `QDRANT_API_KEY`: Qdrant Cloud API key (read-only recommended).
    -   `QDRANT_URL`: Qdrant Cloud service URL.
    -   `NEON_DATABASE_URL`: Connection string for Neon Postgres.
    -   `OPENAI_API_KEY`: API key for OpenAI-compatible endpoint (for Gemini via wrapper).
    -   `JWT_SECRET_KEY`: Secret key for JWT signing.
    -   `ALGORITHM`: JWT algorithm (e.g., HS256).
    -   `ACCESS_TOKEN_EXPIRE_MINUTES`: JWT token expiry.
    -   `FRONTEND_ORIGIN`: CORS allowed origin for Docusaurus frontend.
-   **Qdrant API keys**: Qdrant read-only API key will be used by the backend.
-   **Neon connection string**: Full connection string including username, password, host, port, database name.
-   **Backend deployment on Railway/Render**:
    *   Containerized FastAPI application (Dockerfile).
    *   Deployment scripts/configuration for platform-specific setup (e.g., `railway.json`, `render.yaml`).
    *   Automatic deployment on Git push to `main` branch.
-   **Frontend deployment on Vercel**:
    *   Docusaurus static site deployment.
    *   Vercel CLI or Git integration for automatic deployment.
    *   CORS configuration on Vercel to allow requests to FastAPI backend.
-   **CORS configuration**: FastAPI backend will use `CORSMiddleware` to allow requests from the Docusaurus frontend domain (`FRONTEND_ORIGIN`).
-   **Folder tree for final repo**:
    ```
    /
    ├───.env.example
            ├───backend/
        │   ├───main.py
        │   ├───tests/
        │   ├───schemas/
        │   ├───agents/    │   │   └───gemini_agent.py
    │   ├───tools/
    │   │   ├───qdrant_tool.py
    │   │   ├───selected_text_tool.py
    │   │   └───neon_user_profile_tool.py
    │   ├───routes/
    │   │   ├───chat.py
    │   │   └───user.py
    │   ├───db/
    │   │   ├───connection.py
    │   │   ├───models.py
    │   │   └───crud.py
    │   ├───vector/
    │   │   └───qdrant_client.py
    │   ├───utils/
    │   │   ├───logging.py
    │   │   ├───errors.py
    │   │   ├───security.py
    │   │   └───cors.py
    │   └───requirements.txt
    ├───frontend/ (existing Docusaurus project)
    │   ├───src/
    │   │   ├───components/
    │   │   │   └───ChatbotComponent.js/tsx (ChatKit integration, FloatingChatbot, AskFromSelection)
    │   └───docusaurus.config.js
    ├───docs/
    │   └───(updated with RAG chatbot documentation)
    └───README.md (updated with deployment instructions and setup)
    ```

## Success Metrics

-   **Latency targets**:
    -   Normal RAG query response time (p95): < 1.5 seconds.
    -   Selected-Text RAG query response time (p95): < 1.0 second (due to bypass of vector search).
    -   User profile load time (p95): < 200ms.
-   **Hallucination target**: < 2% (measured by manual review of sample responses against book content for RAG, and selected text for selected-text mode).
-   **Correctness metric**: > 90% of answers are deemed "accurate and relevant" by human evaluators.
-   **UX smoothness (highlight-to-chat)**: Appearance of "Ask AI about this" button and initial chatbot response within 300ms after text highlight.

## Risks & Mitigation

-   **Qdrant rate limits**:
    *   **Risk**: Exceeding free tier limits or general API rate limits.
    *   **Mitigation**: Implement exponential backoff and retry logic in `qdrant_tool.py`. Monitor Qdrant usage. Consider upgrading Qdrant plan if needed.
-   **Gemini response drift**:
    *   **Risk**: Model behavior changes over time, leading to inconsistent answers or increased hallucinations.
    *   **Mitigation**: Implement regression tests for chatbot responses with known questions/contexts. Regularly review sample outputs. Pin specific model versions if available.
-   **Streaming disconnects**:
    *   **Risk**: Frontend-backend streaming connection drops, leading to incomplete responses.
    *   **Mitigation**: Implement robust retry mechanisms and error handling on both frontend (ChatKit) and backend (FastAPI). Display clear user messages for connection issues.
-   **CORS issues**:
    *   **Risk**: Cross-Origin Resource Sharing misconfiguration preventing frontend-backend communication.
    *   **Mitigation**: Thorough testing of CORS settings in development and production. Use `FRONTEND_ORIGIN` environment variable for flexible configuration.
-   **Performance bottlenecks in Docusaurus**:
    *   **Risk**: Embedding the ChatKit component degrades Docusaurus page load performance.
    *   **Mitigation**: Lazy load the ChatKit component. Optimize ChatKit bundle size. Minimize re-renders. Performance testing on typical Docusaurus pages.