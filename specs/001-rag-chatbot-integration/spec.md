# Feature Specification: AI-Native RAG Chatbot

**Feature Branch**: `001-rag-chatbot-integration`  
**Created**: 2025-12-10  
**Status**: Draft  
**Input**: User description: "Title: AI-Native RAG Chatbot for Physical AI & Humanoid Robotics Book Purpose: We need to build an integrated Retrieval-Augmented Generation (RAG) chatbot that runs inside the Docusaurus-based Physical AI & Humanoid Robotics textbook. This chatbot will fetch text embeddings from Qdrant Cloud, reference the Neon Postgres user session, and answer questions using the OpenAI Agents SDK (ChatKit) with Gemini models. The chatbot must support answering from full-book context AND user-selected text context. This is Step 2 of the hackathon: integrating the RAG chatbot after ingestion is complete. Target User: Panaversity Hackathon participants who have already finished Step 1 (ingestion pipeline) and now want to add an AI-native chatbot directly inside the book. Core Requirements: - Use **OpenAI Agents SDK** with **Gemini** models (using the OpenAI-style wrapper). - Use **ChatKit SDK** for frontend embedding. - Use **FastAPI** for backend API. - Use **Neon Serverless Postgres** for storing: - User profiles (hardware/software background) - User personalization settings - Optional: chapter-level preferences (Urdu translation, personalized difficulty) - Use **Qdrant Cloud** Free Tier to retrieve top-k embeddings. - Chatbot must support **two modes**: 1. **Normal RAG Mode** → answer from entire book vector store. 2. **Selected-Text RAG Mode** → only answer using the text the user highlighted on the page. - Provide **streamed responses**. - MUST run inside Docusaurus as an embedded component. Functional Requirements: 1. **RAG Pipeline** - Accepts user query from ChatKit frontend. - Retrieves top-k relevant chunks from Qdrant Cloud. - Injects retrieved context into the agent prompt. - Model: `gemini-2.0-flash` via OpenAIChatCompletionsModel. - Citation-style answer (list of matched chunk IDs). 2. **Selected Text Mode** - When user highlights text → frontend sends that text block. - Backend bypasses vector search. - Model answers ONLY using that selected text. - Prevent hallucination by strong “only answer using provided text” instruction. 3. **User Personalization (Bonus Points)** - Fetch user background profile from Neon DB. - Personalize answer style: - Beginner friendly - Hardware-focused - ROS 2 focused - Robotics engineering - Modify system prompt accordingly. 4. **Neon Database Integration** - FastAPI endpoint for: - `/auth/signup` - `/auth/signin` - `/user/preferences` - `/chat/query` - Store: - user_id - background summary - preferred difficulty - preferred language (English/Urdu) - personalization enabled? (true/false) 5. **OpenAI Agents / Tools** - Use the Agents SDK with Gemini model. - Define tools: - **qdrant_search_tool** (custom function tool) - **selected_text_tool** (bypass mode) - Optional: **Neon tool** to fetch user preferences - All tools must be implemented using @function_tool decorator. 6. **FastAPI Backend** - Endpoints: 1. `/api/chat` → main RAG query 2. `/api/chat/selected` → selected-text mode 3. `/api/user/profile` 4. `/api/user/update` - Return JSON: - response - citations - chunks_used - personalization_applied 7. **Frontend Integration (ChatKit)** - A floating chatbot UI inside the book. - Markdown rendering of model output. - Highlight-to-chat interaction: - When user highlights text → “Ask AI about this” button appears. - Sends selected text to `/api/chat/selected`. 8. **Security** - No API keys on frontend. - All keys stored in `.env` (FastAPI backend). - Qdrant read-only API key recommended. Success Criteria: - Chatbot successfully returns accurate answers from Qdrant. - Selected-text mode works and uses ONLY the provided text. - Responses appear inside the book UI. - Model uses Gemini through OpenAI Agents SDK. - Backend & frontend code run without modification (except .env). - Neon database correctly stores/retrieves personalization data. - Fast, low-latency interaction (<1.5s typical). - Clean logging system for debugging. Constraints: - Language: Python 3.10+ for backend - Framework: FastAPI - Only allowed backends: Gemini + OpenAI Agents SDK, Qdrant, Neon - Must remain deployable on Vercel / Render / Railway - Must support CORS for Docusaurus frontend - No LangChain or LlamaIndex (raw RAG only) - Chat model: `gemini-2.0-flash` Not In Scope: - Ingestion pipeline (done in Step 1) - Multi-turn memory beyond personalization - Auth UI (Better-auth only, backend) - Robot-control or ROS integration Deliverables: - Full FastAPI backend folder (router, tools, model provider, utils) - Complete ChatKit frontend component for Docusaurus - Neon schema SQL - Qdrant retrieval code - Python function tools for Agents SDK - System/instruction prompts - Documentation section added into the book (README inside repo) - Final `/sp.plan`, `/sp.tasks`, `/sp.implement` flows working in Claude Code Qdrant Collection Name: ai_robotics_book"

## User Scenarios & Testing

### User Story 1 - Ask AI about the book (Normal RAG) (Priority: P1)

A user asks a question about the book content, and the chatbot provides an answer by retrieving relevant information from the entire book's vector store.

**Why this priority**: This is the primary function of the RAG chatbot, providing comprehensive answers from the book.

**Independent Test**: A user can type a question into the chatbot, and it returns a coherent answer with citations.

**Acceptance Scenarios**:

1.  **Given** the chatbot is active and the user types a general question about robotics, **When** the user submits the query, **Then** the chatbot returns an answer with citations from the book content.
2.  **Given** the chatbot is active, **When** the user asks a question that is not covered in the book, **Then** the chatbot indicates that it cannot find relevant information in the book.

### User Story 2 - Ask AI about selected text (Selected-Text RAG) (Priority: P1)

A user highlights a section of text in the book and asks the chatbot a question specifically about that selected text. The chatbot answers using only the provided context.

**Why this priority**: This provides a focused, contextual questioning ability, crucial for detailed understanding and avoiding hallucination.

**Independent Test**: A user can highlight text, click "Ask AI about this," and receive an answer strictly based on the highlighted content.

**Acceptance Scenarios**:

1.  **Given** a user has highlighted text on a page, **When** the user clicks "Ask AI about this" and then asks a question, **Then** the chatbot provides an answer using *only* the highlighted text as context.
2.  **Given** a user has highlighted text, **When** the user asks a question that cannot be answered by the highlighted text alone, **Then** the chatbot explicitly states that it cannot answer based on the provided text.

### User Story 3 - Personalized Chatbot Interaction (Priority: P2)

A user, who has configured their preferences (e.g., beginner-friendly, hardware-focused) in the Neon Postgres database, receives answers tailored to their specified style.

**Why this priority**: Enhances user experience and learning by adapting to individual needs, a bonus feature.

**Independent Test**: A user can set a personalization preference, ask a question, and observe that the answer reflects their chosen style.

**Acceptance Scenarios**:

1.  **Given** a user has set a "beginner-friendly" personalization preference, **When** the user asks a technical question, **Then** the chatbot explains the concept in simpler terms.
2.  **Given** a user has set a "hardware-focused" preference, **When** the user asks a general question about a robot, **Then** the chatbot's answer emphasizes hardware components and their functions.

### User Story 4 - User Profile Management (Priority: P2)

Users can create an account, sign in, and update their personalization preferences which are stored in Neon Serverless Postgres.

**Why this priority**: Essential for enabling personalized experiences.

**Independent Test**: A user can sign up, log in, update their profile, and verify the changes persist.

**Acceptance Scenarios**:

1.  **Given** a new user, **When** they sign up, **Then** a new user profile is created in the Neon database.
2.  **Given** an existing user, **When** they update their preferred language to Urdu, **Then** their preference is stored and retrieved correctly.

### Edge Cases

-   What happens when the Qdrant Cloud returns no relevant chunks for a query? (Chatbot should indicate no information found).
-   How does the system handle very long highlighted text in "Selected-Text RAG Mode"? (Backend should process efficiently or truncate appropriately).
-   What if a user's personalization settings are incomplete or invalid? (System should default to a neutral style).
-   What if the Neon Postgres database is unreachable? (Graceful degradation, e.g., chatbot functions without personalization).
-   What if API keys are missing or invalid in the backend? (Appropriate error logging and user-friendly error messages).

## Requirements

### Functional Requirements

-   **FR-001**: The chatbot system MUST accept user queries from the ChatKit frontend.
-   **FR-002**: The RAG pipeline MUST retrieve top-k relevant text chunks from Qdrant Cloud based on the user's query.
-   **FR-003**: The system MUST inject the retrieved context into the agent prompt using `gemini-2.0-flash` via `OpenAIChatCompletionsModel`.
-   **FR-004**: The chatbot MUST provide citation-style answers, listing matched chunk IDs.
-   **FR-005**: In "Selected-Text RAG Mode", the backend MUST bypass vector search and answer *only* using the user-highlighted text.
-   **FR-006**: The system MUST enforce a strict instruction to the model to prevent hallucination in "Selected-Text RAG Mode" (only answer using provided text).
-   **FR-007**: The system MUST fetch user background profiles from Neon DB for personalization.
-   **FR-008**: The system MUST personalize answer style (e.g., beginner-friendly, hardware-focused) by modifying the system prompt based on user preferences.
-   **FR-009**: The FastAPI backend MUST provide endpoints for user authentication (`/auth/signup`, `/auth/signin`) using JWT for session management, and user preferences (`/user/preferences`).
-   **FR-010**: The FastAPI backend MUST provide endpoints for chat queries (`/api/chat`, `/api/chat/selected`, `/chat/query`).
-   **FR-011**: The Neon Postgres database MUST store `user_id`, `background_summary`, `preferred_difficulty`, `preferred_language` (English/Urdu), and `personalization_enabled` status.
-   **FR-012**: The system MUST implement `qdrant_search_tool` (custom function tool) using the OpenAI Agents SDK.
-   **FR-013**: The system MUST implement `selected_text_tool` (bypass mode) using the OpenAI Agents SDK.
-   **FR-014**: The system SHOULD implement a `Neon tool` to fetch user preferences using the OpenAI Agents SDK.
-   **FR-015**: All tools MUST be implemented using the `@function_tool` decorator.
-   **FR-016**: The FastAPI backend MUST return JSON responses containing `response`, `citations`, `chunks_used`, and `personalization_applied`.
-   **FR-017**: The chatbot MUST run inside Docusaurus as an embedded component using ChatKit for frontend embedding.
-   **FR-018**: The ChatKit frontend MUST provide a floating chatbot UI.
-   **FR-019**: The ChatKit frontend MUST render model output in Markdown.
-   **FR-020**: The ChatKit frontend MUST display an "Ask AI about this" button when a user highlights text, sending the selected text to `/api/chat/selected`.
-   **FR-021**: All API keys MUST be stored in `.env` files on the FastAPI backend and NOT on the frontend.
-   **FR-022**: A Qdrant read-only API key is recommended.
-   **FR-023**: The system MUST support streamed responses from the chatbot.
-   **FR-024**: The FastAPI backend MUST support CORS for the Docusaurus frontend.
-   **FR-025**: The ChatKit frontend MUST provide clear UI feedback for API loading states (e.g., spinners), errors (e.g., toast messages), and empty responses (e.g., informative messages).

### Key Entities

-   **User Profile**: Represents a user's background and preferences.
    -   `user_id` (unique identifier, UUID)
    -   `background_summary` (text describing hardware/software background)
    -   `preferred_difficulty` (e.g., "Beginner", "Intermediate", "Advanced", "Expert")
    -   `preferred_language` (e.g., "English", "Urdu")
    -   `personalization_enabled` (boolean)
-   **Chat Query**: Represents a user's question to the chatbot.
    -   `query_text`
    -   `selected_text` (optional, for selected-text mode)
    -   `user_id` (for personalization)
-   **Chat Response**: Represents the chatbot's answer.
    -   `response_text`
    -   `citations` (list of chunk IDs)
    -   `chunks_used` (number of chunks used)
    -   `personalization_applied` (boolean)

## Success Criteria

### Measurable Outcomes

-   **SC-001**: The chatbot successfully returns accurate answers from Qdrant with relevant citations in Normal RAG Mode, verifiable by user testing.
-   **SC-002**: The Selected-Text RAG Mode accurately uses *only* the provided text for answers, achieving a 100% adherence rate to the selected text as context.
-   **SC-003**: Chatbot responses are displayed within the Docusaurus book UI without layout or functional issues.
-   **SC-004**: The system utilizes Gemini models through the OpenAI Agents SDK for all AI interactions.
-   **SC-005**: The backend and frontend code run without requiring manual modifications (except for `.env` file setup) upon deployment.
-   **SC-006**: The Neon database correctly stores and retrieves user personalization data, verifiable through database inspection and user preference application.
-   **SC-007**: The chatbot provides fast, low-latency interaction, with typical response times under 1.5 seconds for 90% of queries.
-   **SC-008**: The system provides clear and comprehensive logging for debugging purposes, easily accessible in development and production environments.

## Clarifications

### Session 2025-12-10

- Q: What authentication/authorization mechanism should the FastAPI backend implement for user sessions? → A: JWT (JSON Web Tokens)
- Q: How should the `user_id` be generated and what are its uniqueness guarantees? → A: UUID (Universally Unique Identifier)
- Q: How should the ChatKit frontend visibly indicate loading states, display API errors, and handle empty responses from the backend? → A: Clear UI Feedback (spinners for loading, toasts/alerts for errors, informative messages for empty)
- Q: What are the supported values for `preferred_difficulty`? → A: Standardized Levels: "Beginner", "Intermediate", "Advanced", "Expert"
- Q: What API versioning strategy should be adopted for the FastAPI backend? → A: URL Versioning (e.g., `/api/v1/chat`)