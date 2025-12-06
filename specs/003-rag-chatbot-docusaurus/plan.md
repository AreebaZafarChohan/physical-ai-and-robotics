# Implementation Plan: RAG Chatbot for Docusaurus Book

**Feature Branch**: `001-rag-chatbot-docusaurus`
**Created**: December 6, 2025
**Status**: Draft

## Technical Context

This plan outlines the architectural design and implementation strategy for integrating a RAG Chatbot into a Docusaurus-based book website. The chatbot will enable users to query book content, broader course materials, and contextually-selected text, providing intelligent and cited answers. The solution leverages OpenAI models, FastAPI, Neon Postgres, and Qdrant.

### Core Components and Technologies

*   **Frontend**: Docusaurus for the website, with a custom floating chatbot widget. OpenAI Agents + ChatKit SDK for managing conversational flows and UI interactions.
*   **Backend**: FastAPI for the API layer, handling embedding, querying, asking, selected text processing, and logging.
*   **Database**: Neon Serverless Postgres for metadata, chat sessions, messages, feedback, and book metadata storage.
*   **Vector Database**: Qdrant Cloud Free Tier for storing OpenAI embeddings and performing vector search.
*   **RAG Pipeline**: Utilizes OpenAI embeddings and models for retrieval and generation.
*   **Languages**: Python for backend (FastAPI), JavaScript/TypeScript for frontend (Docusaurus).

### Key Integrations

*   **Docusaurus**: Custom React component for the chatbot widget, theme synchronization, and text selection events.
*   **OpenAI**: Embedding API for text chunk vectorization, and LLM API for answer generation.
*   **ChatKit SDK**: Frontend and backend integration for conversation state management.
*   **Neon Postgres**: Database connectivity and ORM integration for data persistence.
*   **Qdrant Cloud**: Vector database client for embedding storage and retrieval.

### Clarifications from Spec

*   **Compliance**: Data handling must comply with GDPR and CCPA regulations.
*   **Data Volume**: Estimated book/course content up to 10GB; approximately 1000 messages/sessions per day.
*   **Error/Empty/Loading UX**: Display distinct, themed loading spinners, error messages with retry options, and "no results" messages.
*   **Observability**: Collect query response time, RAG accuracy (if measurable), API usage, and user engagement metrics; monitor via dedicated dashboard with alerts.
*   **Supplementary Document Formats**: Primarily Markdown and PDF, with potential for plain text.

## Constitution Check

### Core Principles Alignment

*   **Quality**: The plan aims for high-quality, accurate chatbot responses and a smooth user experience.
*   **Testability**: Each component and integration point will be designed for independent testability.
*   **Scalability**: The chosen serverless and cloud-native components (Neon, Qdrant) inherently support scalability.
*   **Security**: JWT for authentication and compliance with GDPR/CCPA are explicitly planned.
*   **User Focus**: Dual modes, selected text answering, and multilingual support directly address user needs.

### Implementation Stack Validation

The proposed technologies align perfectly with the "Chatbot Integration Stack (Future)" section in the Constitution.
*   **Backend**: RAG Chatbot with FastAPI backend - **Aligned**.
*   **Database**: Neon Serverless Postgres for storing user data and session information - **Aligned**.
*   **Vector Store**: Qdrant Cloud for semantic search on textbook content - **Aligned**.
*   **Agent SDK**: OpenAI Agents / ChatKit SDK for building agent skills - **Aligned**.
*   **Features**: The integrated chatbot will answer user queries, personalize content, and manage the language toggle (English/Urdu) - **Aligned** (with Arabic added from spec).

### Gates

*   **Gate 1: Specification Complete**: PASSED. The `spec.md` is complete and clarified.
*   **Gate 2: Technical Context Clear**: PASSED. All identified `NEEDS CLARIFICATION` markers have been addressed in the previous phase.
*   **Gate 3: Constitution Alignment**: PASSED. The plan adheres to the core principles and leverages the defined implementation stack.

## Phase 0: Outline & Research

Based on the clarified spec, no further `NEEDS CLARIFICATION` markers were identified in the technical context that require additional research *before* proceeding to design. All key technology choices and integration points are defined in the spec and aligned with the constitution. Therefore, a separate `research.md` is not strictly necessary at this stage.

## Phase 1: Design & Contracts

### Data Model (`data-model.md`)

This section will define the detailed schema for each entity identified in the feature specification, to be implemented in Neon Postgres.

*   **`User`**:
    *   `id` (PK, UUID)
    *   `email` (string, optional, unique)
    *   `preferences` (JSONB, stores language, theme, etc.)
    *   `created_at` (timestamp with timezone)
    *   `last_login_at` (timestamp with timezone)
*   **`ChatSession`**:
    *   `id` (PK, UUID)
    *   `user_id` (FK to `User.id`, nullable for unauthenticated users)
    *   `started_at` (timestamp with timezone)
    *   `last_activity_at` (timestamp with timezone)
    *   `mode` (enum: 'book', 'course')
    *   `status` (enum: 'active', 'ended', 'archived', 'pending', 'error'): Current status of the chat session.
*   **`Message`**:
    *   `id` (PK, UUID)
    *   `session_id` (FK to `ChatSession.id`)
    *   `sender` (enum: 'user', 'bot')
    *   `content` (text)
    *   `timestamp` (timestamp with timezone)
    *   `citations` (JSONB, array of objects with `text` and `link` properties)
    *   `language` (string, e.g., 'en', 'ur', 'ar')
*   **`Feedback`**:
    *   `id` (PK, UUID)
    *   `message_id` (FK to `Message.id`, can be nullable if feedback is on overall session)
    *   `user_id` (FK to `User.id`, nullable)
    *   `type` (enum: 'positive', 'negative', 'comment')
    *   `comment` (text, optional)
    *   `created_at` (timestamp with timezone)
*   **`BookMetadata`**:
    *   `id` (PK, UUID or natural key from source_link + paragraph_id)
    *   `chapter` (string)
    *   `source_link` (string, URL or path to Docusaurus page)
    *   `paragraph_id` (string, unique ID within source_link)
    *   `text_content_hash` (string, for change detection)
    *   `created_at` (timestamp with timezone)
    *   `updated_at` (timestamp with timezone)

### API Contracts (`contracts/`)

This will involve defining the OpenAPI (Swagger) specifications for the FastAPI backend routes. Each endpoint will have its request and response models clearly defined.

*   **`POST /embed`**:
    *   **Request Model**: `EmbedRequest` (text: str, chapter: str, source_link: str, paragraph_id: str)
    *   **Response Model**: `EmbedResponse` (status: str, vector_id: str)
*   **`POST /query`**:
    *   **Request Model**: `QueryRequest` (query: str, mode: Literal["book", "course"], user_id: Optional[str])
    *   **Response Model**: `QueryResponse` (relevant_chunks: List[Chunk], citation_links: List[str]) where `Chunk` has `text` and `metadata`
*   **`POST /ask`**:
    *   **Request Model**: `AskRequest` (query: str, context: List[str], chat_history: List[Message], user_id: Optional[str], language: Literal["en", "ur", "ar"], mode: Literal["book", "course"])
    *   **Response Model**: `AskResponse` (answer: str, citation_links: List[str])
*   **`POST /selected_text`**:
    *   **Request Model**: `SelectedTextRequest` (query: str, selected_text: str, user_id: Optional[str], language: Literal["en", "ur", "ar"])
    *   **Response Model**: `SelectedTextResponse` (answer: str)
*   **`POST /log`**:
    *   **Request Model**: `LogRequest` (session_id: str, user_id: Optional[str], message_type: Literal["user", "bot"], content: str, mode: Literal["book", "course", "selected_text"], feedback: Optional[Literal["positive", "negative", "comment"]])
    *   **Response Model**: `LogResponse` (status: str, entry_id: str)

### Quickstart Guide (`quickstart.md`)

This will outline the high-level steps for setting up and running the chatbot for development.

1.  **Clone the repository**: `git clone [repo-url]`
2.  **Frontend Setup**:
    *   Navigate to `frontend/` directory.
    *   Install dependencies: `npm install`
    *   Start Docusaurus in development mode: `npm start`
3.  **Backend Setup**:
    *   Navigate to `backend/` directory.
    *   Set up Python virtual environment: `python -m venv venv && source venv/bin/activate`
    *   Install dependencies: `pip install -r requirements.txt`
    *   Configure environment variables (e.g., `OPENAI_API_KEY`, `NEON_DB_URL`, `QDRANT_API_KEY`).
    *   Start FastAPI server: `uvicorn main:app --reload`
4.  **Database Initialization**:
    *   Run database migrations for Neon Postgres.
    *   Ingest initial book content using the `/embed` endpoint (details in Data Pipeline).
5.  **Access Chatbot**: Open the Docusaurus website in your browser, the chatbot widget should be visible.

### Agent Context Update

This will update the Gemini agent context with the newly defined entities and API routes for future interactions.

---
[End of Plan Content]