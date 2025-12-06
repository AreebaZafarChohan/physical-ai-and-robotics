# Feature Specification: RAG Chatbot for Docusaurus Book

**Feature Branch**: `001-rag-chatbot-docusaurus`  
**Created**: December 6, 2025  
**Status**: Draft  
**Input**: User description: "You are an expert AI Product Architect. I want a complete and detailed specification for building a **RAG Chatbot embedded inside a Docusaurus Book website**, using the following stack: • OpenAI Agents + ChatKit SDK • FastAPI backend • Neon Serverless Postgres (for metadata + conversation logs) • Qdrant Cloud Free Tier (vector DB for embeddings) • Docusaurus frontend + floating chatbot widget • OpenAI embeddings + RAG pipeline • User-selected text → answer only from selection mode =========================== SYSTEM BEHAVIOR =========================== The chatbot must be able to: 1. **Answer questions about the book's content** stored as Markdown files in the Docusaurus project. 2. **Perform Retrieval-Augmented Generation (RAG)** using: - Embeddings stored in Qdrant - Metadata stored in Neon Postgres - Vector search (semantic + keyword hybrid) 3. **Provide dual modes:** - "Ask the Book" → search only the uploaded markdown chapters - "Ask the Course" → search all course documents (book + extra notes) 4. **User-Selected Text Mode**: - If the user highlights/selects text inside the book page → bot answers ONLY from that text (no other retrieval). 5. **Frontend Chat Widget Features**: - Floating bottom-right widget - Dark/light theme sync with Docusaurus theme - Streaming token output - Source citations + clickable anchors - “Answer based on selected text” button - Language-switch: English, Urdu, Arabic 6. **Backend Features (FastAPI)**: - API routes: • /embed • /query • /ask • /selected_text • /log (store into Neon) - JWT optional user auth - Rate limiting + safety mode - Integration with ChatKit for conversation state 7. **Database Layer**: - Neon Postgres schema: • users • chat_sessions • messages • feedback • book_metadata 8. **Vector DB Layer (Qdrant)**: - Collection name: book_chapters - Metadata: chapter, source_link, paragraph_id =========================== DELIVERABLES REQUIRED =========================== Your output should contain: 1. **Title** 2. **Summary** 3. **Problem Definition** 4. **User Types** 5. **Functional Requirements** 6. **Non-functional Requirements** 7. **Full System Architecture (Frontend + Backend + DB + Vector DB)** 8. **Data Pipeline & Embedding Pipeline** 9. **UI/UX Flow (Docusaurus Widget)** 10. **Backend API Routes + Models** 11. **Integration Plan (end-to-end)** 12. **Error Handling + Safety** 13. **Risks, Assumptions, Out of Scope** 14. **Acceptance Criteria** Follow a formal product-spec format."

## Title
RAG Chatbot for Docusaurus Book Website

## Summary
This specification outlines the development of a Retrieval-Augmented Generation (RAG) chatbot seamlessly integrated into a Docusaurus-based book website. The chatbot will enable users to interactively query the book's content, and optionally broader course materials, receiving accurate and contextually relevant answers. A key feature includes the ability to answer questions based solely on user-selected text, enhancing focused learning. The solution will leverage a modern stack for robustness, scalability, and an intuitive user experience.

## Problem Definition
Users of educational or technical Docusaurus-based book websites often struggle to quickly find specific information or get answers to their questions directly within the content. Navigating large documents or multiple resources to synthesize answers is time-consuming and inefficient. Existing search functionalities may lack semantic understanding, leading to irrelevant results. There is a need for an intelligent, interactive assistant that can provide precise, context-aware answers from the book's content and related course materials, improving user engagement and learning efficiency.

## User Types

*   **Learner/Reader**: An individual consuming the Docusaurus book content, seeking quick answers, clarifications, or deeper insights into the material.
*   **Course Administrator/Educator**: Manages and updates the book content and potentially additional course materials, and reviews chatbot interactions for quality and feedback.
*   **Developer/Maintainer**: Responsible for the technical infrastructure and evolution of the Docusaurus website and chatbot.

## User Scenarios & Testing

### User Story 1 - Ask the Book (Priority: P1)

As a Learner, I want to ask questions about the book's content using the chatbot, so that I can quickly get answers directly related to the material I am studying.

**Why this priority**: This is the core functionality and primary value proposition of the chatbot, directly addressing the main problem of information retrieval from the book.

**Independent Test**: Can be fully tested by asking a question about a known chapter in the book and verifying the chatbot provides a relevant answer from that chapter.

**Acceptance Scenarios**:

1.  **Given** I am on any page of the Docusaurus book and the chatbot widget is visible, **When** I click the chatbot icon to open it, **Then** the chat interface appears.
2.  **Given** the chat interface is open and set to "Ask the Book" mode, **When** I type a question related to the book's content (e.g., "What are the main components of ROS2?"), **Then** the chatbot displays a streaming response relevant to the book's content.
3.  **Given** the chatbot has provided an answer, **When** I review the response, **Then** source citations with clickable anchors to the original book sections are provided.

### User Story 2 - Ask the Course (Priority: P2)

As a Learner, I want to ask questions that can draw from all available course documents (book + extra notes), so that I can get comprehensive answers beyond just the current book's scope.

**Why this priority**: This expands the utility of the chatbot to a broader knowledge base, offering more complete answers for users engaging with the entire course.

**Independent Test**: Can be tested by switching to "Ask the Course" mode and asking a question that requires information from both the book and supplementary course notes, verifying a consolidated answer.

**Acceptance Scenarios**:

1.  **Given** the chat interface is open, **When** I select the "Ask the Course" mode, **Then** the chatbot acknowledges the mode change.
2.  **Given** the chatbot is in "Ask the Course" mode, **When** I type a question that may span book chapters and external course notes, **Then** the chatbot provides a response that integrates information from both sources.
3.  **Given** the chatbot has provided an answer, **When** I review the response, **Then** source citations correctly link to both book content and supplementary course documents.

### User Story 3 - Answer from Selected Text (Priority: P1)

As a Learner, when I highlight text within the Docusaurus book, I want the chatbot to answer questions *only* based on that selected text, so that I can get highly focused explanations without external context.

**Why this priority**: This provides a powerful, highly contextualized learning tool that directly supports deep engagement with specific content sections.

**Independent Test**: Can be tested by highlighting a paragraph in the book, activating the "Answer based on selected text" feature, and asking a question. The response should strictly adhere to the highlighted content.

**Acceptance Scenarios**:

1.  **Given** I have highlighted a section of text on a Docusaurus book page, **When** I click a "Answer based on selected text" button or equivalent action, **Then** the chatbot interface opens/focuses with the selected text as context.
2.  **Given** the chatbot is operating with user-selected text as context, **When** I ask a question, **Then** the chatbot's response is generated *exclusively* from the content within the previously selected text.
3.  **Given** the chatbot has provided an answer based on selected text, **When** I review the response, **Then** no citations outside the selected text are provided, reinforcing the focused nature of the answer.

### User Story 4 - Multi-language Support (Priority: P2)

As a global Learner, I want to interact with the chatbot in English, Urdu, or Arabic, so that I can learn in my preferred language.

**Why this priority**: This significantly expands the accessibility and usability of the chatbot for a diverse audience.

**Independent Test**: Can be tested by changing the Docusaurus site's language, then interacting with the chatbot and verifying its interface and responses are in the selected language.

**Acceptance Scenarios**:

1.  **Given** the Docusaurus website's language is set to English, Urdu, or Arabic, **When** I open the chatbot widget, **Then** the chatbot interface elements (e.g., input placeholder, mode labels) are displayed in the corresponding language.
2.  **Given** the chatbot is open and the website's language is set to Urdu, **When** I ask a question in Urdu, **Then** the chatbot responds in Urdu.
3.  **Given** the chatbot is open and the website's language is set to Arabic, **When** I ask a question in Arabic, **Then** the chatbot responds in Arabic.

### Edge Cases

*   **Empty Query**: What happens when a user submits an empty query? The chatbot should prompt the user for input.
*   **Irrelevant Query**: How does the system handle questions completely outside the scope of the book/course content? The chatbot should indicate it cannot answer and suggest rephrasing or switching modes.
*   **No Selection for "Selected Text" Mode**: What if the user attempts to use "Answer based on selected text" without any text highlighted? The chatbot should prompt the user to select text.
*   **Rate Limit Exceeded**: How does the system inform the user if they exceed query rate limits? An informative message should be displayed.
*   **API Downtime**: How does the system gracefully handle temporary unavailability of backend services or external APIs (e.g., OpenAI, Qdrant)? A fallback message or retry mechanism should be in place.

## Requirements

### Functional Requirements

*   **FR-001**: The chatbot MUST be able to answer questions based on the Markdown content of the Docusaurus book.
*   **FR-002**: The chatbot MUST support two distinct retrieval modes: "Ask the Book" (limited to book content) and "Ask the Course" (book content + supplementary materials).
*   **FR-003**: The chatbot MUST integrate a "User-Selected Text Mode" where answers are exclusively derived from highlighted text on the page.
*   **FR-004**: The chatbot MUST provide streaming output for responses to enhance user experience.
*   **FR-005**: The chatbot MUST include source citations with clickable anchors linking back to the relevant sections in the Docusaurus book or course materials.
*   **FR-006**: The chatbot widget MUST be a floating component positioned at the bottom-right of the Docusaurus website.
*   **FR-007**: The chatbot widget MUST synchronize its theme (dark/light) with the Docusaurus website's active theme.
*   **FR-008**: The chatbot MUST offer language-switching capabilities for its interface and responses, supporting English, Urdu, and Arabic.
*   **FR-009**: The system MUST log all chat interactions, including user queries, chatbot responses, and chosen modes, to a persistent store.
*   **FR-010**: The system MUST implement rate limiting for API requests to prevent abuse and ensure stability.
*   **FR-011**: The system MUST integrate with ChatKit for managing conversation state.
*   **FR-012**: The system MUST support optional JWT-based user authentication for backend API routes.

### Non-functional Requirements

*   **NFR-001 - Performance**: 95% of chatbot responses in "Ask the Book" mode MUST be delivered within 3 seconds.
*   **NFR-002 - Scalability**: The backend API MUST be capable of handling 100 concurrent users without significant degradation in response time.
*   **NFR-003 - Reliability**: The chatbot service MUST maintain 99.9% uptime.
*   **NFR-004 - Security**: User authentication, if enabled, MUST secure all sensitive API endpoints, and data in transit and at rest MUST be encrypted. All data handling MUST comply with GDPR and CCPA regulations.
*   **NFR-005 - User Experience**: The chatbot interface MUST be intuitive and responsive across various screen sizes and devices.
*   **NFR-006 - Data Retention**: Conversation logs and feedback MUST be retained for a minimum of 1 year for analysis and improvement.

## Clarifications

### Session 2025-12-06

- Q: Are there any specific data privacy or other regulatory compliance requirements (e.g., GDPR, CCPA, HIPAA) for user data logging and feedback storage? → A: GDPR and CCPA

### Key Entities

*   **User**: Represents an individual interacting with the chatbot.
*   **Chat Session**: A continuous conversation thread between a user and the chatbot.
*   **Message**: An individual turn in a chat session, either from the user or the chatbot.
*   **Feedback**: User-provided assessment or comments on chatbot responses.
*   **Book Metadata**: Information about the book's chapters, sections, and their corresponding file paths or unique identifiers.
*   **Embedding**: Vector representations of text content for semantic search.
*   **Document Chunk**: Small, retrievable segments of the book or course content.

## Full System Architecture (Frontend + Backend + DB + Vector DB)

### Frontend (Docusaurus)
*   **Docusaurus Website**: Serves as the primary platform for the book content.
*   **Chatbot Widget**: A floating UI component embedded within the Docusaurus site. This widget will handle user input, display chatbot responses, and manage UI state (e.g., theme, language).
*   **OpenAI Agents + ChatKit SDK**: Integrated into the frontend for managing conversational flows and potentially handling basic UI interactions or agent-like behavior.

### Backend (FastAPI)
*   **FastAPI Application**: A high-performance Python web framework serving as the chatbot's API layer.
*   **API Routes**: Dedicated endpoints for embedding generation, querying, answering questions, handling selected text queries, and logging interactions.
*   **JWT Optional User Auth**: Middleware for securing API endpoints where user authentication is desired.
*   **Rate Limiting + Safety Mode**: Mechanisms to control request frequency and filter potentially harmful inputs/outputs.
*   **ChatKit Integration**: Used on the backend to manage and persist complex conversation states.

### Database (Neon Serverless Postgres)
*   **Neon Postgres**: A scalable, serverless PostgreSQL database.
*   **Schema**:
    *   `users`: Stores user authentication details (if JWT is enabled), and preferences.
    *   `chat_sessions`: Records metadata for each conversation session.
    *   `messages`: Stores the history of messages within each chat session.
    *   `feedback`: Stores user feedback on chatbot responses.
    *   `book_metadata`: Stores structured information about the book's chapters, sections, source links, and paragraph IDs, aligning with the vector DB's metadata.

### Vector DB (Qdrant Cloud Free Tier)
*   **Qdrant Cloud Free Tier**: A vector similarity search engine.
*   **Collection**: `book_chapters`
*   **Metadata per vector**: `chapter`, `source_link`, `paragraph_id` (used for retrieval and citation linking).
*   **OpenAI Embeddings**: Stores vector representations of the book and course content.

## Data Pipeline & Embedding Pipeline

### Data Ingestion
1.  **Source Data**: Markdown files from the Docusaurus project (book chapters) and supplementary course documents (primarily Markdown and PDF, with potential for plain text).
2.  **Text Extraction & Chunking**:
    *   Markdown files are parsed to extract raw text content.
    *   Content is segmented into manageable "chunks" (e.g., paragraphs, sections) suitable for embedding. Overlapping chunks may be used to preserve context.
3.  **Metadata Extraction**: For each chunk, relevant metadata is extracted, including:
    *   `chapter`: The chapter title or identifier.
    *   `source_link`: The URL or path to the original markdown file/section in Docusaurus.
    *   `paragraph_id`: A unique identifier for the specific text chunk within its source document.

### Embedding Generation
1.  **OpenAI Embeddings**: Each text chunk is sent to the OpenAI embeddings API to generate a high-dimensional vector representation.
2.  **Vector Storage**: The generated embeddings, along with their extracted metadata, are stored in the Qdrant `book_chapters` collection.

## UI/UX Flow (Docusaurus Widget)

1.  **Widget Activation**: A floating icon (e.g., chat bubble) is displayed in the bottom-right corner of all Docusaurus pages.
2.  **Chat Interface Toggle**: Clicking the icon toggles the visibility of the chat interface, which appears as an overlay or sidebar.
3.  **Theme Sync**: The chat interface automatically detects and applies the Docusaurus website's current theme (dark/light).
4.  **Mode Selection**: Users can select between "Ask the Book" and "Ask the Course" modes via clear UI toggles or buttons.
5.  **Text Input**: A text input field allows users to type their questions.
6.  **Streaming Output**: Chatbot responses are streamed token-by-token for a dynamic user experience.
7.  **Citations**: Responses include clickable links (anchors) to the exact source sections within the Docusaurus book.
8.  **"Answer based on Selected Text"**:
    *   When a user highlights text on a Docusaurus page, a small, ephemeral UI element (e.g., a floating button or context menu option) appears.
    *   Clicking this element either opens the chatbot with the selected text pre-loaded as context or sends the selected text to the backend with the user's next query.
    *   The chatbot's response clearly indicates it is answering *only* from the selected text.
9.  **Language Switch**: A language selection option (e.g., dropdown, toggle) within the chatbot widget allows users to switch the interaction language between English, Urdu, and Arabic.

## Backend API Routes + Models

### API Routes (FastAPI)

*   **`POST /embed`**:
    *   **Purpose**: Ingest and embed new or updated book/course content.
    *   **Request**: `{"text": "...", "chapter": "...", "source_link": "...", "paragraph_id": "..."}`
    *   **Response**: `{"status": "success", "vector_id": "..."}`
    *   **Authentication**: Admin/ingestion specific JWT (if implemented).

*   **`POST /query`**:
    *   **Purpose**: Perform semantic search on the vector database.
    *   **Request**: `{"query": "user question", "mode": "book"|"course", "user_id": "optional"}`
    *   **Response**: `{"relevant_chunks": [{"text": "...", "metadata": {...}}], "citation_links": ["..."]}`
    *   **Authentication**: Optional JWT.

*   **`POST /ask`**:
    *   **Purpose**: Generate a comprehensive answer based on retrieved context.
    *   **Request**: `{"query": "user question", "context": ["retrieved chunks"], "chat_history": [...], "user_id": "optional", "language": "en"|"ur"|"ar", "mode": "book"|"course"}`
    *   **Response**: `{"answer": "streaming text", "citation_links": ["..."]}`
    *   **Authentication**: Optional JWT.

*   **`POST /selected_text`**:
    *   **Purpose**: Handle queries based on user-highlighted text.
    *   **Request**: `{"query": "user question", "selected_text": "highlighted content", "user_id": "optional", "language": "en"|"ur"|"ar"}`
    *   **Response**: `{"answer": "streaming text"}` (no citations expected here as answer is from selected text)
    *   **Authentication**: Optional JWT.

*   **`POST /log`**:
    *   **Purpose**: Log chat interactions and feedback.
    *   **Request**: `{"session_id": "...", "user_id": "optional", "message_type": "user"|"bot", "content": "...", "mode": "...", "feedback": "positive"|"negative"|"comment"}`
    *   **Response**: `{"status": "logged", "entry_id": "..."}`
    *   **Authentication**: Optional JWT.

### Backend Models (Pydantic/FastAPI)

*   **`User`**: `id`, `email` (optional), `preferences` (language, theme), `created_at`.
*   **`ChatSession`**: `id`, `user_id` (FK to User), `started_at`, `last_activity_at`, `mode` (`book`|`course`).
*   **`Message`**: `id`, `session_id` (FK to ChatSession), `sender` (`user`|`bot`), `content`, `timestamp`, `citations` (list of links), `language`.
*   **`Feedback`**: `id`, `message_id` (FK to Message), `user_id` (FK to User), `type` (`positive`|`negative`|`comment`), `comment` (optional), `created_at`.
*   **`BookMetadata`**: `id`, `chapter`, `source_link`, `paragraph_id`, `text_content` (optional, for direct retrieval).

## Integration Plan (end-to-end)

1.  **Docusaurus Integration**:
    *   Develop a Docusaurus plugin or theme component for the chatbot widget.
    *   Implement client-side logic to handle widget visibility, theme synchronization, language switching, and text selection events.
    *   Integrate ChatKit SDK for frontend conversation management.
2.  **FastAPI Backend Setup**:
    *   Set up a FastAPI project with initial API routes (`/embed`, `/query`, `/ask`, `/selected_text`, `/log`).
    *   Implement JWT authentication middleware and rate limiting.
    *   Integrate ChatKit for backend conversation state management.
3.  **Neon Postgres Configuration**:
    *   Define the PostgreSQL schema (users, chat_sessions, messages, feedback, book_metadata).
    *   Set up connection pooling and ORM (e.g., SQLAlchemy/SQLModel).
4.  **Qdrant Vector DB Configuration**:
    *   Create the `book_chapters` collection with specified metadata fields (`chapter`, `source_link`, `paragraph_id`).
    *   Configure index for vector search.
5.  **Embedding Pipeline Development**:
    *   Create a script or service to process Docusaurus Markdown files and supplementary materials.
    *   Implement text chunking, metadata extraction, and OpenAI embedding generation.
    *   Develop an ingestion endpoint (`/embed`) to populate Qdrant.
6.  **RAG Pipeline Implementation**:
    *   **Retrieval**: Implement hybrid search (semantic + keyword) in Qdrant, using the `/query` endpoint.
    *   **Augmentation**: Combine retrieved text chunks with the user's query and chat history to form a comprehensive prompt for the LLM.
    *   **Generation**: Use OpenAI LLM via the `/ask` endpoint to generate answers.
7.  **"Selected Text" Feature**:
    *   Frontend: Implement JavaScript to detect text selection and trigger the "Answer based on selected text" action, sending selected text to `/selected_text` endpoint.
    *   Backend: Modify `/selected_text` endpoint to perform generation directly from the provided `selected_text` without external retrieval.
8.  **Language Support**:
    *   Implement internationalization (i18n) for frontend chatbot UI.
    *   Backend `/ask` and `/selected_text` endpoints should take a `language` parameter and potentially use language-specific LLM prompts or models.
9.  **Deployment**:
    *   Deploy Docusaurus frontend to a suitable static hosting service.
    *   Deploy FastAPI backend as a serverless function or containerized service.
    *   Configure Neon Postgres and Qdrant connections.

## Error Handling + Safety

*   **Robust Error Logging**: Comprehensive logging for all API errors, external service failures (OpenAI, Qdrant, Neon), and unexpected system behavior. Beyond logging, collect query response time, RAG accuracy (if measurable), API usage, and user engagement metrics, monitored via a dedicated dashboard with alerts.
*   **User-Friendly Error Messages**: Clear and actionable error messages displayed to the user in the chatbot widget, avoiding technical jargon. This includes displaying distinct, themed loading spinners for loading states, specific error messages with retry options for error states, and clear "no results" messages for empty states.
*   **Graceful Degradation**: If an external service (e.g., OpenAI) is unavailable, the chatbot should inform the user and potentially offer a fallback (e.g., basic keyword search).
*   **Input Validation**: Strict validation of all user inputs to API endpoints to prevent injection attacks and malformed requests.
*   **Output Filtering/Sanitization**: Implement safety mechanisms (e.g., OpenAI's moderation API, custom filters) to prevent the chatbot from generating harmful, biased, or inappropriate content.
*   **Rate Limiting**: Configurable rate limits on API endpoints to prevent abuse and ensure fair usage.
*   **Retry Mechanisms**: Implement exponential backoff and retry logic for external API calls to handle transient failures.
*   **Circuit Breaker Pattern**: Consider implementing circuit breakers for critical external service calls to prevent cascading failures.

## Risks, Assumptions, Out of Scope

### Risks

*   **Embedding Quality**: Poor embedding quality from OpenAI could lead to irrelevant search results and poor RAG performance.
*   **LLM Hallucinations**: The OpenAI LLM might generate factually incorrect information, even with RAG, potentially undermining user trust.
*   **Data Staleness**: If the Docusaurus book content changes frequently, ensuring the vector database is up-to-date could be a challenge.
*   **Performance Bottlenecks**: Large book sizes or high concurrent user loads could strain Qdrant, Neon, or FastAPI, leading to slow responses.
*   **Security Vulnerabilities**: Potential for JWT misconfiguration, API key exposure, or prompt injection attacks.
*   **Multilingual Accuracy**: Maintaining high translation and response quality across English, Urdu, and Arabic.

### Assumptions

*   **OpenAI API Access**: Stable and performant access to OpenAI's embedding and LLM APIs.
*   **Qdrant Free Tier Sufficiency**: The Qdrant Cloud Free Tier will be sufficient for initial development and a reasonable amount of data/queries.
*   **Neon Postgres Scalability**: Neon's serverless architecture will adequately scale to meet database demands.
*   **Docusaurus Extensibility**: Docusaurus allows for seamless integration of custom React components (the chatbot widget) and client-side scripting.
*   **ChatKit Suitability**: ChatKit SDK provides the necessary functionality for conversation state management without major customization.
*   **Markdown Structure**: The Docusaurus Markdown files have a consistent enough structure to enable reliable text extraction, chunking, and metadata generation.
*   **Translation APIs**: If direct LLM translation is not used, external translation APIs for Urdu and Arabic will be available and performant.
*   **Data Volume**: Estimated book/course content up to 10GB; approximately 1000 messages/sessions per day.

### Out of Scope

*   **Content Editing within Chatbot**: The chatbot will not provide functionality to edit or modify the Docusaurus book content.
*   **User Management System**: Full-fledged user management (registration, profile management) beyond basic JWT authentication and logging.
*   **Advanced Analytics Dashboard**: While data is logged, a sophisticated analytics dashboard for chatbot usage is not included in this phase.
*   **Offline Mode**: The chatbot will require an active internet connection.
*   **Voice Input/Output**: No speech-to-text or text-to-speech capabilities are included.
*   **Complex conversational flows (e.g., multi-turn disambiguation)**: Focus on direct Q&A.
*   **Real-time content updates**: The embedding pipeline will be run periodically, not in real-time on every content change.

## Acceptance Criteria

### Measurable Outcomes

*   **AC-001**: 90% of user queries in "Ask the Book" mode receive an accurate and relevant answer from the book's content.
*   **AC-002**: 85% of user queries in "Ask the Course" mode receive an accurate and relevant answer from the combined course materials.
*   **AC-003**: 100% of responses in "User-Selected Text Mode" are derived exclusively from the selected text.
*   **AC-004**: The chatbot widget successfully loads and displays on Docusaurus pages with less than 500ms overhead.
*   **AC-005**: The chatbot widget's theme (dark/light) accurately reflects the Docusaurus theme 100% of the time.
*   **AC-006**: 95% of streamed chatbot responses complete rendering within 5 seconds.
*   **AC-007**: 98% of source citations generated are accurate and link to the correct section within the Docusaurus site.
*   **AC-008**: The chatbot interface and responses are correctly displayed in English, Urdu, and Arabic when the Docusaurus site language is switched.
*   **AC-009**: The backend API successfully logs 100% of chat interactions without errors.
*   **AC-010**: All defined API rate limits are enforced without blocking legitimate user traffic.
*   **AC-011**: JWT authenticated endpoints correctly validate tokens and reject unauthorized access attempts.

### Quality and Experience

*   **AC-012**: The chatbot provides helpful responses even for moderately complex or nuanced questions related to the content.
*   **AC-013**: The chatbot gracefully handles out-of-scope or irrelevant questions by informing the user it cannot provide an answer.
*   **AC-014**: The "Answer based on selected text" feature is easily discoverable and intuitive to use.
*   **AC-015**: The chatbot maintains conversational context within a single session using ChatKit.

### Deliverables Required: (As per user prompt - these are integrated into the sections above)
1.  **Title**: Included at the top.
2.  **Summary**: Included after the title.
3.  **Problem Definition**: Included.
4.  **User Types**: Included.
5.  **Functional Requirements**: Included under Requirements.
6.  **Non-functional Requirements**: Included under Requirements.
7.  **Full System Architecture (Frontend + Backend + DB + Vector DB)**: Included.
8.  **Data Pipeline & Embedding Pipeline**: Included.
9.  **UI/UX Flow (Docusaurus Widget)**: Included.
10. **Backend API Routes + Models**: Included.
11. **Integration Plan (end-to-end)**: Included.
12. **Error Handling + Safety**: Included.
13. **Risks, Assumptions, Out of Scope**: Included.
14. **Acceptance Criteria**: Included.