# Data Model: RAG Chatbot for Docusaurus Book

This document defines the detailed schema for each entity identified in the feature specification, to be implemented in Neon Postgres.

## Entities

### User

Represents an individual interacting with the chatbot.

*   **`id`** (PK, UUID): Unique identifier for the user.
*   **`email`** (string, optional, unique): User's email address.
*   **`preferences`** (JSONB): Stores user-specific settings (e.g., language, theme).
*   **`created_at`** (timestamp with timezone): Timestamp of user creation.
*   **`last_login_at`** (timestamp with timezone): Timestamp of last user login.

### ChatSession

A continuous conversation thread between a user and the chatbot.

*   **`id`** (PK, UUID): Unique identifier for the chat session.
*   **`user_id`** (FK to `User.id`, nullable): Foreign key linking to the User entity. Nullable for unauthenticated users.
*   **`started_at`** (timestamp with timezone): Timestamp when the session started.
*   **`last_activity_at`** (timestamp with timezone): Timestamp of the last activity in the session.
*   **`mode`** (enum: 'book', 'course'): The operational mode of the chatbot during the session.
*   **`status`** (enum: 'active', 'ended', 'archived'): Current status of the chat session. Further states like 'pending' or 'error' may be considered during implementation.

### Message

An individual turn in a chat session, either from the user or the chatbot.

*   **`id`** (PK, UUID): Unique identifier for the message.
*   **`session_id`** (FK to `ChatSession.id`): Foreign key linking to the parent chat session.
*   **`sender`** (enum: 'user', 'bot'): Indicates whether the message was sent by the user or the chatbot.
*   **`content`** (text): The actual text content of the message.
*   **`timestamp`** (timestamp with timezone): Timestamp when the message was sent.
*   **`citations`** (JSONB): Array of objects, each containing `text` (citation snippet) and `link` (URL to source).
*   **`language`** (string, e.g., 'en', 'ur', 'ar'): The language of the message.

### Feedback

User-provided assessment or comments on chatbot responses.

*   **`id`** (PK, UUID): Unique identifier for the feedback entry.
*   **`message_id`** (FK to `Message.id`, nullable): Foreign key linking to a specific message. Nullable if feedback is for the overall session.
*   **`user_id`** (FK to `User.id`, nullable): Foreign key linking to the User entity. Nullable for unauthenticated feedback.
*   **`type`** (enum: 'positive', 'negative', 'comment'): Type of feedback.
*   **`comment`** (text, optional): Additional text comments from the user.
*   **`created_at`** (timestamp with timezone): Timestamp when the feedback was provided.

### BookMetadata

Structured information about the book's chapters, sections, and their corresponding file paths or unique identifiers.

*   **`id`** (PK, UUID or natural key from `source_link` + `paragraph_id`): Unique identifier for the metadata entry.
*   **`chapter`** (string): The title or identifier of the chapter.
*   **`source_link`** (string): URL or path to the original Docusaurus page/section.
*   **`paragraph_id`** (string): Unique ID for the specific text chunk within its source document.
*   **`text_content_hash`** (string): Hash of the text content for change detection.
*   **`created_at`** (timestamp with timezone): Timestamp of metadata creation.
*   **`updated_at`** (timestamp with timezone): Timestamp of last metadata update.
