# Feature Specification: Integrated RAG Chatbot Retrieval Module

**Feature Branch**: `004-rag-chatbot-retrieval`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Integrated RAG Chatbot Retrieval Module Goal: Define the retrieval logic for the RAG chatbot embedded within the Physical AI & Humanoid Robotics textbook. Retrieval must fetch highly relevant text chunks from Qdrant, using Cohere embeddings, based on user-selected text or user queries. Target context Chatbot is part of a published textbook (web + PDF + interactive versions) Users include: students, researchers, robotics engineers, teachers Retrieval must support: General Q&A on the book Contextual Q&A on only user-selected text High-precision semantic search using Qdrant + Cohere Success criteria Retrieves top-N (3–5) semantically relevant passages from humanoid_ai_book_two Qdrant collection Must use embed-english-v3.0 with input_type="search_query" Retrieval must return: Payload text Metadata (section, chapter, page, id) Latency under 500ms for 1–2 paragraph queries Production-ready code in Python (FastAPI compatible) Must be able to switch between: Full-book retrieval Selected-text-only retrieval Constraints Embeddings already stored in Qdrant (no re-embedding) Qdrant Cloud Free Tier (limits: 1 collection, 1M vectors) Cohere API usage must not exceed monthly free quota Retrieval output format: JSON array of matched chunks Strictly no hallucinations in retrieval output No LLM-generated summaries at retrieval stage (that happens in generation stage) Not building Not building FastAPI routes here (done in integration stage) Not creating or uploading new embeddings Not generating final LLM answers Not implementing feedback ranking or reranking pipelines yet Not implementing UI — only core retrieval specification Deliverable A fully documented specification describing how retrieval works, referencing this code structure: def get_embedding(text): ... def retrieve(query): ... Including: Inputs Outputs Required parameters Error handling Switching between modes (full-book vs selected-text retrieval) Integration notes for Agents/ChatKit but do all in one folder which you created"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Contextual Q&A on Selected Text (Priority: P1)

A student reading the textbook highlights a paragraph about "inverse kinematics" and asks the chatbot to "explain this concept with an analogy." The system needs to retrieve passages that are semantically very similar to the selected text to provide a highly focused and relevant answer.

**Why this priority**: This is the core value proposition of an embedded chatbot – providing instant, contextual clarification on the material the user is actively consuming. It directly enhances the learning experience.

**Independent Test**: Can be tested by providing a specific text snippet and a query, and verifying that the retrieved results are directly related to the provided snippet, not just the general query. This delivers the value of contextual help.

**Acceptance Scenarios**:

1.  **Given** a user has selected a specific text passage, **When** they submit a query to the chatbot, **Then** the system should retrieve the top 3-5 text chunks most semantically relevant to the *selected passage*.
2.  **Given** a user has selected a text passage, **When** they ask a question, **Then** the retrieved chunks' metadata (chapter, section) should be closely related to the source of the selected passage.

---

### User Story 2 - General Q&A on Entire Book (Priority: P2)

A robotics engineer is looking for information on a specific topic without browsing the table of contents. They open the chatbot and type, "What are the main challenges in humanoid robot locomotion?" The system should search the entire textbook to find the most relevant sections that address this question.

**Why this priority**: This supports a key use case for discoverability and quick lookups, functioning as an intelligent search engine for the entire book.

**Independent Test**: Can be tested by submitting a general query without any selected text and evaluating if the top retrieved passages are the most relevant ones from the entire book for that query.

**Acceptance Scenarios**:

1.  **Given** a user has not selected any text, **When** they submit a query to the chatbot, **Then** the system should retrieve the top 3-5 text chunks from the entire book collection that are most semantically relevant to the query.
2.  **Given** a general query, **When** the results are returned, **Then** they should be presented as a JSON array of objects, each containing text and metadata.

---

### Edge Cases

-   **What happens when** a user's query has no relevant matches in the book? The system should return an empty array, not irrelevant results.
-   **How does the system handle** very short or ambiguous queries (e.g., "what is it?")? The system should still attempt to find the best possible matches, but the calling application should be prepared for low-relevance results.
-   **What happens when** the user selects a very large portion of text? The retrieval should focus on the most meaningful concepts within the selection, though performance may degrade. The system will handle a maximum of 4096 characters of user-selected text effectively.

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The system MUST accept a text-based user query.
-   **FR-002**: The system MUST optionally accept a larger block of user-selected text to define the context.
-   **FR-003**: The system MUST differentiate between two retrieval modes:
    -   **Full-Book Retrieval**: Search the entire textbook content if no selected text is provided.
    -   **Selected-Text Retrieval**: Search for content semantically similar to the user-selected text if it is provided.
-   **FR-004**: The system MUST return the top N (configurable, default 3-5) most semantically relevant text passages.
-   **FR-005**: The system MUST return each passage with its associated metadata: section, chapter, page, and a unique ID.
-   **FR-006**: The system's output MUST be a well-formed JSON array of matched chunks.

### Out of Scope

-   Generating the final LLM-generated answer.
-   Implementing the user interface for text selection or chat.
-   Implementing feedback, ranking, or reranking pipelines.
-   Creating, managing, or uploading embeddings.
-   Defining or building the API endpoints (e.g., in FastAPI).

### Assumptions

-   Textbook content has already been chunked and stored as embeddings in a vector database.
-   A mechanism exists within the user-facing application to capture selected text.
-   The number of results to retrieve (top-N) is a parameter that can be configured.

### Key Entities *(include if feature involves data)*

-   **Retrieved Chunk**: Represents a single relevant passage retrieved from the vector store.
    -   `text`: (String) The actual content of the passage.
    -   `metadata`: (Object) The contextual information for the passage.
        -   `section`: (String) The section title or identifier.
        -   `chapter`: (String) The chapter title or identifier.
        -   `page`: (Number) The page number where the passage can be found.
        -   `id`: (String) A unique identifier for the text chunk.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: For a benchmark set of 50 queries, at least 90% of the retrieved top-3 results are judged as "highly relevant" by a human evaluator.
-   **SC-002**: The P95 (95th percentile) latency for a retrieval operation (from query input to JSON output) MUST be under 500 milliseconds.
-   **SC-003**: The system MUST return zero results for queries that are definitively off-topic from the book's content.
-   **SC-004**: The system must correctly execute "Selected-Text Retrieval" in 100% of test cases where selected text is provided and "Full-Book Retrieval" in 100% of cases where it is not.