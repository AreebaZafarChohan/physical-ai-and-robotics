# End-to-End (E2E) Testing Plan for RAG Chatbot

## Objective:
-   Verify that all components of the RAG Chatbot system (frontend, backend, databases, LLM integrations) work together seamlessly.
-   Ensure all user stories are fully implemented and meet their acceptance criteria.

## Scope:
-   **User Story 1 (P1): Ask the Book**: Ask questions about book content, verify accurate and cited responses.
-   **User Story 3 (P1): Answer from Selected Text**: Select text, ask questions, verify context-aware responses.
-   **User Story 2 (P2): Ask the Course**: Switch to course mode, ask questions, verify integration of course materials.
-   **User Story 4 (P2): Multi-language Support**: Switch languages, verify UI and responses are localized.
-   Core functionalities: Chatbot widget appearance, opening/closing, message sending, response display.

## Test Environment:
-   Integrated development environment with frontend and backend running.
-   Staging/Production environment (for full deployment verification).
-   Browsers (Chrome, Firefox, Safari, Edge).

## Test Cases:

### 1. Ask the Book (Happy Path):
-   **Precondition**: Chatbot in "Book" mode.
-   **Steps**:
    1.  Open Docusaurus site.
    2.  Open chatbot.
    3.  Ask a question directly related to a known section of the book.
-   **Expected**:
    1.  Bot responds with a relevant answer.
    2.  Response includes clickable citation links.
    3.  Response is in the default language (English).

### 2. Ask the Book (Out-of-context):
-   **Precondition**: Chatbot in "Book" mode.
-   **Steps**:
    1.  Ask a question unrelated to the book content.
-   **Expected**: Bot states it doesn't know based on provided context.

### 3. Answer from Selected Text:
-   **Precondition**: Docusaurus page with content visible.
-   **Steps**:
    1.  Highlight a paragraph of text.
    2.  Click the "Answer based on selected text" button.
    3.  Ask a question relevant *only* to the highlighted text.
-   **Expected**:
    1.  Chatbot opens with the selected text as context.
    2.  Bot responds accurately based *only* on the selected text.

### 4. Ask the Course:
-   **Precondition**: Chatbot open.
-   **Steps**:
    1.  Switch mode to "Course".
    2.  Ask a question that would require knowledge from supplementary materials (if available).
-   **Expected**: Bot responds, potentially integrating information beyond the core book content.

### 5. Multi-language Support:
-   **Precondition**: Chatbot open.
-   **Steps**:
    1.  Select "Urdu" or "Arabic" from the language dropdown.
    2.  Ask a simple question.
-   **Expected**:
    1.  Chatbot UI elements (e.g., "AI Assistant") are translated.
    2.  Bot responds in the selected language.

### 6. Error Handling:
-   **Steps**: Trigger known error conditions (e.g., invalid API key if possible, malformed request, unavailable external service).
-   **Expected**: Chatbot displays graceful error messages; backend logs errors appropriately.

## Procedure:
1.  Set up frontend and backend development environments.
2.  Populate Qdrant with book and course embeddings via `/embed` endpoint.
3.  Manually execute test cases following the outlined steps.
4.  Automate tests where feasible (e.g., using Playwright, Cypress).
5.  Document test results and any bugs found.

## Acceptance Criteria:
-   All core user stories are fully functional.
-   No critical bugs or regressions are introduced.
-   Chatbot is user-friendly and intuitive.
