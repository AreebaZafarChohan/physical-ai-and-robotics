# Data Model: Integrate RAG Chatbot

**Branch**: `007-integrate-rag-chatbot` | **Date**: 2025-12-17 | **Spec**: specs/007-integrate-rag-chatbot/spec.md
**Input**: Key Entities from `spec.md`

## Entities

### User Question
- **Description**: The natural language query posed by the reader.
- **Attributes**:
    - `text`: String - The actual question text.

### Selected Text
- **Description**: Optional segment of book content chosen by the reader to provide additional context for a question.
- **Attributes**:
    - `text`: String - The selected text content.

### Chatbot Response
- **Description**: The AI-generated answer provided by the RAG backend.
- **Attributes**:
    - `answer`: String - The AI-generated response text.