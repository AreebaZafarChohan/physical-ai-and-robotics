---
id: 0001
title: Generate RAG Chatbot Docusaurus Spec
stage: spec
date: 2025-12-06
surface: agent
model: gemini-1.5-pro
feature: 001-rag-chatbot-docusaurus
branch: 001-rag-chatbot-docusaurus
user: (not provided)
command: /sp.specify "You are an expert AI Product Architect. I want a complete and detailed specification for building a **RAG Chatbot embedded inside a Docusaurus Book website**, using the following stack: • OpenAI Agents + ChatKit SDK • FastAPI backend • Neon Serverless Postgres (for metadata + conversation logs) • Qdrant Cloud Free Tier (vector DB for embeddings) • Docusaurus frontend + floating chatbot widget • OpenAI embeddings + RAG pipeline • User-selected text → answer only from selection mode =========================== SYSTEM BEHAVIOR =========================== The chatbot must be able to: 1. **Answer questions about the book's content** stored as Markdown files in the Docusaurus project. 2. **Perform Retrieval-Augmented Generation (RAG)** using: - Embeddings stored in Qdrant - Metadata stored in Neon Postgres - Vector search (semantic + keyword hybrid) 3. **Provide dual modes:** - "Ask the Book" → search only the uploaded markdown chapters - "Ask the Course" → search all course documents (book + extra notes) 4. **User-Selected Text Mode**: - If the user highlights/selects text inside the book page → bot answers ONLY from that text (no other retrieval). 5. **Frontend Chat Widget Features**: - Floating bottom-right widget - Dark/light theme sync with Docusaurus theme - Streaming token output - Source citations + clickable anchors - “Answer based on selected text” button - Language-switch: English, Urdu, Arabic 6. **Backend Features (FastAPI)**: - API routes: • /embed • /query • /ask • /selected_text • • /log (store into Neon) - JWT optional user auth - Rate limiting + safety mode - Integration with ChatKit for conversation state 7. **Database Layer**: - Neon Postgres schema: • users • chat_sessions • messages • feedback • book_metadata 8. **Vector DB Layer (Qdrant)**: - Collection name: book_chapters - Metadata: chapter, source_link, paragraph_id =========================== DELIVERABLES REQUIRED =========================== Your output should contain: 1. **Title** 2. **Summary** 3. **Problem Definition** 4. **User Types** 5. **Functional Requirements** 6. **Non-functional Requirements** 7. **Full System Architecture (Frontend + Backend + DB + Vector DB)** 8. **Data Pipeline & Embedding Pipeline** 9. **UI/UX Flow (Docusaurus Widget)** 10. **Backend API Routes + Models** 11. **Integration Plan (end-to-end)** 12. **Error Handling + Safety** 13. **Risks, Assumptions, Out of Scope** 14. **Acceptance Criteria** Follow a formal product-spec format."
labels: [rag, chatbot, docusaurus, openai, fastapi, neon, qdrant]
links:
  spec: /specs/001-rag-chatbot-docusaurus/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /specs/001-rag-chatbot-docusaurus/spec.md
  - /specs/001-rag-chatbot-docusaurus/checklists/requirements.md
tests:

---

## Prompt

You are an expert AI Product Architect. I want a complete and detailed specification for building a **RAG Chatbot embedded inside a Docusaurus Book website**, using the following stack: • OpenAI Agents + ChatKit SDK • FastAPI backend • Neon Serverless Postgres (for metadata + conversation logs) • Qdrant Cloud Free Tier (vector DB for embeddings) • Docusaurus frontend + floating chatbot widget • OpenAI embeddings + RAG pipeline • User-selected text → answer only from selection mode =========================== SYSTEM BEHAVIOR =========================== The chatbot must be able to: 1. **Answer questions about the book's content** stored as Markdown files in the Docusaurus project. 2. **Perform Retrieval-Augmented Generation (RAG)** using: - Embeddings stored in Qdrant - Metadata stored in Neon Postgres - Vector search (semantic + keyword hybrid) 3. **Provide dual modes:** - "Ask the Book" → search only the uploaded markdown chapters - "Ask the Course" → search all course documents (book + extra notes) 4. **User-Selected Text Mode**: - If the user highlights/selects text inside the book page → bot answers ONLY from that text (no other retrieval). 5. **Frontend Chat Widget Features**: - Floating bottom-right widget - Dark/light theme sync with Docusaurus theme - Streaming token output - Source citations + clickable anchors - “Answer based on selected text” button - Language-switch: English, Urdu, Arabic 6. **Backend Features (FastAPI)**: - API routes: • /embed • /query • /ask • /selected_text • /log (store into Neon) - JWT optional user auth - Rate limiting + safety mode - Integration with ChatKit for conversation state 7. **Database Layer**: - Neon Postgres schema: • users • chat_sessions • messages • feedback • book_metadata 8. **Vector DB Layer (Qdrant)**: - Collection name: book_chapters - Metadata: chapter, source_link, paragraph_id =========================== DELIVERABLES REQUIRED =========================== Your output should contain: 1. **Title** 2. **Summary** 3. **Problem Definition** 4. **User Types** 5. **Functional Requirements** 6. **Non-functional Requirements** 7. **Full System Architecture (Frontend + Backend + DB + Vector DB)** 8. **Data Pipeline & Embedding Pipeline** 9. **UI/UX Flow (Docusaurus Widget)** 10. **Backend API Routes + Models** 11. **Integration Plan (end-to-end)** 12. **Error Handling + Safety** 13. **Risks, Assumptions, Out of Scope** 14. **Acceptance Criteria** Follow a formal product-spec format.

## Response snapshot

The feature specification for 'RAG Chatbot for Docusaurus Book Website' has been successfully created, including the `spec.md` and `requirements.md` checklist files.

## Outcome

- ✅ Impact: Spec created
- 🧪 Tests: No tests were run for spec generation
- 📁 Files: Created `specs/001-rag-chatbot-docusaurus/spec.md` and `specs/001-rag-chatbot-docusaurus/checklists/requirements.md`
- 🔁 Next prompts: /sp.clarify or /sp.plan
- 🧠 Reflection: Ensure correct feature name in `create-phr.sh` command.

## Evaluation notes (flywheel)

- Failure modes observed: Incorrect feature name in `create-phr.sh`
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
