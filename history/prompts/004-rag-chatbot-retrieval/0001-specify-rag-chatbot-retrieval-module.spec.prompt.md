---
id: 0001
title: Specify RAG Chatbot Retrieval Module
stage: spec
date: 2025-12-10
surface: agent
model: gemini-cli
feature: 004-rag-chatbot-retrieval
branch: 004-rag-chatbot-retrieval
user: unspecified
command: /sp.specify
labels: ["spec", "rag", "chatbot", "retrieval"]
links:
  spec: specs/004-rag-chatbot-retrieval/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-rag-chatbot-retrieval/spec.md
 - specs/004-rag-chatbot-retrieval/checklists/requirements.md
tests:
 - None
---

## Prompt

Integrated RAG Chatbot Retrieval Module Goal: Define the retrieval logic for the RAG chatbot embedded within the Physical AI & Humanoid Robotics textbook. Retrieval must fetch highly relevant text chunks from Qdrant, using Cohere embeddings, based on user-selected text or user queries. Target context Chatbot is part of a published textbook (web + PDF + interactive versions) Users include: students, researchers, robotics engineers, teachers Retrieval must support: General Q&A on the book Contextual Q&A on only user-selected text High-precision semantic search using Qdrant + Cohere Success criteria Retrieves top-N (3–5) semantically relevant passages from humanoid_ai_book_two Qdrant collection Must use embed-english-v3.0 with input_type="search_query" Retrieval must return: Payload text Metadata (section, chapter, page, id) Latency under 500ms for 1–2 paragraph queries Production-ready code in Python (FastAPI compatible) Must be able to switch between: Full-book retrieval Selected-text-only retrieval Constraints Embeddings already stored in Qdrant (no re-embedding) Qdrant Cloud Free Tier (limits: 1 collection, 1M vectors) Cohere API usage must not exceed monthly free quota Retrieval output format: JSON array of matched chunks Strictly no hallucinations in retrieval output No LLM-generated summaries at retrieval stage (that happens in generation stage) Not building Not building FastAPI routes here (done in integration stage) Not creating or uploading new embeddings Not generating final LLM answers Not implementing feedback ranking or reranking pipelines yet Not implementing UI — only core retrieval specification Deliverable A fully documented specification describing how retrieval works, referencing this code structure: def get_embedding(text): ... def retrieve(query): ... Including: Inputs Outputs Required parameters Error handling Switching between modes (full-book vs selected-text retrieval) Integration notes for Agents/ChatKit but do all in one folder which you created

## Response snapshot

I have created the initial specification for the `RAG Chatbot Retrieval Module` on branch `004-rag-chatbot-retrieval`. You can review the full spec at `specs/004-rag-chatbot-retrieval/spec.md`.

Before we proceed to the planning phase, I need one point of clarification based on the validation checklist (`specs/004-rag-chatbot-retrieval/checklists/requirements.md`).

## Question 1: Maximum Selection Size

**Context**: In the "Edge Cases" section of the specification, I noted: "What happens when the user selects a very large portion of text? The retrieval should focus on the most meaningful concepts within the selection, though performance may degrade."

**What we need to know**: What is the maximum size of user-selected text (in characters) that the retrieval system should be designed to handle effectively? This has implications for performance and the cost of embedding the selected text.

**Suggested Answers**:

| Option | Answer | Implications |
|---|---|---|
| A | **1024 characters** (approx. 2-3 paragraphs) | Balances conciseness and context. Good for focused questions on a specific point. Low performance/cost impact. |
| B | **4096 characters** (approx. 1-2 pages) | A common context window size for many models. Allows for broader context from a larger selection. Moderate performance/cost impact. |
| C | **8192 characters** (approx. 2-4 pages) | Provides very broad context, useful for summarizing larger sections, but may introduce more noise. Higher performance/cost impact. |
| Custom | Provide your own answer | You can specify a different character limit based on your expected use case. |

**Your choice**: _[Please respond with your choice, e.g., "Q1: B"]_

## Outcome

- ✅ Impact: High. Created a new feature specification and branch.
- 🧪 Tests: None run.
- 📁 Files: 
  - `specs/004-rag-chatbot-retrieval/spec.md`
  - `specs/004-rag-chatbot-retrieval/checklists/requirements.md`
- 🔁 Next prompts: `/sp.plan`
- 🧠 Reflection: The process of creating the spec, validating it, and handling clarifications worked as expected. The initial user prompt was very detailed, which made generating the spec straightforward. The clarification loop was necessary for a key technical parameter.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): None
