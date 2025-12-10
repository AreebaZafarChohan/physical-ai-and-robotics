---
id: 0003
title: AI-Native RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-10
surface: agent
model: gemini-1.5-flash
feature: 001-rag-chatbot-integration
branch: 001-rag-chatbot-integration
user: user
command: /sp.plan
labels: ["plan", "rag-chatbot"]
links:
  spec: specs/001-rag-chatbot-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot-integration/plan.md
tests:

---

## Prompt

You are now generating the COMPLETE architecture plan for Step-2 of the Panaversity AI-Native Textbook: the RAG Chatbot System.

Use the specification from /sp.specify and produce a fully structured, detailed, engineering-grade plan.

This plan will be consumed by /sp.tasks and then /sp.implement, so it must be exhaustive.

Follow the official Spec-Kit-Plus structure:

1. **High-Level Summary**
   Provide a short overview of what the Step-2 RAG chatbot will do: Docusaurus-embedded chatbot, OpenAI Agents SDK with Gemini model, Qdrant retrieval, Neon personalization, both standard RAG and selected-text mode.

2. **System Architecture**
   Create a block-diagram-style breakdown (text-based) showing all major components:
   - FastAPI backend
   - Agent (Gemini via OpenAI Agents SDK)
   - Tools (Qdrant search tool, selected text tool, Neon preferences tool)
   - Qdrant Cloud vector DB
   - Neon Postgres user DB
   - ChatKit frontend
   - Docusaurus embedding component
   - Auth layer (better-auth)
   - Streaming pipeline

3. **Data Flow (End-to-End)**
   Describe precise step-by-step flow for:
   - Normal RAG query
   - Selected-text mode query
   - User preference loading
   - Response streaming
   - Storing user settings

4. **Backend Services**
   Define all backend modules that will exist:
   - `/agents/` (agent setup)
   - `/tools/` (function_tools for qdrant_search, selected_text, neon_user_profile)
   - `/routes/chat.py`
   - `/routes/user.py`
   - `/db/` (Neon connector + SQL models)
   - `/vector/` (Qdrant connector)
   - `/utils/` (logging, error handling)
   Include exactly what each file does.

5. **FastAPI Endpoints**
   List all endpoints with:
   - method (GET/POST)
   - request body
   - response schema
   - which tools they call
   - streaming enabled? (yes/no)
   - personalization applied? (yes/no)

6. **Agents + Tools Design**
   Specify:
   - System prompt template for RAG mode
   - System prompt template for Selected-Text mode
   - Tool schema for:
     - Qdrant search
     - Selected text
     - Neon preferences loader
   - Model: `gemini-2.0-flash` via OpenAIChatCompletionsModel
   - Tool arbitration behavior
   - Hallucination guardrails

7. **Retrieval Logic**
   Define:
   - embedding model (`text-embedding-004`)
   - vector size
   - top_k retrieval
   - metadata filters
   - how citations are mapped back to chunk IDs
   - selected-text bypass logic

8. **Neon Personalization Logic**
   Describe:
   - user profile schema
   - how background modifies the system prompt
   - fallbacks when profile missing
   - optional difficulty tuning
   - Urdu/English output switching

9.  **Frontend Integration Plan (ChatKit + Docusaurus)**
    Provide:
    - component structure
      - `<FloatingChatbot />`
      - `<AskFromSelection />`
    - event flow: “highlight text → ask AI”
    - how streaming is handled in ChatKit
    - how to display citations
    - how to switch between the two modes

10. **DevOps & Deployment**
    Include:
    - environment variables needed
    - Qdrant API keys
    - Neon connection string
    - backend deployment on Railway/Render
    - frontend deployment on Vercel
    - CORS configuration
    - folder tree for final repo

11. **Success Metrics**
    Define:
    - latency targets
    - hallucination target (<2%)
    - correctness metric
    - UX smoothness (highlight-to-chat < 300ms)

12. **Risks & Mitigation**
    Identify:
    - Qdrant rate limits
    - Gemini response drift
    - streaming disconnects
    - CORS issues
    - performance bottlenecks in Docusaurus

Requirements:
- Your plan must be complete enough that `/sp.tasks` can break it into 40–80 tasks easily.
- Avoid vague statements.
- Everything must be explicit, production-ready, and implementable.
                                                                                                            Collection Name of Qdrant Where data is stored is "ai_robotics_book".
End of prompt.

## Response snapshot

Generated a comprehensive implementation plan for the AI-Native RAG Chatbot System, covering all requested sections in detail, including system architecture, data flow, backend services, API endpoints, agent/tool design, retrieval logic, personalization, frontend integration, DevOps, success metrics, and risks.

## Outcome

- ✅ Impact: A comprehensive implementation plan has been created for the AI-Native RAG Chatbot System, detailing system architecture, data flow, backend services, API endpoints, agent/tool design, retrieval logic, personalization, frontend integration, DevOps, success metrics, and risks.
- 🧪 Tests: N/A
- 📁 Files: Created `specs/001-rag-chatbot-integration/plan.md`.
- 🔁 Next prompts: Ready for `/sp.tasks`.
- 🧠 Reflection: The plan was generated successfully, covering all requested sections in detail.

## Evaluation notes (flywheel)

- Failure modes observed: None during plan generation.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A