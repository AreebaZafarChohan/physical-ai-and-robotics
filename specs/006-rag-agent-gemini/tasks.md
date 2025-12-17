# Tasks for RAG Agent with Gemini Backend

**Branch**: `006-rag-agent-gemini` | **Date**: 2025-12-16 | **Spec**: [specs/006-rag-agent-gemini/spec.md](specs/006-rag-agent-gemini/spec.md)
**Plan**: [specs/006-rag-agent-gemini/plan.md](specs/006-rag-agent-gemini/plan.md)

This document outlines the implementation tasks for the RAG Agent with Gemini Backend, organized by phases and user stories.

---
## Phase 1: Setup

-   [ ] T001 Create Python virtual environment and activate it.
-   [ ] T002 Add required Python packages (`fastapi`, `uvicorn`, `qdrant-client`, `openai-agents-sdk`, `async-openai`, `python-dotenv`) to `backend/requirements.txt`.
-   [ ] T003 Install Python dependencies: `pip install -r backend/requirements.txt`.
-   [ ] T004 Create `.env.example` with placeholders for `GEMINI_API_KEY`, `GEMINI_API_BASE_URL`, `QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION_NAME`, `COHERE_API_KEY`.
-   [ ] T005 Update `README.md` in `backend/` to include setup instructions for environment variables.

---
## Phase 2: Foundational

-   [ ] T006 Implement logging configuration for the backend service in `backend/src/utils/logging_config.py`.
-   [ ] T007 Configure `AsyncOpenAI` client with `GEMINI_API_BASE_URL` and `GEMINI_API_KEY` in `backend/src/models/gemini_model.py`.
-   [ ] T008 Initialize `OpenAIChatCompletionsModel` using the configured Gemini model in `backend/src/models/gemini_model.py`.
-   [ ] T009 Create a utility to load environment variables from `.env` in `backend/src/utils/config.py`.
-   [ ] T010 Implement Qdrant client initialization and connection in `backend/src/retrieval/qdrant_client.py`.
-   [ ] T011 Implement Cohere embedding generation for query in `backend/src/retrieval/embedder.py`.

---
## Phase 3: User Story 1 - Agent Answers Grounded Questions (Priority: P1)

**Goal**: The RAG agent successfully retrieves relevant information and formulates an answer strictly grounded in that information.

**Independent Test Criteria**: Provide questions known to be answerable by the Qdrant content and verify that the responses are accurate and directly reference the retrieved information.

-   [ ] T012 [P] [US1] Implement retrieval tool using `@function_tool` decorator in `backend/src/agent/tools.py`.
-   [ ] T013 [P] [US1] Define retrieval tool logic to generate query embeddings using Cohere and query Qdrant for top-k chunks in `backend/src/agent/tools.py`.
-   [ ] T014 [P] [US1] Ensure retrieval tool returns only text payloads from Qdrant results in `backend/src/agent/tools.py`.
-   [ ] T015 [P] [US1] Define system prompt for the agent enforcing retrieval-first behavior and context-only answering in `backend/src/agent/instructions.py`.
-   [ ] T016 [P] [US1] Instantiate the RAG Agent using OpenAI Agents SDK in `backend/src/agent/rag_agent.py`.
-   [ ] T017 [P] [US1] Attach the Gemini-backed model and the retrieval tool to the RAG Agent in `backend/src/agent/rag_agent.py`.
-   [ ] T018 [US1] Implement agent execution flow: pass user query, invoke retrieval tool, inject retrieved content into agent reasoning in `backend/src/services/agent_service.py`.
-   [ ] T019 [US1] Ensure the Gemini model generates grounded answers based on the retrieved content in `backend/src/services/agent_service.py`.

---
## Phase 4: User Story 2 - Agent Responds to Unanswerable Questions (Priority: P1)

**Goal**: The agent identifies when no relevant information exists and responds with "I don't know" without hallucinating.

**Independent Test Criteria**: Provide questions known to be unanswerable by the Qdrant content and verify that it consistently responds with "I don't know."

-   [ ] T020 [P] [US2] Define explicit fallback response ("I don't know") for missing context in the agent's instructions/logic in `backend/src/agent/instructions.py` or `backend/src/services/agent_service.py`.
-   [ ] T021 [US2] Verify agent cannot answer without tool invocation and responds with fallback when retrieval yields no results in `backend/src/services/agent_service.py`.

---
## Phase 5: User Story 3 - API Interaction with the Agent (Priority: P2)

**Goal**: Integrate the RAG agent into an application via a FastAPI endpoint.

**Independent Test Criteria**: Send HTTP requests to the FastAPI endpoint with queries and verify that it returns the agent's responses correctly.

-   [ ] T022 [P] [US3] Create FastAPI application instance in `backend/src/api/main.py`.
-   [ ] T023 [P] [US3] Implement `/chat` endpoint in `backend/src/api/main.py` according to `chat_api.json` contract.
-   [ ] T024 [P] [US3] Define Pydantic model for incoming chat request (`query: str`) in `backend/src/models/chat_request.py`.
-   [ ] T025 [P] [US3] Define Pydantic model for outgoing chat response (`response: str`) in `backend/src/models/chat_response.py`.
-   [ ] T026 [US3] Implement logic to accept user query payload and invoke the agent runner in `backend/src/api/main.py`.
-   [ ] T027 [US3] Implement logic to return the agent's response, or generic error on retrieval tool failure in `backend/src/api/main.py`.
-   [ ] T028 [US3] Add basic error handling for invalid requests to `/chat` endpoint in `backend/src/api/main.py`.

---
## Phase 6: Polish & Cross-Cutting Concerns

-   [ ] T029 Implement basic unit tests for Gemini model configuration in `backend/tests/unit/test_gemini_model.py`.
-   [ ] T030 Implement basic unit tests for Qdrant client and embedder in `backend/tests/unit/test_retrieval.py`.
-   [ ] T031 Implement integration tests for the retrieval tool's functionality in `backend/tests/integration/test_tools.py`.
-   [ ] T032 Implement integration tests for the RAG agent's grounded and ungrounded responses in `backend/tests/integration/test_rag_agent.py`.
-   [ ] T033 Implement end-to-end API tests for the `/chat` endpoint in `backend/tests/integration/test_api.py`.
-   [ ] T034 Configure Prometheus metrics for request count and latency for the FastAPI application in `backend/src/metrics.py`.
-   [ ] T035 Integrate metrics exposition endpoint into FastAPI application in `backend/src/api/main.py`.
-   [ ] T036 Review and update `backend/README.md` with full setup, environment variables, running instructions, and API usage.

---
**Dependency Graph (User Story Completion Order):**
-   Phase 1 (Setup) -> Phase 2 (Foundational)
-   Phase 2 (Foundational) -> Phase 3 (User Story 1)
-   Phase 2 (Foundational) -> Phase 4 (User Story 2)
-   Phase 2 (Foundational) -> Phase 5 (User Story 3)
-   Phase 3 (User Story 1) and Phase 4 (User Story 2) are prerequisites for full agent functionality in Phase 5.
-   Phase 5 (User Story 3) relies on the agent functionality from Phase 3 and 4.
-   Phase 6 (Polish & Cross-Cutting Concerns) can run in parallel with or after User Story phases.

**Parallel Execution Examples:**
-   **After Phase 2:**
    -   Implementing tasks for User Story 1 (Phase 3) can proceed in parallel with implementing tasks for User Story 2 (Phase 4).
    -   Within each User Story phase, tasks marked with `[P]` can be developed concurrently. For example, `T012`, `T013`, `T014`, `T015`, `T016`, `T017` in Phase 3.
-   **During User Story Phases:**
    -   Tasks for User Story 3 (Phase 5) can begin once the foundational agent components are in place (Phase 2).
    -   Test tasks in Phase 6 can be developed concurrently with their respective functional implementations.

**Implementation Strategy:**
-   **MVP First:** Prioritize implementing User Story 1 and User Story 2, as they represent the core RAG agent functionality and essential fallback behavior.
-   **Incremental Delivery:** Develop and test each user story phase independently.
-   **Test-Driven Development (TDD):** Where applicable, tests (T029-T033) should be written before or alongside the code they validate.