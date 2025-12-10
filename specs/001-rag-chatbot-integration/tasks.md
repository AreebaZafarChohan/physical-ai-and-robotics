# Tasks: AI-Native RAG Chatbot System

**Input**: Design documents from `/specs/001-rag-chatbot-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/, quickstart.md

**Tests**: Test tasks are included as they are integral to ensuring correctness.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/` (as per plan.md structure)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic environment configuration.

- [ ] T001 Create backend/ folder and initialize Python virtual environment for backend.
- [ ] T002 Install core Python packages (`FastAPI`, `uvicorn`, `python-dotenv`, `psycopg2-binary`, `qdrant-client`, `openai`, `python-jose`, `passlib[bcrypt]`) in `backend/`.
- [ ] T003 Generate `backend/requirements.txt` from installed packages.
- [ ] T004 Create frontend/ folder and initialize Docusaurus project (if not existing).
- [ ] T005 [P] Create `.env.example` in `backend/` with placeholders for `QDRANT_API_KEY`, `QDRANT_URL`, `NEON_DATABASE_URL`, `OPENAI_API_KEY`, `JWT_SECRET_KEY`, `ALGORITHM`, `ACCESS_TOKEN_EXPIRE_MINUTES`, `FRONTEND_ORIGIN`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [ ] T006 Create `backend/main.py` for FastAPI application entry point.
- [ ] T007 Implement CORS middleware in `backend/utils/cors.py` and integrate into `backend/main.py`.
- [ ] T008 Implement logging utilities in `backend/utils/logging.py`.
- [ ] T009 Implement custom exception handling and error response schemas in `backend/utils/errors.py`.
- [ ] T010 Implement JWT encoding/decoding and password hashing utilities in `backend/utils/security.py`.
- [ ] T011 Create `backend/db/connection.py` to establish and manage connection to Neon Postgres.
- [ ] T012 Define User Profile SQL model in `backend/db/models.py` based on `data-model.md`.
- [ ] T013 Implement basic CRUD operations for User Profile in `backend/db/crud.py`.
- [ ] T014 Create `backend/vector/qdrant_client.py` to configure Qdrant client and connect to `ai_robotics_book` collection.
- [ ] T015 Create `backend/utils/prompts.py` for managing system prompt templates.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel.

---

## Phase 3: User Story 4 - User Profile Management (Priority: P2)

**Goal**: Enable users to create accounts, sign in, and manage their personalization preferences securely.

**Independent Test**: A user can successfully sign up, log in, and update their profile, with changes persisting in the Neon database.

### Implementation for User Story 4

- [ ] T016 [P] [US4] Define Pydantic models for user signup, signin, and profile update requests in `backend/schemas/user.py`.
- [ ] T017 [US4] Implement `/api/v1/auth/signup` endpoint in `backend/routes/user.py`, using `backend/db/crud.py` for user creation and `backend/utils/security.py` for password hashing.
- [ ] T018 [US4] Implement `/api/v1/auth/signin` endpoint in `backend/routes/user.py` for user authentication and JWT token generation using `backend/utils/security.py`.
- [ ] T019 [US4] Implement JWT dependency and current user retrieval logic in `backend/utils/security.py`.
- [ ] T020 [US4] Implement `/api/v1/user/profile` endpoint in `backend/routes/user.py` to fetch authenticated user's profile.
- [ ] T021 [US4] Implement `/api/v1/user/update` endpoint in `backend/routes/user.py` to update authenticated user's profile.
- [ ] T022 [US4] Define `neon_preferences_tool` in `backend/tools/neon_user_profile_tool.py` using `@function_tool` decorator, integrating with `backend/db/crud.py`.

### Tests for User Story 4

- [ ] T023 [P] [US4] Write unit tests for user authentication and profile management endpoints in `tests/backend/routes/test_user.py`.
- [ ] T024 [P] [US4] Write unit tests for `neon_preferences_tool` in `tests/backend/tools/test_neon_user_profile_tool.py`.

**Checkpoint**: At this point, User Story 4 should be fully functional and testable independently.

---

## Phase 4: User Story 1 - Ask AI about the book (Normal RAG) (Priority: P1)

**Goal**: Provide a chatbot that answers general questions using the full book content via RAG.

**Independent Test**: A user can type a question, and the chatbot returns a coherent, cited answer based on the Qdrant vector store.

### Implementation for User Story 1

- [ ] T025 [US1] Implement `qdrant_search_tool` in `backend/tools/qdrant_tool.py` using `@function_tool` decorator, integrating with `backend/vector/qdrant_client.py`.
- [ ] T026 [US1] Define RAG system prompt template in `backend/utils/prompts.py` incorporating `{user_profile_summary}` and `{personalization_style_description}`.
- [ ] T027 [US1] Configure `gemini-2.0-flash` via `OpenAIChatCompletionsModel` in `backend/agents/gemini_agent.py`.
- [ ] T028 [US1] Implement core agent logic for Normal RAG mode in `backend/agents/gemini_agent.py`, handling context injection from `qdrant_search_tool` and basic personalization.
- [ ] T029 [US1] Create `/api/v1/chat` endpoint in `backend/routes/chat.py` to accept chat queries, call the agent, and prepare for streaming.

### Tests for User Story 1

- [ ] T030 [P] [US1] Write unit tests for `qdrant_search_tool` in `tests/backend/tools/test_qdrant_tool.py`.
- [ ] T031 [P] [US1] Write unit tests for Normal RAG agent logic in `tests/backend/agents/test_gemini_agent.py`.
- [ ] T032 [P] [US1] Write integration tests for `/api/v1/chat` endpoint in `tests/backend/routes/test_chat.py`.

**Checkpoint**: At this point, User Stories 4 AND 1 should both work independently.

---

## Phase 5: User Story 2 - Ask AI about selected text (Selected-Text RAG) (Priority: P1)

**Goal**: Allow users to ask questions specifically about highlighted text on a Docusaurus page.

**Independent Test**: A user can highlight text, trigger the chatbot, and receive an answer strictly based on the highlighted content, with no external information.

### Implementation for User Story 2

- [ ] T033 [US2] Implement `selected_text_tool` in `backend/tools/selected_text_tool.py` using `@function_tool` decorator.
- [ ] T034 [US2] Define Selected-Text RAG system prompt template in `backend/utils/prompts.py` with strong hallucination guardrails.
- [ ] T035 [US2] Implement agent logic for Selected-Text RAG mode in `backend/agents/gemini_agent.py`, handling direct text injection and basic personalization.
- [ ] T036 [US2] Create `/api/v1/chat/selected` endpoint in `backend/routes/chat.py` to accept selected text queries and call the agent.

### Tests for User Story 2

- [ ] T037 [P] [US2] Write unit tests for `selected_text_tool` in `tests/backend/tools/test_selected_text_tool.py`.
- [ ] T038 [P] [US2] Write integration tests for `/api/v1/chat/selected` endpoint in `tests/backend/routes/test_chat.py`.

**Checkpoint**: User Stories 4, 1 AND 2 should all work independently.

---

## Phase 6: User Story 3 - Personalized Chatbot Interaction (Priority: P2)

**Goal**: Tailor chatbot responses based on user preferences stored in Neon Postgres.

**Independent Test**: A user can set a personalization preference (e.g., "Beginner" difficulty), ask a question, and verify the chatbot's answer reflects that style.

### Implementation for User Story 3

- [ ] T039 [US3] Implement logic in `backend/agents/gemini_agent.py` to dynamically construct `{user_profile_summary}` and `{personalization_style_description}` from `neon_preferences_tool` output.
- [ ] T040 [US3] Implement fallback logic for when personalization is disabled or profile data is missing in `backend/agents/gemini_agent.py`.

### Tests for User Story 3

- [ ] T041 [P] [US3] Write unit tests for personalization logic in `tests/backend/agents/test_gemini_agent.py`.

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 7: Frontend Integration (ChatKit + Docusaurus)

**Purpose**: Integrate the chatbot UI into the Docusaurus book and enable user interaction.

- [ ] T042 [P] Install ChatKit SDK in `frontend/`.
- [ ] T043 [P] Create `<FloatingChatbot />` component in `frontend/src/components/ChatbotComponent.js/tsx` to initialize ChatKit UI.
- [ ] T044 [P] Implement Docusaurus/JavaScript listener for text selection in `frontend/src/components/ChatbotComponent.js/tsx`.
- [ ] T045 [P] Create `<AskFromSelection />` UI element in `frontend/src/components/ChatbotComponent.js/tsx` that appears on text highlight.
- [ ] T046 [P] Implement logic to send selected text to `/api/v1/chat/selected` when "Ask AI about this" is clicked.
- [ ] T047 [P] Configure ChatKit to connect to FastAPI streaming endpoints for `/api/v1/chat` and `/api/v1/chat/selected`.
- [ ] T048 [P] Implement incremental markdown rendering of model output in `<FloatingChatbot />`.
- [ ] T049 [P] Implement display of citations (chunk IDs) in the ChatKit UI.
- [ ] T050 [P] Implement clear UI feedback for API loading states (spinners), errors (toast messages), and empty responses in `<FloatingChatbot />`.

---

## Phase 8: DevOps & Deployment

**Purpose**: Ensure the system can be deployed and managed in production environments.

- [ ] T051 Create `Dockerfile` for the FastAPI backend in `backend/`.
- [ ] T052 Create `railway.json` or `render.yaml` for backend deployment on Railway/Render.
- [ ] T053 Configure Vercel deployment for the Docusaurus frontend.
- [ ] T054 Document environment variables and deployment steps in `README.md` and `docs/deployment.md`.

---

## Phase 9: Testing & Documentation

**Purpose**: Final verification, performance checks, and comprehensive documentation.

- [ ] T055 Run all backend unit and integration tests.
- [ ] T056 Manually test end-to-end functionality for Normal RAG mode.
- [ ] T057 Manually test end-to-end functionality for Selected-Text RAG mode.
- [ ] T058 Manually test user profile management (signup, signin, update).
- [ ] T059 Manually test personalization effects on chatbot responses.
- [ ] T060 Conduct performance testing against latency targets.
- [ ] T061 Conduct hallucination review for a sample set of responses.
- [ ] T062 Document the RAG chatbot system, API endpoints, and usage in `docs/chatbot_integration.md`.
- [ ] T063 Update main `README.md` with overview and setup instructions.

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
-   **User Stories (Phase 3-6)**: All depend on Foundational phase completion
    -   Stories can then proceed in parallel (if staffed) or sequentially in priority order.
-   **Frontend Integration (Phase 7)**: Primarily depends on Backend User Stories (Phase 3-6) being implemented.
-   **DevOps & Deployment (Phase 8)**: Depends on all core backend and frontend functionality being implemented.
-   **Testing & Documentation (Phase 9)**: Depends on all implementation phases being largely complete.

### User Story Dependencies

-   **User Story 4 (P2 - User Profile Management)**: Can start after Foundational (Phase 2). Blocking for User Story 3.
-   **User Story 1 (P1 - Normal RAG)**: Can start after Foundational (Phase 2). No direct blocking dependency on other stories but benefits from US4 for personalization.
-   **User Story 2 (P1 - Selected-Text RAG)**: Can start after Foundational (Phase 2). Builds on US1's agent setup.
-   **User Story 3 (P2 - Personalized Chatbot Interaction)**: Depends on User Story 4 (User Profile Management) for profile data.

### Within Each User Story

-   Tests MUST be written and FAIL before implementation.
-   Models/Schemas before CRUD/Service logic.
-   Tools before Agent integration.
-   Agent logic before Endpoint implementation.
-   Core implementation before integration.

### Parallel Opportunities

-   All tasks marked `[P]` can run in parallel within their phase/story.
-   Once the Foundational phase (Phase 2) is complete, User Stories 4, 1, and 2 can be worked on in parallel by different teams/developers. User Story 3 depends on User Story 4, so it would follow.
-   Within each user story, implementation and test tasks can be parallelized where marked.

---

## Parallel Example: Backend Setup (Phase 2)

```bash
# These can be done in parallel:
- [ ] T007 Implement CORS middleware in backend/utils/cors.py and integrate into backend/main.py
- [ ] T008 Implement logging utilities in backend/utils/logging.py
- [ ] T009 Implement custom exception handling and error response schemas in backend/utils/errors.py
```

## Parallel Example: User Story 4 (US4) Implementation

```bash
# These can be done in parallel:
- [ ] T016 [P] [US4] Define Pydantic models for user signup, signin, and profile update requests in backend/schemas/user.py
- [ ] T022 [P] [US4] Define neon_preferences_tool in backend/tools/neon_user_profile_tool.py
```

---

## Implementation Strategy

### MVP First (User Story 1 & User Story 4)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 4 (User Profile Management) - provides authentication
4.  Complete Phase 4: User Story 1 (Normal RAG) - provides core RAG
5.  **STOP and VALIDATE**: Test User Stories 4 & 1 independently. This provides a functional, albeit unpersonalized, RAG chatbot with user management.
6.  Deploy/demo if ready.

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready.
2.  Add User Story 4 → Test independently → Provides user management.
3.  Add User Story 1 → Test independently → Provides core RAG functionality.
4.  Add User Story 2 → Test independently → Adds selected-text RAG functionality.
5.  Add User Story 3 → Test independently → Adds personalization.
6.  Complete Frontend Integration, DevOps, Testing & Documentation.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together.
2.  Once Foundational is done:
    -   Developer A: User Story 4 (User Profile Management)
    -   Developer B: User Story 1 (Normal RAG)
    -   Developer C: User Story 2 (Selected-Text RAG)
    -   Developer D (once US4 is done): User Story 3 (Personalization)
3.  Dedicated team for Frontend Integration, DevOps, and overall Testing/Documentation.

---

## Summary

Once all tasks are completed, the development team will have a fully functional AI-Native RAG Chatbot System integrated into the Docusaurus textbook. This includes:
-   A FastAPI backend providing authentication (JWT), user profile management, and RAG/selected-text chat endpoints.
-   Integration with Qdrant Cloud for vector search and Neon Postgres for user data.
-   An OpenAI Agents SDK-powered agent using Gemini models and custom function tools.
-   A ChatKit-based frontend component embedded in Docusaurus, supporting normal RAG and highlight-to-chat interactions, with streaming responses and clear UI feedback.
-   Robust DevOps setup for deployment on Railway/Render (backend) and Vercel (frontend).
-   Comprehensive testing and documentation, ensuring a production-ready system.

## Execution Order

The tasks should be executed in the sequence they are listed above, respecting the dependencies outlined in the "Dependencies & Execution Order" section. Within each phase and user story, tasks marked `[P]` can be executed in parallel. The foundational phase (Phase 2) is a critical blocking step before any user story implementation begins. User Stories should ideally be implemented in the order of their phases (US4, then US1, then US2, then US3) for a logical build-up of functionality, but P1 stories (US1, US2) can be prioritized depending on immediate MVP goals after US4.
