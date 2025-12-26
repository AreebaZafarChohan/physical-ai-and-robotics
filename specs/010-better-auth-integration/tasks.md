# Tasks: Signup & Signin Integration

**Input**: Design documents from `/specs/010-better-auth-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/auth_api.yaml

**Tests**: The tasks below include test tasks where appropriate.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
-   Include exact file paths in descriptions

## Path Conventions

-   **Web app**: `backend/src/`, `frontend/src/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

[x] T001 Create project structure for `backend/` and `frontend/`
[x] T002 Initialize Python backend project with FastAPI and dependencies in `backend/`
[x] T003 Initialize Node.js frontend project with React and dependencies in `frontend/`
[x] T004 [P] Configure backend linting (e.g., Black, Flake8) and formatting (e.g., isort) in `backend/`
[x] T005 [P] Configure frontend linting (e.g., ESLint) and formatting (e.g., Prettier) in `frontend/`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

[x] T006 Setup database connection and ORM (e.g., SQLAlchemy/Pydantic) in `backend/src/`
[x] T007 Implement base User model with `id`, `email`, `password_hash`, `oauth_provider_ids` in `backend/src/models/user.py`
[x] T008 Implement PersonalizationData model with `software_background`, `hardware_background` in `backend/src/models/personalization.py`
[x] T009 Setup database migrations (e.g., Alembic) in `backend/`
[x] T010 Implement session management mechanism (e.g., JWT token handling) in `backend/src/utils/auth.py`
[x] T011 Configure shared error handling and logging in `backend/src/utils/errors.py`, `backend/src/utils/logger.py`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - User Registration (Priority: P1) 🎯 MVP

**Goal**: Enable new users to register via email/password or OAuth and store their background info.
**Independent Test**: A new user can successfully register, their data is stored, and they are automatically logged in.

### Implementation for User Story 1

[x] T012 [P] [US1] Implement password hashing utility in `backend/src/utils/security.py`
[x] T013 [P] [US1] Create User service for registration logic in `backend/src/services/user_service.py`
[x] T014 [P] [US1] Implement `POST /signup` endpoint in `backend/src/api/auth.py` (email/password registration)
[x] T015 [P] [US1] Implement `POST /signup/oauth` endpoint in `backend/src/api/auth.py` (OAuth registration)
[x] T016 [P] [US1] Design and create Signup form component in `frontend/src/components/SignupForm.tsx`
[x] T017 [P] [US1] Integrate email/password registration with backend in `frontend/src/services/auth.ts`
[x] T018 [P] [US1] Integrate OAuth registration with backend in `frontend/src/services/auth.ts`
[x] T019 [P] [US1] Implement data validation for `software_background` and `hardware_background` in `backend/src/validation/user_validation.py`
[x] T020 [US1] Create registration page in `frontend/src/pages/RegistrationPage.tsx`, using SignupForm.
[x] T021 [US1] Ensure background data is collected and sent during signup in `frontend/src/components/SignupForm.tsx`
[x] T022 [US1] Handle existing email registration error and display appropriate message/links in `frontend/src/components/SignupForm.tsx`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - User Login (Priority: P1)

**Goal**: Enable existing users to log in via email/password or OAuth and maintain session state.
**Independent Test**: An existing user can successfully log in and their session is maintained.

### Implementation for User Story 2

[x] T023 [P] [US2] Create User service for login logic in `backend/src/services/user_service.py`
[x] T024 [P] [US2] Implement `POST /login` endpoint in `backend/src/api/auth.py` (email/password login)
[x] T025 [P] [US2] Design and create Login form component in `frontend/src/components/LoginForm.tsx`
[x] T026 [P] [US2] Integrate email/password login with backend in `frontend/src/services/auth.ts`
[x] T027 [P] [US2] Implement OAuth login flow in `frontend/src/services/auth.ts`
[x] T028 [US2] Create login page in `frontend/src/pages/LoginPage.tsx`, using LoginForm.
[x] T029 [US2] Implement session token storage and retrieval in `frontend/src/utils/session.ts`
[x] T030 [US2] Implement session timeout handling (modal dialog) in `frontend/src/components/SessionTimeoutModal.tsx`
[x] T031 [US2] Handle invalid credentials error and display appropriate message in `frontend/src/components/LoginForm.tsx`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Personalized Content Access (Priority: P2)

**Goal**: Allow authenticated users to access content tailored to their background.
**Independent Test**: Authenticated user accesses content which demonstrates personalization based on their background.

### Implementation for User Story 3

[x] T032 [P] [US3] Implement `GET /user/profile` endpoint in `backend/src/api/user.py`
[x] T033 [P] [US3] Implement `GET /user/personalization-data` endpoint in `backend/src/api/user.py`
[x] T034 [P] [US3] Create a service for fetching user profile and personalization data in `frontend/src/services/user_profile.ts`
[x] T035 [US3] Implement personalization logic hooks on frontend to consume user background data in `frontend/src/hooks/usePersonalization.ts`
[x] T036 [US3] Integrate personalization hooks into example book chapter component (e.g., `frontend/src/components/BookChapter.tsx`) to demonstrate dynamic content adjustment.

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

[x] T037 [P] Add comprehensive unit tests for backend services in `backend/tests/unit/`
[x] T038 [P] Add integration tests for backend API endpoints in `backend/tests/integration/`
[x] T039 [P] Conduct performance testing for login and registration endpoints in `backend/tests/performance/test_auth_performance.py`
[x] T040 [P] Add unit/integration tests for frontend components/services in `frontend/tests/`
[x] T041 [P] Implement `POST /logout` endpoint in `backend/src/api/auth.py`
[x] T042 [P] Implement logout functionality in `frontend/src/services/auth.ts` and UI.
[x] T043 Review and refine API documentation based on `contracts/auth_api.yaml`.
[x] T044 Perform security review (e.g., input sanitization, secure header configuration).
[x] T045 Run quickstart.md validation to ensure setup and basic functionality work.

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
-   **User Stories (Phase 3+)**: All depend on Foundational phase completion
    -   User stories can then proceed in parallel (if staffed)
    -   Or sequentially in priority order (P1 → P2 → P3)
-   **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
-   **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

-   Tests (if included) MUST be written and FAIL before implementation
-   Models before services
-   Services before endpoints
-   Core implementation before integration
-   Story complete before moving to next priority

### Parallel Opportunities

-   All Setup tasks marked [P] can run in parallel
-   All Foundational tasks marked [P] can run in parallel (within Phase 2)
-   Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
-   All tests for a user story marked [P] can run in parallel
-   Models within a story marked [P] can run in parallel
-   Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "- [ ] T010 [P] [US1] Contract test for POST /signup in backend/tests/contract/test_auth.py"
Task: "- [ ] T011 [P] [US1] Integration test for email/password registration in backend/tests/integration/test_auth.py"

# Launch all models for User Story 1 together:
Task: "- [ ] T012 [P] [US1] Implement password hashing utility in backend/src/utils/security.py"
Task: "- [ ] T013 [P] [US1] Create User service for registration logic in backend/src/services/user_service.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1
4.  **STOP and VALIDATE**: Test User Story 1 independently
5.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3.  Add User Story 2 → Test independently → Deploy/Demo
4.  Add User Story 3 → Test independently → Deploy/Demo
5.  Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together
2.  Once Foundational is done:
    -   Developer A: User Story 1
    -   Developer B: User Story 2
    -   Developer C: User Story 3
3.  Stories complete and integrate independently

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
