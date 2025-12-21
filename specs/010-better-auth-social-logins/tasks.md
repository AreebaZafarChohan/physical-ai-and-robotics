# Tasks: Better-Auth Integration with Social Logins

**Input**: Design documents from `/specs/010-better-auth-social-logins/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 [P] Configure Better-Auth API keys and secrets in `backend/.env`.
- [ ] T002 [P] Configure Neon Postgres connection string in `backend/.env`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [ ] T003 Create User model in `backend/src/models/user.py` based on `data-model.md`.
- [ ] T004 Setup database connection and session management in `backend/src/database.py`.

---

## Phase 3: User Story 1 - New User Signup with Email (Priority: P1) 🎯 MVP

**Goal**: Allow new users to create an account using their email and password and provide their background information.

**Independent Test**: A new user can successfully sign up, and their data is correctly stored in the database.

### Implementation for User Story 1

- [ ] T005 [US1] Implement the `/signup` endpoint in `backend/src/api/auth.py`.
- [ ] T006 [US1] Create a `SignupForm` component in `frontend/src/components/SignupForm/index.tsx`.
- [ ] T007 [US1] Add a new signup page at `frontend/src/pages/signup.tsx` that uses the `SignupForm` component.
- [ ] T008 [US1] Implement form validation for the signup form.
- [ ] T009 [US1] Connect the signup form to the backend `/signup` endpoint.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Existing User Login with Email (Priority: P1)

**Goal**: Allow existing users to log in to their accounts using their email and password.

**Independent Test**: An existing user can successfully log in and receive a session token.

### Implementation for User Story 2

- [ ] T010 [US2] Implement the `/signin` endpoint in `backend/src/api/auth.py`.
- [ ] T011 [US2] Create a `SigninForm` component in `frontend/src/components/SigninForm/index.tsx`.
- [ ] T012 [US2] Add a new signin page at `frontend/src/pages/signin.tsx` that uses the `SigninForm` component.
- [ ] T013 [US2] Implement form validation for the signin form.
- [ ] T014 [US2] Connect the signin form to the backend `/signin` endpoint.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - New/Existing User Login with Social Provider (Priority: P2)

**Goal**: Allow users to sign up or log in using their Google or GitHub accounts.

**Independent Test**: A user can successfully authenticate using Google or GitHub, and a new user is created if one does not exist.

### Implementation for User Story 3

- [ ] T015 [US3] Implement the `/auth/google` and `/auth/github` redirect endpoints in `backend/src/api/auth.py`.
- [ ] T016 [US3] Implement the `/auth/callback` endpoint in `backend/src/api/auth.py` to handle the social login callback.
- [ ] T017 [US3] [P] Add Google and GitHub social login buttons to the `SignupForm` and `SigninForm` components.
- [ ] T018 [US3] Implement the optional prompt for background information after a new user signs up via a social provider.

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T019 [P] Create a React context for authentication in `frontend/src/contexts/AuthContext.tsx` to manage user session.
- [ ] T020 [P] Write unit tests for the backend API endpoints in `backend/tests/unit/`.
- [ ] T021 Write integration tests for the authentication flows in `backend/tests/integration/`.
- [ ] T022 Write E2E tests for the signup and signin flows in `tests/e2e/`.
- [ ] T023 [P] Update `README.md` with instructions on how to run the application with the new authentication feature.
- [ ] T024 Run `quickstart.md` validation.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion.
- **Polish (Phase 6)**: Depends on all user stories being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2).
- **User Story 2 (P1)**: Can start after Foundational (Phase 2).
- **User Story 3 (P2)**: Can start after Foundational (Phase 2).

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel.
- Once Foundational phase completes, all user stories can start in parallel.
- All Polish tasks marked [P] can run in parallel.
