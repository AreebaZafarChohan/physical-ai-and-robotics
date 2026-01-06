---
id: 0004
title: Tasks Docusaurus i18n Urdu
stage: tasks
date: 2026-01-01
surface: agent
model: Qwen
feature: 012-docusaurus-i18n-urdu
branch: 012-docusaurus-i18n-urdu
user: qwen
command: /sp.tasks
labels: ["i18n", "docusaurus", "translation", "urdu", "rtl", "tasks"]
links:
  spec: /specs/012-docusaurus-i18n-urdu/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - /specs/012-docusaurus-i18n-urdu/tasks.md
tests:
 - /specs/012-docusaurus-i18n-urdu/tasks.md
---

## Prompt

```
/sp.tasks
Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.

Context for task generation: Break down the i18n + English → Urdu translation implementation for a Docusaurus (React + Tailwind + TypeScript) project into concrete, executable tasks. Tasks should include: - Enabling Docusaurus i18n configuration - Creating Urdu locale structure - Extracting existing English content - Automating translation via Claude (SpecKitPlus workflow) - Validating translated MDX and JSON files - Implementing RTL support (HTML dir, CSS, Tailwind config) - Adding language switcher to navbar - Testing language switching and layout correctness - Handling future content updates and re-translation Each task should have: - Clear objective - Expected output - Dependencies (if any) Task granularity: - Fine-grained enough that each task can be done independently - Suitable for CLI-driven or script-driven execution Exclude: - Vague tasks like “test everything” - Non-technical tasks - Any feature not directly related to translation or i18n

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.
```

## Response snapshot

```
---

description: "Task list for Docusaurus Urdu Translation Implementation"
---

# Tasks: Docusaurus Urdu Translation with i18n

**Input**: Design documents from `/specs/012-docusaurus-i18n-urdu/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: [ID] [P?] [Story] Description

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: src/, tests/ at repository root
- **Web app**: backend/src/, frontend/src/
- **Mobile**: api/src/, ios/src/ or android/src/
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project structure per implementation plan
- [ ] T002 Initialize environment variables for Claude API in .env file
- [ ] T003 [P] Configure linting and formatting tools for TypeScript and MDX

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [ ] T004 Configure Docusaurus i18n settings in docusaurus.config.ts
- [ ] T005 [P] Create directory structure for Urdu content (docs/ur/)
- [ ] T006 [P] Set up Tailwind CSS RTL configuration
- [ ] T007 Create basic translation CLI tool in scripts/translate.mjs
- [ ] T008 Configure HTML direction attribute for Urdu locale
- [ ] T009 Set up API endpoints for translation management

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - View Translated Content (Priority: P1) 🎯 MVP

**Goal**: Enable users to switch documentation language to Urdu and view translated content

**Independent Test**: Can be fully tested by navigating to an English documentation page, switching the language to Urdu via the UI, and verifying the content is displayed in Urdu.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T010 [P] [US1] Contract test for language switching API in tests/contract/test_language_switching.py
- [ ] T011 [P] [US1] Integration test for language switching functionality in tests/integration/test_language_switching.py

### Implementation for User Story 1

- [ ] T012 [P] [US1] Create LanguageSwitcher component in src/components/LanguageSwitcher/index.tsx
- [ ] T013 [P] [US1] Create UrduLayout wrapper in src/theme/UrduLayout/index.tsx
- [ ] T014 [US1] Add language switcher to navbar in src/theme/Navbar/index.tsx
- [ ] T015 [US1] Implement locale detection and routing in src/utils/locale.ts
- [ ] T016 [US1] Add CSS for RTL support in src/css/rtl.css
- [ ] T017 [US1] Create initial Urdu translation files for home page in docs/ur/

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Automated Translation Workflow (Priority: P2)

**Goal**: Create an automated process to translate new or updated English documentation into Urdu using AI

**Independent Test**: Can be fully tested by adding new English content or updating existing English content, triggering the automated translation script/CLI, and then verifying that corresponding Urdu translations are generated or updated in the Docusaurus i18n structure.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T018 [P] [US2] Contract test for translation API in tests/contract/test_translation_api.py
- [ ] T019 [P] [US2] Integration test for automated translation workflow in tests/integration/test_translation_workflow.py

### Implementation for User Story 2

- [ ] T020 [P] [US2] Create TranslationJob model in src/models/TranslationJob.ts
- [ ] T021 [US2] Implement translation service in src/services/translationService.ts
- [ ] T022 [US2] Create Claude API integration in src/services/claudeService.ts
- [ ] T023 [US2] Implement MDX parsing and translation in src/utils/mdxTranslator.ts
- [ ] T024 [US2] Add translation status tracking in src/services/translationTracker.ts
- [ ] T025 [US2] Create CLI command for translation in scripts/translate.mjs

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - RTL Support (Priority: P2)

**Goal**: Ensure Urdu documentation displays with Right-to-Left text direction and appropriate layout adjustments

**Independent Test**: Can be fully tested by switching the site language to Urdu and visually verifying that text flows from right to left and UI elements are adapted for RTL layout.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T026 [P] [US3] Contract test for RTL API in tests/contract/test_rtl_api.py
- [ ] T027 [P] [US3] Integration test for RTL layout in tests/integration/test_rtl_layout.py

### Implementation for User Story 3

- [ ] T028 [P] [US3] Update Tailwind CSS configuration for RTL in tailwind.config.js
- [ ] T029 [US3] Create RTL-specific utility classes in src/css/rtl.css
- [ ] T030 [US3] Modify Navbar for RTL layout in src/theme/Navbar/index.tsx
- [ ] T031 [US3] Update sidebar and navigation for RTL in src/theme/Sidebar/index.tsx
- [ ] T032 [US3] Adjust code block and text alignment for RTL in src/theme/CodeBlock/index.tsx
- [ ] T033 [US3] Test and validate RTL layout across all components

**Checkpoint**: All user stories should now be independently functional

---

[Add more user story phases as needed, following the same pattern]

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T034 [P] Documentation updates in docs/
- [ ] T035 Code cleanup and refactoring
- [ ] T036 Performance optimization across all stories
- [ ] T037 [P] Additional unit tests (if requested) in tests/unit/
- [ ] T038 Security hardening
- [ ] T039 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for language switching API in tests/contract/test_language_switching.py"
Task: "Integration test for language switching functionality in tests/integration/test_language_switching.py"

# Launch all models for User Story 1 together:
Task: "Create LanguageSwitcher component in src/components/LanguageSwitcher/index.tsx"
Task: "Create UrduLayout wrapper in src/theme/UrduLayout/index.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
```

## Outcome

- ✅ Impact: Created comprehensive task list for Docusaurus Urdu translation implementation, organized by user stories with dependencies and parallel execution opportunities
- 🧪 Tests: N/A (Task generation phase)
- 📁 Files: Created tasks.md with structured tasks for all user stories
- 🔁 Next prompts: Begin implementation of tasks starting with Phase 1 and 2
- 🧠 Reflection: The task list is well-structured according to the template, with clear phases, dependencies, and parallel execution opportunities. It follows the user story prioritization from the spec and includes specific file paths.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation of the generated tasks