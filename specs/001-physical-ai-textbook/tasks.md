# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md, spec.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure.

- [ ] T001 [P] Initialize a new Docusaurus classic project in the repository root.
- [ ] T002 [P] Create the module directories inside the `docs/` folder as specified in `plan.md`.

## Phase 2: Foundational - Create Chapter Files (User Story 1)

**Goal**: Create the file structure for all textbook chapters.
**Independent Test**: All chapter files exist in the correct directory, and the Docusaurus dev server runs without errors.

### Implementation for User Story 1

- [ ] T003 [P] [US1] Create chapter file `docs/01-module-ros/01-intro-to-ros2.md` with a placeholder title.
- [ ] T004 [P] [US1] Create chapter file `docs/01-module-ros/02-nodes-topics-services.md` with a placeholder title.
- [ ] T005 [P] [US1] Create chapter file `docs/01-module-ros/03-rclpy-integration.md` with a placeholder title.
- [ ] T006 [P] [US1] Create chapter file `docs/01-module-ros/04-urdf-for-humanoids.md` with a placeholder title.
- [ ] T007 [P] [US1] Create chapter file `docs/02-module-digital-twin/01-gazebo-setup.md` with a placeholder title.
- [ ] T008 [P] [US1] Create chapter file `docs/02-module-digital-twin/02-physics-simulation.md` with a placeholder title.
- [ ] T009 [P] [US1] Create chapter file `docs/02-module-digital-twin/03-sensor-simulation.md` with a placeholder title.
- [ ] T010 [P] [US1] Create chapter file `docs/02-module-digital-twin/04-unity-rendering.md` with a placeholder title.
- [ ] T011 [P] [US1] Create chapter file `docs/03-module-nvidia-isaac/01-isaac-sim-intro.md` with a placeholder title.
- [ ] T012 [P] [US1] Create chapter file `docs/03-module-nvidia-isaac/02-isaac-ros-vslam.md` with a placeholder title.
- [ ] T013 [P] [US1] Create chapter file `docs/03-module-nvidia-isaac/03-nav2-path-planning.md` with a placeholder title.
- [ ] T014 [P] [US1] Create chapter file `docs/03-module-nvidia-isaac/04-reinforcement-learning.md` with a placeholder title.
- [ ] T015 [P] [US1] Create chapter file `docs/04-module-vla/01-voice-to-action.md` with a placeholder title.
- [ ] T016 [P] [US1] Create chapter file `docs/04-module-vla/02-cognitive-planning-llms.md` with a placeholder title.
- [ ] T017 [P] [US1] Create chapter file `docs/04-module-vla/03-gpt-integration.md` with a placeholder title.
- [ ] T018 [P] [US1] Create chapter file `docs/05-capstone-project/01-autonomous-humanoid.md` with a placeholder title.

---

## Phase 3: Interactive Placeholders (User Stories 2 & 3)

**Goal**: Add non-functional UI buttons for future interactive features.
**Independent Test**: The placeholder buttons are visible on each chapter page. Clicking them shows a "coming soon" message.

### Implementation for User Stories 2 & 3

- [ ] T019 [P] [US2] Create a reusable React component `src/components/PlaceholderButton.js` for the interactive placeholders. It should accept a `label` prop and show a tooltip on click.
- [ ] T020 [US2, US3] Insert placeholder button components for RAG Chatbot, Personalization, and Urdu Translation into all 17 chapter markdown files created in Phase 2.

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Final cleanup and configuration.

- [ ] T021 Configure the sidebar in `sidebars.js` to reflect the module and chapter structure.
- [ ] T022 Update `docusaurus.config.js` with the book's title, description, and author.
- [ ] T023 [P] Review the generated site for any broken links or layout issues.

---

## Dependencies & Execution Order

- **Phase 1** must be completed before any other phase.
- **Phase 2** must be completed before Phase 3.
- **Phase N** should be done last.

Tasks within each phase can largely be run in parallel, especially the file creation tasks in Phase 2.

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Create Chapter Files
3. Complete Phase N: Configure sidebar and site metadata.
4. **STOP and VALIDATE**: The basic book structure is navigable on the Docusaurus site.

### Incremental Delivery

1.  Deliver MVP.
2.  Add Phase 3 to implement the placeholder buttons.
