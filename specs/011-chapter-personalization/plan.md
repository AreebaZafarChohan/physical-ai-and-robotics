# Implementation Plan: Chapter Content Personalization

**Branch**: `011-chapter-personalization` | **Date**: 2025-12-23 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/011-chapter-personalization/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Enable logged-in users to personalize chapter content based on their software and hardware background. The system will display a "Personalize Content" button at the start of each Docusaurus chapter, which when clicked fetches personalized content from a backend API based on the user's profile. The content will be dynamically updated without page reload, using predefined markers in the content that get replaced based on personalization rules stored in a dedicated database table.

## Technical Context

**Language/Version**: Python 3.11 (backend), TypeScript (frontend), Markdown (content)
**Primary Dependencies**: FastAPI (backend), React (frontend), Neon Postgres (database), Docusaurus (content framework)
**Storage**: Neon Serverless Postgres database with dedicated personalization rules table, using JSONB fields for flexible matching criteria and content variations
**Testing**: pytest for backend API testing, Jest for frontend component testing
**Target Platform**: Web application (Linux server)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <1 second content personalization response time (p95 latency)
**Constraints**: <200ms p95, must maintain existing RAG chatbot functionality, seamless UX without page reloads
**Scale/Scope**: Up to 10k users, extendable personalization rules for multiple software/hardware backgrounds

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution file, this feature aligns with the following principles:
- Uses the specified implementation stack (Python, FastAPI, Docusaurus, Neon Postgres)
- Integrates with the planned chatbot features
- Supports AI-native features (personalization button)
- Follows the module structure defined in the constitution
- Maintains the textbook content format (Markdown)

## Project Structure

### Documentation (this feature)

```text
specs/011-chapter-personalization/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── user_profile.py
│   │   ├── personalization_rules.py
│   │   └── content.py
│   ├── services/
│   │   ├── personalization_service.py
│   │   ├── user_service.py
│   │   └── content_service.py
│   └── api/
│       ├── routes/
│       │   ├── personalization.py
│       │   └── user_profile.py
│       └── main.py
└── tests/

frontend/
├── src/
│   ├── components/
│   │   ├── PersonalizeContentButton.tsx
│   │   └── PersonalizedContent.tsx
│   ├── services/
│   │   └── api.ts
│   └── utils/
│       └── contentParser.ts
└── tests/
```

**Structure Decision**: Web application (frontend + backend) structure selected to match the specified implementation stack (FastAPI backend with React/TypeScript frontend in Docusaurus environment). Backend handles personalization logic and API, while frontend handles UI components and content replacement.

## Phase 1: Design & Contracts Completed

- **Data Model**: Created data model for personalization feature (data-model.md)
- **API Contracts**: Defined API endpoints and contracts (contracts/personalization-api.yaml)
- **Quickstart Guide**: Created developer quickstart guide (quickstart.md)
- **Agent Context**: Updated Qwen agent context with new technology stack

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
