# Implementation Plan: Docusaurus Urdu Translation with i18n

**Branch**: `012-docusaurus-i18n-urdu` | **Date**: 2026-01-01 | **Spec**: [012-docusaurus-i18n-urdu/spec.md](./spec.md)
**Input**: Feature specification from `/specs/012-docusaurus-i18n-urdu/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the implementation of Urdu translation for a Docusaurus-based documentation site using i18n. The approach involves:
1. Setting up Docusaurus i18n infrastructure for English and Urdu
2. Creating an automated translation workflow using Claude AI via SpecKitPlus
3. Implementing RTL layout support for Urdu
4. Developing a language switcher UI component
5. Maintaining English as the source of truth with automated sync to Urdu

## Technical Context

**Language/Version**: TypeScript, JavaScript
**Primary Dependencies**: Docusaurus v2+, React, Tailwind CSS, Node.js
**Storage**: File-based (Markdown/MDX files), potentially Neon Serverless Postgres for future chatbot features
**Testing**: Jest for unit tests, potentially Cypress for E2E tests
**Target Platform**: Web (Static Site Generation)
**Project Type**: Web application (Docusaurus-based documentation site)
**Performance Goals**: Fast page load times, efficient translation processing (95% success within 15 min for updates)
**Constraints**: Must maintain existing English documentation functionality, support RTL layout for Urdu, handle up to 500 documents and 1M words
**Scale/Scope**: Up to 500 documents, 1M total words, 10 daily updates

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance with Constitution
- **Implementation Stack**:
  - Uses Docusaurus (as specified in constitution) for static site generation
  - Uses React, TypeScript, and Tailwind CSS (as specified in constitution)
  - Translation will be handled via AI integration (SpecKitPlus + Claude) as per constitution
- **AI-native Features**:
  - Urdu Translation Button is a core requirement from constitution
  - Future chatbot integration planned with Neon Serverless Postgres (as per constitution)
- **Module Structure**:
  - This i18n feature will be integrated into the existing module structure
  - All documentation content will continue to be in Markdown format

### Gates
- **GATE 1: Technology Alignment** - PASSED - Uses technologies specified in constitution
- **GATE 2: Feature Alignment** - PASSED - Urdu translation is explicitly mentioned in constitution
- **GATE 3: Architecture Compliance** - PASSED - Follows the planned architecture for AI-native features
- **GATE 4: Documentation Format** - PASSED - Maintains Markdown as primary content format

### Post-Design Re-evaluation
After implementing the design artifacts (data-model.md, contracts/, quickstart.md), the solution continues to comply with the constitution:

- **Data Model**: Aligns with the file-based storage approach specified in the constitution (Markdown/MDX files)
- **API Contracts**: Support the AI integration requirements (SpecKitPlus + Claude) as specified
- **Quickstart Guide**: Maintains the technology stack (Docusaurus, React, TypeScript, Tailwind CSS) as specified
- **RTL Implementation**: Adds necessary UI enhancements without changing core architecture
- **Language Switcher**: Integrates with the existing Docusaurus navigation as an AI-native feature

## Project Structure

### Documentation (this feature)

```text
specs/012-docusaurus-i18n-urdu/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus project with i18n support
docs/
├── en/                  # English documentation (existing content)
│   ├── module-1/
│   ├── module-2/
│   └── ...
└── ur/                  # Urdu documentation (to be generated)
    ├── module-1/
    ├── module-2/
    └── ...
i18n/
├── en/                  # English locale config
└── ur/                  # Urdu locale config (to be created)
    └── docusaurus-theme-classic/
        └── translations.json
src/
├── components/          # Custom React components
│   └── LanguageSwitcher/ # Language switcher component
├── css/                 # Custom CSS/SCSS files
│   └── rtl.css          # RTL-specific styles
├── pages/               # Custom pages
└── theme/               # Custom theme components
    └── UrduLayout/      # RTL layout wrapper
static/                  # Static assets
docusaurus.config.ts     # Docusaurus configuration with i18n settings
package.json             # Project dependencies
tsconfig.json            # TypeScript configuration
```

**Structure Decision**: This is a Docusaurus-based documentation site with i18n capabilities. The structure follows Docusaurus conventions with separate directories for documentation content in different languages (docs/en/, docs/ur/), locale-specific configurations (i18n/en/, i18n/ur/), and custom components for language switching and RTL support. The implementation will add Urdu-specific directories and files while preserving the existing English documentation structure.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
