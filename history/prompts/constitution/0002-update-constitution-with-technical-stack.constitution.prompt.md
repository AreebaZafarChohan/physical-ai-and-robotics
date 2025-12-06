---
id: 0002
title: Update Constitution with Technical Stack
stage: constitution
date: 2025-12-06
surface: agent
model: gemini-pro
feature: none
branch: master
user: {{USER}}
command: /sp.constitution
labels: ["constitution", "technical-stack"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution 

You have an existing **Constitution document** for the "Physical AI & Humanoid Robotics" textbook. The current document defines modules, chapters, exercises, and high-level structure, but it **does not specify programming languages, markup, frameworks, deployment platforms, or AI-native integration stack**.

Your task is to **update the Constitution** by adding all missing implementation details, without changing the existing structure or learning objectives.

**Update Requirements:**
1. Add **Programming / Markup Languages** section:
   - Markdown for book content
   - Python for ROS 2, Isaac Sim, and agent code
   - YAML for ROS 2 launch files
   - Optional C# for Unity scripts (if used)

2. Add **Frameworks / Platforms** section:
   - Spec-Kit Plus for structuring AI-native content
   - Docusaurus for book hosting
   - Claude Code for content generation and task management
   - GitHub Pages / Vercel for deployment

3. Add **Chatbot Integration Stack** section (for future Part 2):
   - RAG Chatbot with FastAPI backend
   - Neon Serverless Postgres as database
   - Qdrant Cloud as vector store
   - OpenAI Agents / ChatKit SDK for agent skills
   - Features: answer user queries, personalize content, language toggle (English/Urdu)

4. Add **AI-native Features** section:
   - Personalization button at start of each chapter
   - Urdu translation button
   - Placeholders for ROS 2 / Isaac / agent code
   - Diagram placeholders for URDF, Digital Twin, sensors, etc.

5. Ensure that the **existing content, modules, chapters, exercises, and capstone projects remain unchanged**.
6. Update the **Resources / References** section to include implementation notes, links, and relevant APIs.
7. Maintain **semantic versioning**, authorship, and amendment procedure.

**Output**: A fully updated Constitution document ready to guide **Claude Code + Spec-Kit Plus workflow**.

## Response snapshot

The `constitution.md` file has been updated to version 1.1.0 to include the technical implementation stack.

- **File**: `.specify/memory/constitution.md`
- **Version**: `1.1.0` (minor bump for new sections)
- **Changes**: Added the "Implementation Stack" section with details on languages, frameworks, and future chatbot integration.

## Outcome

- ✅ Impact: The constitution now provides a complete technical foundation for the project.
- 🧪 Tests: No tests were run.
- 📁 Files: 1 file modified.
- 🔁 Next prompts: `/sp.plan` can now use this detailed technical context.
- 🧠 Reflection: The user's request was clear and the update was straightforward. The version was bumped according to semantic versioning rules for adding new features/sections.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): n/a