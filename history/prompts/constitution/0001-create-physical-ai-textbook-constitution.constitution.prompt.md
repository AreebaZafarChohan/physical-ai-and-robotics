---
id: 0001
title: Create Physical AI Textbook Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: gemini-pro
feature: none
branch: master
user: {{USER}}
command: /sp.constitution
labels: ["constitution", "spec-kit-plus", "robotics"]
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

/sp.constitution You are tasked with creating a Spec-Kit Plus textbook for a Physical AI & Humanoid Robotics course. Generate a complete `constitution.md` file that follows Spec-Kit Plus structure and covers the following requirements:

1. **Book Metadata**  
   - Title: "Physical AI & Humanoid Robotics"  
   - Author: "[Your Name]"  
   - Description: "A practical guide to Physical AI, Humanoid Robotics, ROS 2, Gazebo, NVIDIA Isaac, and integrating AI agents with real-world robots."  
   - Version: "1.0.0"

2. **Modules & Chapters**  
   Create chapters and sections for the course following this structure:

   **Module 1: The Robotic Nervous System (ROS 2)**  
   - Introduction to ROS 2  
   - ROS 2 Nodes, Topics, Services  
   - rclpy integration for Python Agents  
   - URDF for humanoid robots  

   **Module 2: The Digital Twin (Gazebo & Unity)**  
   - Gazebo setup and simulation basics  
   - Physics, gravity, and collision simulation  
   - Sensor simulation: LiDAR, Depth Cameras, IMUs  
   - Unity high-fidelity rendering & interaction  

   **Module 3: The AI-Robot Brain (NVIDIA Isaac)**  
   - NVIDIA Isaac Sim introduction  
   - Isaac ROS: VSLAM and navigation  
   - Nav2 path planning for humanoid movement  
   - Reinforcement learning & sim-to-real  

   **Module 4: Vision-Language-Action (VLA)**  
   - Voice-to-Action using OpenAI Whisper  
   - Cognitive planning with LLMs  
   - Integration of GPT models for robotics  

   **Capstone Project**  
   - Autonomous Humanoid Robot: voice command → planning → navigation → object recognition → manipulation  

3. **Features & Integrations**  
   - Add **RAG chatbot integration points** for each chapter for user queries.  
   - Add placeholders for **personalized content per user**.  
   - Add placeholders for **Urdu translation toggle** for each chapter.  
   - Include **links to GitHub Pages deployment and resources**.  

4. **Spec-Kit Plus Specifics**  
   - Use proper Spec-Kit Plus headings: `#`, `##`, `###` for modules, chapters, and sections.  
   - Add **`[RAG Chatbot]`, `[Personalization]`, `[Urdu Translation]`** notes where integration is required.  
   - Include **example exercises**, **capstone checkpoints**, and **hardware requirements notes**.  

5. **Output**  
   - Provide a ready-to-use `constitution.md` file that can be directly used with Spec-Kit Plus CLI to generate a book structure.  
   - Organize content in clear, educational style suitable for students with intermediate AI/robotics background.  
   - Add tables where appropriate (e.g., Hardware Requirements, Lab Options).  

**Instruction to Claude Code CLI:**  
"Generate a Spec-Kit Plus `constitution.md` file for a Physical AI & Humanoid Robotics textbook including the above modules, chapters, RAG chatbot integration, personalization, and Urdu translation placeholders. Organize sections with headings, tables, and bullet points for clarity."

## Response snapshot

I have successfully created the `constitution.md` file located at `.specify/memory/constitution.md`.

- **Version**: 1.0.0
- **Summary**: The file outlines the complete structure for the "Physical AI & Humanoid Robotics" textbook, including all specified modules, chapters, and integration points.
- **Next Steps**: You can now use this constitution file with your Spec-Kit Plus CLI.
- **Commit Message Suggestion**: `docs: create v1.0.0 of Physical AI & Robotics textbook constitution`

## Outcome

- ✅ Impact: Created the initial constitution for the project.
- 🧪 Tests: No tests were run as this was a documentation change.
- 📁 Files: 1 file created: `.specify/memory/constitution.md`.
- 🔁 Next prompts: The user can now proceed with other Spec-Kit Plus commands.
- 🧠 Reflection: The user provided a very detailed prompt which made it easy to generate the required file. The process of checking other templates and creating a PHR was followed.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): n/a