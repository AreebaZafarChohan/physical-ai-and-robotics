---
title: 'Try with AI: "Cognitive Planning with Large Language Models (LLMs)"'
---

# Try with AI: Cognitive Planning with Large Language Models (LLMs)

This section provides curated prompts for you to explore Cognitive Planning with LLMs further with an AI assistant. Use these prompts with your favorite Large Language Model (LLM) to deepen your understanding or tackle planning challenges.

---

## Prompt 1: Designing an LLM Prompt for Humanoid Task Planning

**Goal**: Learn to craft effective prompts for LLM-driven robot planning.

**AI Prompt**:
"You are integrating an LLM into a humanoid robot's cognitive planning architecture. Design a comprehensive prompt (system prompt or few-shot examples) that would enable the LLM to generate a step-by-step plan for the humanoid to 'prepare a simple breakfast' (e.g., toast and coffee). Your prompt should:
1.  Define the robot's capabilities (e.g., has grippers, can move, has camera vision).
2.  Specify available low-level actions the robot can execute (e.g., `pick_up(object)`, `place_down(object, location)`, `activate_toaster()`, `pour_coffee()`).
3.  Instruct the LLM on the desired output format (e.g., a JSON list of actions).
4.  Include considerations for error handling or asking clarifying questions if the plan becomes ambiguous.
Discuss why each component of your prompt is important for effective planning."

---

## Prompt 2: Handling LLM Plan Failures in Robotics

**Goal**: Address the challenges of robustness and error recovery in LLM-driven planning.

**AI Prompt**:
"An LLM-generated plan for a humanoid robot encounters a failure during execution (e.g., the `pick_up` action fails because the object is too heavy). Explain how the robotic system could detect this failure and how the LLM could be leveraged to recover or replan. Discuss the information that would need to be fed back to the LLM (e.g., current state, type of failure) and how the LLM could then propose a new plan or a corrective action. Provide a conceptual example of this feedback loop."

---

## Prompt 3: LLMs for Commonsense Reasoning in Robotics

**Goal**: Explore the application of LLMs for commonsense knowledge.

**AI Prompt**:
"Elaborate on how Large Language Models (LLMs) can provide 'commonsense reasoning' to a humanoid robot's cognitive planning. Provide specific examples of everyday scenarios where a robot might need commonsense knowledge (e.g., what typically goes inside a refrigerator, how to open different types of doors, the likely location of common objects). Explain how an LLM, through its pre-training, could provide this knowledge to the robot's planning system, filling gaps where explicit programming would be impractical."
```