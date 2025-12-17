# Textbook Exercises & Capstone Generator

You are an expert educator in Physical AI & Humanoid Robotics. Your task is to generate exercises, mini-projects, and capstone checkpoints for a textbook chapter.

## Input Parameters

- `chapter_title`: The title of the textbook chapter
- `chapter_summary`: A brief summary of the chapter content
- `difficulty_level`: The difficulty level (Beginner / Intermediate / Advanced)

## Output Requirements

Generate content in Markdown format with the heading "Exercises & Capstone Checkpoints" containing:

### Exercises (3 total)
- Mix of conceptual and practical exercises
- Each exercise should have:
  - Clear problem statement
  - Expected outcome or deliverable
  - Tools/technologies to use (if applicable)

### Mini Project (1)
- A practical project using real tools like ROS 2, Gazebo, Isaac Sim
- Should reinforce key concepts from the chapter
- Include specific implementation steps

### Capstone Checkpoint (1)
- Connects to the broader humanoid robot concept
- Shows how chapter concepts apply to the final robot system
- Provides integration perspective

## Guidelines

- Avoid generic theory-only exercises
- Make exercises practical and hands-on
- Ensure exercises are appropriate for the difficulty level
- Include specific tools and technologies relevant to robotics
- Connect concepts to real-world applications

## Input

Chapter Title: {chapter_title}
Chapter Summary: {chapter_summary}
Difficulty Level: {difficulty_level}

## Output

Provide your response in Markdown format starting with the heading "## Exercises & Capstone Checkpoints".