# Textbook Personalization Skill

This skill personalizes textbook chapters based on user profiles to provide tailored learning experiences.

## Overview

The Textbook Personalization Skill takes chapter content in Markdown format and user profile information, then rewrites the content to match the user's level, adjusts examples for their hardware, and adds appropriate learning paths while maintaining technical terminology.

## Features

- Adjusts content complexity based on user level (beginner, intermediate, advanced)
- Adapts examples to match user's hardware capabilities (simulation, basic hardware, advanced hardware, mixed)
- Customizes explanations based on user's academic background (computer science, engineering, mathematics, physics, other)
- Modifies content based on learning style preferences (theoretical, practical, balanced)
- Adds personalized learning path suggestions based on user goals
- Maintains technical accuracy while personalizing content

## Usage

### Input Format

The skill requires two inputs:

1. **Markdown Content**: The textbook chapter in Markdown format
2. **User Profile**: A JSON object with the following structure:

```json
{
  "level": "beginner|intermediate|advanced",
  "hardware": "simulation|basic_hardware|advanced_hardware|mixed",
  "background": "computer_science|engineering|mathematics|physics|other",
  "learning_style": "theoretical|practical|balanced",
  "goals": ["goal1", "goal2", ...],
  "constraints": {
    "time": "timeframe",
    "resources": "available resources"
  }
}
```

### Example Usage

```python
from personalization_skill import personalize_textbook_chapter

# Define user profile
user_profile = {
  "level": "intermediate",
  "hardware": "simulation",
  "background": "computer_science",
  "learning_style": "practical",
  "goals": ["understand ROS 2 architecture", "implement basic robot control"],
  "constraints": {
    "time": "3 months",
    "resources": "personal computer, internet access"
  }
}

# Personalize chapter content
personalized_content = personalize_textbook_chapter(
  markdown_content,
  json.dumps(user_profile)
)
```

## Implementation Details

The skill implements several personalization strategies:

1. **Complexity Adjustment**: Modifies explanation depth and terminology based on user level
2. **Hardware Adaptation**: Changes examples to match available hardware
3. **Background Adaptation**: Adjusts explanations to align with user's academic background
4. **Learning Style Adaptation**: Emphasizes theory or practice based on preferences
5. **Learning Path Suggestions**: Provides customized study paths based on goals

## Files

- `SKILL.md`: Main skill definition and documentation
- `personalization_skill.py`: Python implementation
- `config.yaml`: Skill configuration
- `test_personalization.py`: Test cases