# Textbook Chapter Personalization Skill

## Overview
This skill personalizes textbook chapters based on user profiles. It takes chapter content in Markdown format and user profile information, then rewrites the content to match the user's level, adjusts examples for their hardware, and adds appropriate learning paths while maintaining technical terminology.

## Inputs
- **Chapter Content**: Markdown formatted textbook chapter
- **User Profile**: JSON object containing:
  - `level`: "beginner", "intermediate", or "advanced"
  - `hardware`: "simulation", "basic_hardware", "advanced_hardware", or "mixed"
  - `background`: "computer_science", "engineering", "mathematics", "physics", or "other"
  - `learning_style`: "theoretical", "practical", or "balanced"
  - `goals`: Array of learning objectives
  - `constraints`: Optional time or resource constraints

## Outputs
- Personalized chapter content in Markdown format
- Adjusted complexity level appropriate for user
- Hardware-appropriate examples and exercises
- Learning path suggestions
- Maintained technical terminology with appropriate explanations

## Process
1. Analyze user profile and chapter content
2. Adjust content complexity based on user level
3. Modify examples to match user's hardware capabilities
4. Adapt explanations based on user's background
5. Customize learning path suggestions
6. Maintain technical terminology with appropriate context

## Implementation Guidelines

### Complexity Adjustment
- **Beginner**: Add more explanations, analogies, and step-by-step breakdowns
- **Intermediate**: Moderate level of detail, fewer basic explanations
- **Advanced**: Concise explanations, focus on advanced concepts

### Hardware Adaptation
- **Simulation**: Focus on software tools, simulation examples
- **Basic Hardware**: Emphasize fundamental concepts, simple implementations
- **Advanced Hardware**: Include complex examples, performance considerations
- **Mixed**: Provide both simulation and hardware examples

### Background Adaptation
- **Computer Science**: Emphasize algorithms, data structures, software concepts
- **Engineering**: Focus on system design, practical applications, physical principles
- **Mathematics**: Include mathematical foundations, proofs, formal descriptions
- **Physics**: Emphasize physical principles, laws, and real-world applications

### Learning Style Adaptation
- **Theoretical**: Include more conceptual explanations, mathematical foundations
- **Practical**: Focus on hands-on examples, implementation details, exercises
- **Balanced**: Mix of both theoretical and practical content

## Examples

### Example User Profile
```json
{
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
```

### Example Personalization Process
Given a chapter about ROS 2 nodes:

1. **Complexity**: Intermediate level - provide implementation details without basic programming explanations
2. **Hardware**: Simulation-focused examples using Gazebo or similar
3. **Background**: Computer science perspective - emphasize architecture, communication patterns
4. **Learning Style**: Practical approach - include code examples, implementation steps
5. **Goals**: Focus on node architecture and control implementation aspects
6. **Constraints**: Time-appropriate exercises, accessible with provided resources

## Quality Assurance
- Maintain technical accuracy and terminology
- Ensure explanations are appropriate for user level
- Verify examples are compatible with user's hardware
- Preserve educational value while personalizing
- Keep content engaging and relevant to user's goals

## Error Handling
- If user profile is incomplete, use default values
- If hardware constraints are too restrictive, provide alternative approaches
- If content cannot be personalized, return original with explanation
- Validate that technical terminology remains accurate after personalization