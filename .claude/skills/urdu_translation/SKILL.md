# Urdu Translation Skill

## Overview
This skill translates English technical content into Urdu or Roman Urdu for textbook chapters. It preserves technical terms (ROS 2, Node, Topic, Isaac Sim, Gazebo, VLA, LLM, etc.), maintains formatting (headings, code blocks), and provides clear explanations of technical concepts in Urdu while avoiding literal word-by-word translation.

## Inputs
- **Chapter Content**: English Markdown formatted textbook chapter
- **Translation Style**: Either "formal_urdu" or "roman_urdu"

## Outputs
- Translated chapter content in Markdown format
- Preserved technical terminology with appropriate explanations
- Maintained formatting (headings, code blocks, lists)
- Clear explanations of technical concepts in Urdu
- Appropriate translation style based on user preference

## Process
1. Parse the English Markdown content
2. Identify and preserve technical terms that should not be translated
3. Translate non-technical content while explaining technical concepts
4. Maintain all formatting elements (headings, code blocks, lists)
5. Apply appropriate translation style (formal Urdu or Roman Urdu)
6. Ensure technical accuracy is preserved

## Technical Terms Preservation
The skill maintains the following technical terms in English:
- ROS-related: ROS, ROS 2, Node, Topic, Service, Action, rclpy, rclcpp, etc.
- Programming: Python, C++, API, Git, Docker, JSON, etc.
- Robotics: Isaac Sim, Gazebo, VLA, LLM, PID, SLAM, etc.
- Other: Linux, Windows, AWS, GPU, CPU, etc.

## Translation Guidelines

### Technical Explanations
- Explain technical concepts clearly in Urdu
- Provide context for technical terms
- Use appropriate equivalents where established in Urdu technical literature

### Formatting Preservation
- Preserve all Markdown formatting (headings, lists, bold, italics)
- Keep code blocks and inline code unchanged
- Maintain table structures and other Markdown elements

### Translation Styles
- **Formal Urdu**: Proper Urdu script with formal language structure
- **Roman Urdu**: Urdu written in Roman script with transliteration

### Avoid Literal Translation
- Focus on meaning rather than word-for-word translation
- Adapt content for cultural and linguistic context
- Maintain educational value and clarity

## Examples

### Example Input
```markdown
# Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. Nodes are the fundamental building blocks that communicate through topics, services, and actions.
```

### Example Process
1. **Technical Terms**: Keep "ROS 2", "Node", "Topic", "Service", "Action" in English
2. **Translation**: Convert other content to Urdu with appropriate explanations
3. **Formatting**: Preserve the heading structure
4. **Style**: Apply requested translation style (formal or Roman Urdu)

## Quality Assurance
- Maintain technical accuracy and terminology
- Ensure explanations are clear and appropriate for textbook context
- Verify formatting is preserved correctly
- Preserve educational value while translating
- Keep content engaging and accessible to Urdu speakers

## Error Handling
- If content cannot be translated properly, return with explanation
- Validate that technical terminology remains accurate after translation
- Handle malformed Markdown gracefully
- Provide feedback on any terms that couldn't be processed