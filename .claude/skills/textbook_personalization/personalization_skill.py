"""
Textbook Chapter Personalization Skill
This module provides functionality to personalize textbook chapters based on user profiles.
"""

import json
import re
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from enum import Enum


class UserLevel(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class HardwareType(Enum):
    SIMULATION = "simulation"
    BASIC_HARDWARE = "basic_hardware"
    ADVANCED_HARDWARE = "advanced_hardware"
    MIXED = "mixed"


class BackgroundType(Enum):
    COMPUTER_SCIENCE = "computer_science"
    ENGINEERING = "engineering"
    MATHEMATICS = "mathematics"
    PHYSICS = "physics"
    OTHER = "other"


class LearningStyle(Enum):
    THEORETICAL = "theoretical"
    PRACTICAL = "practical"
    BALANCED = "balanced"


@dataclass
class UserProfile:
    level: UserLevel
    hardware: HardwareType
    background: BackgroundType
    learning_style: LearningStyle
    goals: List[str]
    constraints: Optional[Dict[str, Any]] = None


class TextbookPersonalizer:
    """
    A class to personalize textbook chapters based on user profiles.
    """

    def __init__(self):
        self.level_adjustments = {
            UserLevel.BEGINNER: {
                "explanation_depth": "detailed",
                "terminology": "explained",
                "examples": "step_by_step",
                "analogies": "included"
            },
            UserLevel.INTERMEDIATE: {
                "explanation_depth": "moderate",
                "terminology": "standard",
                "examples": "implementation_focused",
                "analogies": "selective"
            },
            UserLevel.ADVANCED: {
                "explanation_depth": "concise",
                "terminology": "technical",
                "examples": "advanced",
                "analogies": "minimal"
            }
        }

        self.background_contexts = {
            BackgroundType.COMPUTER_SCIENCE: {
                "focus": "algorithms, data structures, software architecture",
                "examples": "code implementations, system design",
                "terminology": "software engineering, computational"
            },
            BackgroundType.ENGINEERING: {
                "focus": "system design, practical applications, physical principles",
                "examples": "system integration, control systems, physical models",
                "terminology": "system dynamics, control theory, practical"
            },
            BackgroundType.MATHEMATICS: {
                "focus": "mathematical foundations, proofs, formal descriptions",
                "examples": "mathematical models, proofs, formal analysis",
                "terminology": "theoretical, formal, mathematical"
            },
            BackgroundType.PHYSICS: {
                "focus": "physical principles, laws, real-world applications",
                "examples": "physical models, laws of physics, real-world phenomena",
                "terminology": "physics-based, physical, real-world"
            },
            BackgroundType.OTHER: {
                "focus": "general understanding, broad concepts",
                "examples": "accessible examples, common applications",
                "terminology": "general, accessible"
            }
        }

    def parse_user_profile(self, profile_json: str) -> UserProfile:
        """
        Parse user profile from JSON string.
        """
        try:
            profile_data = json.loads(profile_json)

            return UserProfile(
                level=UserLevel(profile_data.get("level", "intermediate")),
                hardware=HardwareType(profile_data.get("hardware", "simulation")),
                background=BackgroundType(profile_data.get("background", "other")),
                learning_style=LearningStyle(profile_data.get("learning_style", "balanced")),
                goals=profile_data.get("goals", []),
                constraints=profile_data.get("constraints", {})
            )
        except Exception as e:
            raise ValueError(f"Invalid user profile JSON: {str(e)}")

    def personalize_content(self, markdown_content: str, user_profile: UserProfile) -> str:
        """
        Personalize the textbook chapter based on the user profile.
        """
        # Adjust complexity based on user level
        content = self._adjust_complexity(markdown_content, user_profile)

        # Adapt examples based on hardware
        content = self._adapt_examples_for_hardware(content, user_profile)

        # Adapt explanations based on background
        content = self._adapt_explanations_for_background(content, user_profile)

        # Adapt content based on learning style
        content = self._adapt_for_learning_style(content, user_profile)

        # Add learning path suggestions
        content = self._add_learning_path_suggestions(content, user_profile)

        return content

    def _adjust_complexity(self, content: str, user_profile: UserProfile) -> str:
        """
        Adjust content complexity based on user level.
        """
        adjustment = self.level_adjustments[user_profile.level]

        if user_profile.level == UserLevel.BEGINNER:
            # Add more explanations and analogies
            content = self._add_beginner_explanations(content)
        elif user_profile.level == UserLevel.INTERMEDIATE:
            # Moderate explanations
            content = self._add_intermediate_explanations(content)
        elif user_profile.level == UserLevel.ADVANCED:
            # Concise explanations
            content = self._add_advanced_explanations(content)

        return content

    def _add_beginner_explanations(self, content: str) -> str:
        """
        Add beginner-friendly explanations to content.
        """
        # Add analogies and step-by-step explanations
        content = re.sub(
            r'(\*\*.+?\*\*|`.+?`)',
            r'**\1** (Explanation: This is an important concept that will be detailed below)',
            content
        )

        # Add more detailed explanations for technical terms
        content = re.sub(
            r'(?<!\*\*)\b([A-Z]{2,}|[A-Z][a-z]+(?:[A-Z][a-z]*)+)\b(?!.*\*\*)',
            r'**\1** (Technical term explained in detail)',
            content
        )

        return content

    def _add_intermediate_explanations(self, content: str) -> str:
        """
        Add intermediate-level explanations to content.
        """
        # Add moderate explanations
        content = re.sub(
            r'(\*\*.+?\*\*)',
            r'\1 (Key concept)',
            content
        )

        return content

    def _add_advanced_explanations(self, content: str) -> str:
        """
        Add advanced-level explanations to content.
        """
        # Keep explanations concise
        # Remove unnecessary explanations
        return content

    def _adapt_examples_for_hardware(self, content: str, user_profile: UserProfile) -> str:
        """
        Adapt examples based on user's hardware capabilities.
        """
        if user_profile.hardware == HardwareType.SIMULATION:
            # Replace hardware-specific examples with simulation examples
            content = re.sub(
                r'physical robot',
                'simulated robot',
                content,
                flags=re.IGNORECASE
            )
            content = re.sub(
                r'real hardware',
                'simulation environment',
                content,
                flags=re.IGNORECASE
            )
        elif user_profile.hardware == HardwareType.BASIC_HARDWARE:
            # Focus on fundamental concepts, simple implementations
            content = re.sub(
                r'advanced implementation',
                'basic implementation',
                content,
                flags=re.IGNORECASE
            )
        elif user_profile.hardware == HardwareType.ADVANCED_HARDWARE:
            # Include complex examples and performance considerations
            content = re.sub(
                r'basic approach',
                'advanced approach with performance considerations',
                content,
                flags=re.IGNORECASE
            )
        elif user_profile.hardware == HardwareType.MIXED:
            # Provide both simulation and hardware examples
            content = re.sub(
                r'implementation',
                'implementation (both simulation and hardware approaches available)',
                content,
                flags=re.IGNORECASE
            )

        return content

    def _adapt_explanations_for_background(self, content: str, user_profile: UserProfile) -> str:
        """
        Adapt explanations based on user's background.
        """
        context = self.background_contexts[user_profile.background]

        # Add context-appropriate explanations
        if user_profile.background == BackgroundType.COMPUTER_SCIENCE:
            content = re.sub(
                r'concept',
                'concept (from a software engineering perspective)',
                content,
                flags=re.IGNORECASE
            )
        elif user_profile.background == BackgroundType.ENGINEERING:
            content = re.sub(
                r'concept',
                'concept (from a system design perspective)',
                content,
                flags=re.IGNORECASE
            )
        elif user_profile.background == BackgroundType.MATHEMATICS:
            content = re.sub(
                r'concept',
                'concept (with mathematical foundation)',
                content,
                flags=re.IGNORECASE
            )
        elif user_profile.background == BackgroundType.PHYSICS:
            content = re.sub(
                r'concept',
                'concept (with physical principles)',
                content,
                flags=re.IGNORECASE
            )

        return content

    def _adapt_for_learning_style(self, content: str, user_profile: UserProfile) -> str:
        """
        Adapt content based on user's learning style.
        """
        if user_profile.learning_style == LearningStyle.THEORETICAL:
            # Add more conceptual explanations and mathematical foundations
            content += "\n\n> **Theoretical Foundation**: This concept has deep theoretical roots..."
        elif user_profile.learning_style == LearningStyle.PRACTICAL:
            # Focus on hands-on examples and implementation details
            content += "\n\n> **Implementation Tip**: Here's how you can implement this..."
        elif user_profile.learning_style == LearningStyle.BALANCED:
            # Maintain balance between theory and practice
            pass

        return content

    def _add_learning_path_suggestions(self, content: str, user_profile: UserProfile) -> str:
        """
        Add learning path suggestions based on user's goals and constraints.
        """
        suggestions = []

        if user_profile.goals:
            goals_str = ", ".join(user_profile.goals)
            suggestions.append(f"- **Goal-Oriented Path**: Focus on concepts related to: {goals_str}")

        if user_profile.constraints:
            if "time" in user_profile.constraints:
                time_constraint = user_profile.constraints["time"]
                suggestions.append(f"- **Time-Appropriate Path**: This content is designed for {time_constraint} study period")

        if suggestions:
            suggestions_text = "\n".join(suggestions)
            content += f"\n\n## Learning Path Suggestions\n{suggestions_text}\n"

        return content


def personalize_textbook_chapter(markdown_content: str, user_profile_json: str) -> str:
    """
    Main function to personalize a textbook chapter.

    Args:
        markdown_content: The textbook chapter content in Markdown format
        user_profile_json: JSON string containing user profile information

    Returns:
        Personalized chapter content in Markdown format
    """
    personalizer = TextbookPersonalizer()
    user_profile = personalizer.parse_user_profile(user_profile_json)
    return personalizer.personalize_content(markdown_content, user_profile)


# Example usage
if __name__ == "__main__":
    # Example user profile
    example_profile = {
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

    # Example chapter content
    example_content = """
# ROS 2 Nodes and Communication

ROS 2 uses a distributed computing model where nodes communicate through topics, services, and actions.

## Nodes

Nodes are the fundamental building blocks of ROS 2 applications. Each node runs independently and communicates with other nodes through messages.

### Creating a Node

To create a node in Python, you would use the rclpy library:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
```

## Topics

Topics allow nodes to publish and subscribe to messages. This enables data flow between nodes in a decoupled manner.

The physical robot would use topics to send sensor data to processing nodes.
"""

    personalized = personalize_textbook_chapter(example_content, json.dumps(example_profile))
    print(personized)