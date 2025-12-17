"""
Test file for textbook personalization skill
"""

import json
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from personalization_skill import personalize_textbook_chapter


def test_personalization():
    # Test case 1: Beginner with simulation hardware
    beginner_profile = {
        "level": "beginner",
        "hardware": "simulation",
        "background": "computer_science",
        "learning_style": "practical",
        "goals": ["learn basics of robotics"],
        "constraints": {"time": "6 months"}
    }

    chapter_content = """
# Introduction to ROS 2

ROS 2 is a flexible framework for writing robot software. It's a collection of libraries and tools that help you build robot applications.

## Nodes

Nodes are the fundamental building blocks of ROS 2 applications. Each node runs independently and communicates with other nodes through messages.

### Creating a Node

To create a node in Python:

```python
import rclpy
from rclpy.node import Node
```

The physical robot would use nodes to control its motors.
"""

    result = personalize_textbook_chapter(chapter_content, json.dumps(beginner_profile))
    print("=== Beginner, Simulation, CS Background ===")
    print(result)
    print("\n" + "="*50 + "\n")

    # Test case 2: Advanced with advanced hardware
    advanced_profile = {
        "level": "advanced",
        "hardware": "advanced_hardware",
        "background": "engineering",
        "learning_style": "theoretical",
        "goals": ["optimize robot performance"],
        "constraints": {"time": "2 months"}
    }

    result2 = personalize_textbook_chapter(chapter_content, json.dumps(advanced_profile))
    print("=== Advanced, Advanced Hardware, Engineering Background ===")
    print(result2)
    print("\n" + "="*50 + "\n")

    # Test case 3: Intermediate with mixed hardware, physics background
    mixed_profile = {
        "level": "intermediate",
        "hardware": "mixed",
        "background": "physics",
        "learning_style": "balanced",
        "goals": ["understand robot dynamics"],
        "constraints": {"time": "3 months"}
    }

    result3 = personalize_textbook_chapter(chapter_content, json.dumps(mixed_profile))
    print("=== Intermediate, Mixed Hardware, Physics Background ===")
    print(result3)


if __name__ == "__main__":
    test_personalization()