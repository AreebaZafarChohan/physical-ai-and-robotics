---
title: 'Try with AI: "Nodes, Topics, and Services in ROS 2"'
---

# Try with AI: Nodes, Topics, and Services in ROS 2

This section provides curated prompts for you to further explore the concepts of ROS 2 nodes, topics, and services with an AI assistant. Use these prompts with your favorite Large Language Model (LLM) to deepen your understanding or tackle system design challenges.

---

## Prompt 1: Designing a ROS 2 System for a Mobile Robot

**Goal**: Apply knowledge of nodes, topics, and services to design a practical robot system.

**AI Prompt**:
"Design a conceptual ROS 2 system for an autonomous mobile robot tasked with navigating a warehouse to pick up and deliver packages. Identify the key nodes required (e.g., for localization, mapping, path planning, motor control, object detection, package gripping). For each identified node, specify at least one ROS 2 topic it would publish to or subscribe from, and at least one ROS 2 service it might provide or call. Clearly describe the message types you would expect for these topics and the request/response types for services. Explain how these components interact to achieve the overall mission."

---

## Prompt 2: Deep Dive into Message Types

**Goal**: Understand the design and usage of different ROS 2 message types.

**AI Prompt**:
"Explain the importance of message types in ROS 2 communication. Choose three distinct standard ROS 2 message types (e.g., `geometry_msgs/Twist`, `sensor_msgs/LaserScan`, `nav_msgs/Odometry`) and describe their typical usage scenarios, their key fields, and how their structure is optimized for the data they carry. Discuss how defining custom message types can be beneficial for specific robotic applications and provide a simple example of a custom message definition in a `.msg` file."

---

## Prompt 3: When to Use Services vs. Topics vs. Actions

**Goal**: Differentiate communication patterns for optimal system design.

**AI Prompt**:
"Discuss the primary considerations for choosing between ROS 2 Topics, Services, and Actions for inter-node communication. Provide three distinct scenarios in a humanoid robot application where each communication pattern (one for Topic, one for Service, one for Action) would be the most appropriate choice. Justify your reasoning for each selection, focusing on factors like real-time requirements, data flow direction, and feedback needs."
```