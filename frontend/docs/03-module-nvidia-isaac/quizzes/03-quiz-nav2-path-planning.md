---
title: Quiz: NVIDIA Isaac ROS - Nav2 Path Planning
---

# Quiz: NVIDIA Isaac ROS: Nav2 Path Planning

Test your understanding of the ROS 2 Navigation Stack (Nav2) and its acceleration with NVIDIA Isaac ROS.

---

## Question 1

Which component of Nav2 is responsible for finding a high-level, obstacle-free path from the robot's current location to its goal within a known map?

a) Local Planner
b) Global Planner
c) Costmap Filter
d) Behavior Tree

**Answer**: b) Global Planner

---

## Question 2

How does NVIDIA Isaac ROS primarily contribute to accelerating Nav2's performance?

a) By providing new types of sensors for Nav2.
b) By offering GPU-accelerated primitives for perception (e.g., VSLAM) and potentially costmap generation.
c) By completely replacing the Nav2 stack with a proprietary solution.
d) By simplifying the ROS 2 message types used by Nav2.

**Answer**: b) By offering GPU-accelerated primitives for perception (e.g., VSLAM) and potentially costmap generation.

---

## Question 3

What is a "Costmap" in Nav2?

a) A financial budget for the robot's navigation.
b) A grid map representing the environment, including obstacles and preferred areas.
c) A log of all navigation errors encountered.
d) A visualization tool for robot paths.

**Answer**: b) A grid map representing the environment, including obstacles and preferred areas.

---

## Question 4

Why do humanoid robots present unique challenges for autonomous navigation compared to wheeled robots, particularly in relation to path planning and control?

a) They are generally slower.
b) They have simpler sensor arrays.
c) Their complex kinematics, dynamics, and balance requirements make motion planning much harder.
d) They cannot use maps.

**Answer**: c) Their complex kinematics, dynamics, and balance requirements make motion planning much harder.

---

## Question 5

Which Isaac ROS component, discussed in a previous chapter, can provide highly accurate and fast odometry and localization data, essential for feeding into Nav2's localization component?

a) Isaac Sim
b) Isaac SDK
c) Isaac ROS VSLAM
d) Isaac Gym

**Answer**: c) Isaac ROS VSLAM
