---
title: Quiz: URDF for Humanoids
---

# Quiz: URDF for Humanoids in ROS 2

Test your understanding of URDF and its application to humanoid robots in ROS 2.

---

## Question 1

What is the primary purpose of URDF in ROS 2?

a) To define the robot's control algorithms.
b) To describe the robot's physical structure, visual properties, and kinematics.
c) To manage ROS 2 communication protocols.
d) To simulate robot environments and physics.

**Answer**: b) To describe the robot's physical structure, visual properties, and kinematics.

---

## Question 2

Which URDF element defines a rigid body segment of the robot and includes its visual, collision, and inertial properties?

a) `<joint>`
b) `<link>`
c) `<model>`
d) `<robot>`

**Answer**: b) `<link>`

---

## Question 3

For a humanoid robot, why is a detailed URDF model particularly crucial?

a) It simplifies the graphical rendering in basic visualization tools.
b) It allows for precise definition of its many joints and complex dynamics, vital for balance and motion.
c) It automatically generates C++ code for robot control.
d) It reduces the computational cost of sensor processing.

**Answer**: b) It allows for precise definition of its many joints and complex dynamics, vital for balance and motion.

---

## Question 4

What ROS 2 node typically reads the URDF file and joint states to broadcast the robot's full kinematic state (TF transforms)?

a) `joint_state_publisher`
b) `robot_state_publisher`
c) `tf_broadcaster`
d) `urdf_loader`

**Answer**: b) `robot_state_publisher`

---

## Question 5

Which XML-based extension to URDF allows for more concise and readable robot descriptions through macros, parameters, and mathematical expressions?

a) SDF
b) Collada
c) XACRO
d) YAML

**Answer**: c) XACRO
