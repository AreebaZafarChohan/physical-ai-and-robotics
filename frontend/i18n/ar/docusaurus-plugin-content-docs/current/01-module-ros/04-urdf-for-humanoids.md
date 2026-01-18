---
title: "URDF for Humanoids in ROS 2"
---

# URDF for Humanoids in ROS 2

When working with complex robots, especially humanoids, accurately describing their physical structure is paramount. This is where **URDF (Unified Robot Description Format)** comes into play. URDF is an XML-based file format used in ROS 2 (and ROS 1) to describe all aspects of a robot, including its kinematic structure, visual appearance, collision properties, and inertial characteristics. For humanoid robots, URDF allows for the meticulous definition of their numerous links and joints, which is crucial for simulation, motion planning, and control.

## What is URDF?

URDF provides a standardized way to model a robot as a set of rigid bodies (called **links**) connected by various types of **joints**. This hierarchical description allows ROS 2 packages to understand the robot's structure, enabling functionalities like:

*   **Kinematics**: Calculating forward and inverse kinematics for robotic arms and legs.
*   **Visualization**: Displaying the robot in simulation environments (e.g., Gazebo) or visualization tools (e.g., RViz).
*   **Collision Detection**: Defining simplified geometries for collision checks in simulations and real-time environments.
*   **Motion Planning**: Providing the robot model to motion planners (e.g., MoveIt 2) to generate valid trajectories.

## Why is URDF Crucial for Humanoids?

Humanoid robots are inherently complex due to their many degrees of freedom, intricate joint structures, and the need to interact with environments designed for humans. URDF's ability to precisely define:

*   **Many Joints**: Humanoids can have dozens of joints (neck, shoulders, elbows, wrists, hips, knees, ankles), each needing specific rotational or prismatic limits and dynamics.
*   **Link Geometries**: Accurately representing the shape and size of each body part (torso, head, limbs) for realistic visualization and collision.
*   **Inertial Properties**: Defining the mass and inertia of each link, essential for accurate physics simulation and dynamic control.

Without a detailed URDF model, accurately simulating, planning, and controlling a humanoid robot becomes incredibly challenging.

## Core Elements of a URDF File

A typical URDF file consists of two primary elements:

### 1. `<link>`

A `<link>` element defines a rigid body segment of the robot. It contains information about its:

*   **Visual Properties**: How the link looks (geometry, material, color). This is what you see in RViz or Gazebo.
*   **Collision Properties**: Simplified geometry used for collision detection. Often a simpler shape (box, sphere, cylinder) than the visual geometry for computational efficiency.
*   **Inertial Properties**: Mass, center of mass, and inertia tensor, which are vital for physics simulations.

**Example Link Definition**:

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.2 0.2 0.4"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.2 0.4"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### 2. `<joint>`

A `<joint>` element defines how two links are connected. It specifies the relationship between a `parent` link and a `child` link. Key attributes include:

*   **`type`**: Specifies the type of joint (e.g., `revolute`, `continuous`, `prismatic`, `fixed`). Humanoid joints are typically `revolute` (rotating around an axis) or `fixed` (for rigid connections).
*   **`origin`**: Defines the transformation (position and orientation) of the child link relative to the parent link.
*   **`axis`**: For revolute and prismatic joints, this specifies the axis of motion.
*   **`limit`**: Defines the physical limits of the joint (lower/upper bounds, velocity, effort).

**Example Joint Definition (connecting base_link to a head_link)**:

```xml
<joint name="neck_joint" type="revolute">
  <parent link="base_link"/>
  <child link="head_link"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```

## Integrating URDF into a ROS 2 System

Once you have a URDF file for your humanoid, you typically integrate it into a ROS 2 system using these nodes:

### 1. `robot_state_publisher`

This ROS 2 package reads the URDF file and the current joint states (typically published by `joint_state_publisher` or hardware drivers) and broadcasts the robot's full kinematic state (TF transforms) over the `/tf` topic. This allows visualization tools (RViz), motion planners, and other nodes to know where every part of the robot is in 3D space.

**Launch File Example (`display.launch.py`)**:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF file path
    urdf_file_name = 'my_humanoid.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        urdf_file_name
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_path}],
    )

    # Joint State Publisher node (if simulating joint states)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui', # or joint_state_publisher for non-GUI
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # RViz2 node
    rviz_config_dir = os.path.join(get_package_share_directory('my_robot_description'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
```

### 2. `joint_state_publisher` (or hardware drivers)

This node publishes the values of the joints defined in the URDF. For simulation, `joint_state_publisher_gui` allows you to manually control the joints with sliders. In a real robot, hardware drivers would publish these joint states based on sensor feedback from the robot's motors. `robot_state_publisher` then consumes these joint states to produce the full robot transform tree.

## XACRO: An Extension to URDF

For even more complex robots like humanoids, directly writing URDF can become cumbersome due to repetition and lack of programmatic features. **XACRO (XML Macros)** is an XML macro language that allows for more concise and readable robot descriptions. It enables:

*   **Macros**: Define reusable snippets of XML.
*   **Parameters**: Pass variables into macros to create flexible models.
*   **Mathematical Expressions**: Perform calculations within the file for origin and inertia values.
*   **Conditional Statements**: Include or exclude parts of the robot description based on conditions.

XACRO files are processed into standard URDF before being used by ROS 2 tools.

## Conclusion

URDF is an indispensable tool for describing humanoid robots in ROS 2. By precisely defining links and joints, it provides the necessary foundation for accurate visualization, simulation, motion planning, and control. Leveraging tools like `robot_state_publisher` and extending URDF with XACRO allows developers to manage the inherent complexity of humanoid robot models effectively.