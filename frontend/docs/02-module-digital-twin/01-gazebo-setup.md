---
title: Gazebo Setup for Digital Twin
---

# Gazebo Setup for Digital Twin

In the development of complex robotic systems, especially humanoids, a crucial step involves leveraging **digital twins**. A digital twin is a virtual representation of a physical object or system, allowing for testing, analysis, and optimization in a simulated environment before deployment in the real world. For robotics, **Gazebo** stands out as a powerful and widely used 3D physics simulator that provides a robust platform for creating and interacting with these digital twins.

## What is Gazebo?

Gazebo is an open-source 3D robotics simulator that allows you to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. It offers:

*   **Powerful Physics Engine**: Gazebo uses physics engines like ODE, Bullet, Simbody, and DART to accurately simulate dynamics, gravity, friction, and collisions.
*   **High-Quality Graphics**: Renders realistic environments and robot models.
*   **Sensor Simulation**: Simulates a wide range of sensors (cameras, LIDAR, IMU, sonar, force-torque sensors) with realistic noise and data output.
*   **Plugins**: A flexible plugin architecture allows users to extend its functionality, integrate with external software (like ROS 2), and customize robot behaviors.
*   **Command-Line and GUI Interface**: Can be run headlessly for automated testing or with a graphical interface for interactive development.

Gazebo's ability to create a high-fidelity virtual replica of a robot and its environment makes it an indispensable tool for developing and testing complex AI and control algorithms for humanoids.

## Setting Up Gazebo with ROS 2

Gazebo integrates seamlessly with ROS 2, leveraging ROS 2's communication infrastructure to control robots and read sensor data within the simulation.

### 1. Installation

If you've installed a full ROS 2 distribution, Gazebo (usually Gazebo Garden or Harmonic, depending on your ROS 2 distro) is often included. If not, you can install it:

```bash
# For Ubuntu 22.04 and ROS 2 Jazzy (or similar distros)
sudo apt update
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros2-control
```
This typically installs Gazebo and the necessary ROS 2 bridge packages (`gazebo_ros_pkgs` and `gazebo_ros2_control`) which facilitate communication between Gazebo and ROS 2 nodes.

### 2. Creating a Simple World File

A Gazebo **world file** (typically `.world` extension) defines the environment, including static objects, light sources, and initial robot spawn positions. It's an XML file.

**Example: `my_empty_world.world`**

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_empty_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <!-- You can add more static models here -->
    <!-- <include>
      <uri>model://brick_box_3x1x3</uri>
      <pose>0 2 0.5 0 0 0</pose>
    </include> -->
  </world>
</sdf>
```
You can place this in a `worlds` directory within your ROS 2 package (e.g., `my_robot_simulation/worlds/`).

### 3. Spawning a Robot Model in Gazebo

To spawn a robot, you need its URDF (or SDF) model and a ROS 2 launch file to orchestrate the process.

Let's assume you have a URDF file for a simple robot (e.g., `my_simple_robot.urdf` in `my_robot_description/urdf/`).

**Launch File Example (`spawn_robot.launch.py`)**

This launch file will:
1.  Load the robot's URDF description.
2.  Start Gazebo with your custom world.
3.  Spawn your robot model into the Gazebo world.
4.  Launch `robot_state_publisher` and `joint_state_publisher_gui` (as discussed in the URDF chapter).

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your URDF file
    urdf_file_name = 'my_simple_robot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        urdf_file_name
    )

    # Path to your world file
    world_file_name = 'my_empty_world.world'
    world_path = os.path.join(
        get_package_share_directory('my_robot_simulation'),
        'worlds',
        world_file_name
    )

    # Declare the 'use_sim_time' argument for ROS 2 nodes
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items(),
    )

    # Robot State Publisher node (from URDF chapter)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_path, 'use_sim_time': use_sim_time}],
    )

    # Joint State Publisher node (if simulating joint states)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Spawn the robot into Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_simple_robot',
                                   '-x', '0.0', '-y', '0.0', '-z', '0.1'], # Initial pose
                        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
    ])
```

### Running the Simulation

1.  **Build your packages**: `colcon build --packages-select my_robot_description my_robot_simulation`
2.  **Source the setup files**: `source install/setup.bash`
3.  **Launch the simulation**: `ros2 launch my_robot_simulation spawn_robot.launch.py`

This will open the Gazebo GUI with your custom world and your robot spawned at the specified coordinates. You can then interact with the robot in Gazebo, send commands via ROS 2 topics, and visualize sensor data, effectively working with your robot's digital twin.

## Conclusion

Gazebo is a cornerstone for robotics simulation, enabling the creation and interaction with digital twins. Its powerful physics engine, sensor simulation capabilities, and tight integration with ROS 2 make it an essential tool for developing, testing, and refining Physical AI and humanoid robotics algorithms in a safe and cost-effective virtual environment.