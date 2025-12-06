---
title: Nodes, Topics, and Services in ROS 2
---

# Nodes, Topics, and Services in ROS 2

In the world of the Robotic Operating System 2 (ROS 2), the ability of different software components to communicate seamlessly is fundamental to building complex and robust robotic systems. This communication paradigm is primarily facilitated through the concepts of **Nodes**, **Topics**, and **Services**. These elements form the backbone of ROS 2's distributed architecture, allowing for modularity, reusability, and scalability in robotic software development.

## ROS 2 Nodes: The Computational Units

At the heart of any ROS 2 system are **nodes**. A node is essentially an executable process that performs a specific computational task within the robot's ecosystem. Think of nodes as individual programs or processes, each designed to handle a particular function, such as:

*   **Sensor Data Acquisition**: A node might be dedicated to reading data from a LIDAR, camera, or IMU.
*   **Data Processing**: Another node could filter noise from sensor data, perform object recognition on camera feeds, or execute localization algorithms.
*   **Actuator Control**: A node might be responsible for sending commands to motors, controlling grippers, or managing a robot arm.
*   **Navigation**: A complex node could handle path planning, obstacle avoidance, and overall robot movement.

**Key Characteristics of Nodes:**

*   **Modularity**: Each node is an independent unit, making it easier to develop, test, and debug individual functionalities without affecting the entire system.
*   **Reusability**: Well-designed nodes can be reused across different robotic projects or even by different robots.
*   **Isolation**: If one node crashes, it ideally should not bring down the entire robotic system, enhancing system resilience.
*   **Distribution**: Nodes can run on the same machine, on different machines, or even across a network of robots, allowing for flexible system deployment.

## Topics: The Asynchronous Data Streams (Publish/Subscribe)

**Topics** are the primary mechanism for asynchronous, one-way data streaming in ROS 2. They implement a **publish/subscribe** communication model.

### How Topics Work:

1.  **Publisher**: A node that wants to share data creates a **publisher** for a specific topic. It continuously publishes messages of a defined type to that topic.
2.  **Subscriber**: One or more nodes that are interested in receiving that data create a **subscriber** for the same topic. Whenever a message is published to the topic, all subscribers receive a copy of that message.

**Analogy**: Imagine a radio station (publisher) broadcasting information on a specific frequency (topic). Multiple radios (subscribers) can tune into that frequency and receive the broadcast without the radio station knowing who is listening.

### Key Aspects of Topics:

*   **Loose Coupling**: Publishers and subscribers don't need direct knowledge of each other. They only need to agree on the topic name and the message type.
*   **Many-to-Many Communication**: A single topic can have multiple publishers and multiple subscribers.
*   **Message Types**: Every topic uses a specific message type (e.g., `std_msgs/String`, `sensor_msgs/LaserScan`, `geometry_msgs/Twist`). This ensures that the data being sent and received adheres to a consistent structure.
*   **Data Flow**: Topics are ideal for continuous streams of data, such as sensor readings, robot pose estimates, or motor commands.

### Example: Robot Odometry

*   **Publisher Node**: An `odometry_publisher` node (often part of a driver or localization system) reads data from wheel encoders and IMU.
*   **Message Type**: `nav_msgs/Odometry` (contains robot position, orientation, and velocity).
*   **Topic Name**: `/odom`
*   **Subscriber Nodes**:
    *   A `map_builder` node subscribes to `/odom` to update the robot's position on a map.
    *   A `robot_controller` node subscribes to `/odom` to adjust its movement based on current position.
    *   A `display_node` subscribes to `/odom` to visualize the robot's path in a GUI.

## Services: The Synchronous Request/Reply Interactions

While topics are for continuous, one-way data, **services** provide a mechanism for synchronous, two-way communication, following a **request/reply** model.

### How Services Work:

1.  **Service Server**: A node that offers a particular functionality creates a **service server**. It waits for requests from other nodes.
2.  **Service Client**: A node that needs that functionality creates a **service client**. It sends a request message to the service server and blocks (waits) until it receives a response message.

**Analogy**: Think of calling a function in a programming language. You call the function (send a request), and the function returns a value (sends a reply) after completing its operation.

### Key Aspects of Services:

*   **Synchronous**: The client waits for the server's response. This is suitable for operations where the client needs an immediate result before proceeding.
*   **One-to-One Communication**: Typically, one client sends a request to one server for a specific service.
*   **Service Types**: Like message types, services have defined **service types** (e.g., `std_srvs/Empty`, `rcl_interfaces/SetParameters`). A service type defines both the structure of the request message and the structure of the response message.
*   **Command-and-Control**: Services are perfect for commanding a robot to perform a discrete action, like "take a picture," "clear costmap," or "change a parameter."

### Example: Setting a Robot Parameter

*   **Service Server Node**: A `parameter_server` node manages the robot's configurable parameters (e.g., maximum speed, sensor gain). It provides a `SetParameters` service.
*   **Service Type**: `rcl_interfaces/SetParameters`
*   **Service Name**: `/robot_parameters/set_parameter`
*   **Service Client Node**: A `user_interface` node or a `mission_planner` node might call this service to dynamically change a robot parameter.
    *   **Request**: `parameter_name: "max_speed", value: 1.5`
    *   **Response**: `success: true, message: "max_speed set to 1.5"`

## Conclusion

Nodes, Topics, and Services are the fundamental building blocks for inter-process communication in ROS 2. By effectively utilizing these concepts, developers can create modular, distributed, and highly capable robotic applications. Topics facilitate continuous data streams in an asynchronous manner, while services enable synchronous request-reply interactions for specific commands or queries. Mastering these communication primitives is essential for anyone developing with ROS 2.