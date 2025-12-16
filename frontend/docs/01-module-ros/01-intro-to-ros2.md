---
title: "Introduction to ROS 2"
---

# Introduction to ROS 2

The Robotic Operating System (ROS) has been the de facto standard for robotic software development for over a decade. ROS 2 is the latest iteration, re-engineered to address the limitations of ROS 1 and to support the evolving demands of modern robotics, including real-time performance, multi-robot systems, and embedded platforms.

## What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. It provides a structured communication layer, a robust build system, and a rich ecosystem of development tools.

Unlike a traditional operating system, ROS 2 is a *meta-operating system* that runs on top of a conventional OS (like Linux, Windows, or macOS). It facilitates communication between various components (nodes) of a robotic system, allowing for modular development and reusability of code.

## Key Concepts in ROS 2

Understanding these core concepts is crucial for working with ROS 2:

### 1. Nodes

A **node** is an executable process that performs computation. In a typical ROS 2 system, multiple nodes collaborate to achieve a task. For example, one node might be responsible for reading sensor data, another for processing that data, and a third for controlling robot actuators. Each node is an independent program that can be run on its own or with other nodes.

### 2. Topics

**Topics** are named buses over which nodes exchange messages. A node can *publish* messages to a topic, and other nodes can *subscribe* to that topic to receive the messages. This publish/subscribe mechanism enables loose coupling between nodes, meaning nodes don't need to know about each other directly, only about the topics they communicate through. Messages are strongly typed, ensuring data consistency.

### 3. Services

**Services** are a request/reply mechanism for synchronous communication between nodes. When a node needs to perform a specific action and receive a response, it can *call* a service provided by another node. The service client sends a request message, and the service server processes it and sends back a response message. Services are suitable for operations that need a clear beginning and end with a definitive result.

### 4. Actions

**Actions** are a more complex form of communication used for long-running tasks. They provide a high-level goal, feedback on the progress of the goal, and a final result. Actions are built on top of topics and services and are particularly useful for tasks like navigating a robot to a target location, where intermediate feedback (e.g., "robot is moving towards target") is important.

## Advantages of ROS 2 over ROS 1

ROS 2 was designed from the ground up to address several limitations of ROS 1:

-   **Real-time Capabilities**: ROS 2 incorporates a Data Distribution Service (DDS) for its communication layer, enabling deterministic, low-latency communication crucial for real-time applications.
-   **Multi-robot Support**: Designed with multi-robot systems in mind, ROS 2 easily handles communication and coordination between multiple robots.
-   **Security**: DDS provides built-in security features, including authentication, encryption, and access control, which were largely absent in ROS 1.
-   **Embedded Systems**: ROS 2 is more suitable for embedded and resource-constrained systems due to its more efficient architecture and smaller footprint.
-   **Windows & macOS Support**: While ROS 1 was primarily Linux-based, ROS 2 has full support for Windows and macOS, broadening its development base.

## Simple ROS 2 Publisher/Subscriber Example

Let's illustrate a basic ROS 2 system with two nodes: a "talker" that publishes messages and a "listener" that subscribes to them.

First, ensure you have a ROS 2 environment set up (e.g., Foxy, Galactic, Humble, Iron, Jazzy).

### 1. Create a ROS 2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_ros2_pkg
```

### 2. Talker Node (`talker.py`)

Create `my_ros2_pkg/my_ros2_pkg/talker.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Listener Node (`listener.py`)

Create `my_ros2_pkg/my_ros2_pkg/listener.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Update `setup.py`

In `my_ros2_pkg/setup.py`, add the entry points for the nodes:

```python
from setuptools import find_packages, setup

package_name = 'my_ros2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A minimal ROS 2 package with talker and listener',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_ros2_pkg.talker:main',
            'listener = my_ros2_pkg.listener:main',
        ],
    },
)
```

### 5. Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select my_ros2_pkg
source install/setup.bash

# In one terminal
ros2 run my_ros2_pkg talker

# In another terminal
ros2 run my_ros2_pkg listener
```

You should see the listener node printing the messages published by the talker node. This simple example demonstrates the fundamental publish/subscribe communication pattern in ROS 2.