---
title: "RCLPY Integration: Python Client Library for ROS 2"
---

# RCLPY Integration: Python Client Library for ROS 2

When developing robotic applications with ROS 2, Python is a popular choice due to its readability, vast ecosystem of libraries for AI and data science, and rapid prototyping capabilities. **RCLPY** is the official Python client library for ROS 2, providing a clean and intuitive interface to interact with the core ROS 2 concepts like nodes, topics, services, and actions. This chapter will delve into how to effectively use RCLPY to build robust ROS 2 applications in Python.

## What is RCLPY?

RCLPY is a Python binding for `rcl` (ROS Client Library), which is a C API that provides the implementation of the core ROS 2 concepts. By building upon `rcl`, RCLPY inherits the performance and efficiency of the underlying C implementation while offering the ease of use and expressiveness of Python. It allows Python developers to seamlessly create ROS 2 nodes and integrate them into a larger ROS 2 graph.

## Getting Started: Creating a ROS 2 Python Package

Before writing any Python code, it's good practice to create a dedicated ROS 2 package using `ament_python`.

```bash
# Assuming you are in your ROS 2 workspace's src directory (e.g., ~/ros2_ws/src)
ros2 pkg create --build-type ament_python my_python_pkg
```

This command creates a new directory `my_python_pkg` with the necessary `package.xml` and `setup.py` files. Your Python scripts will typically reside in a subdirectory named after your package (e.g., `my_python_pkg/my_python_pkg/`).

## Implementing ROS 2 Nodes with RCLPY

All ROS 2 applications in RCLPY start with a `Node`.

### 1. Basic Node Structure

A minimal ROS 2 node in Python looks like this:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node') # Initialize the node with a name
        self.get_logger().info("Minimal Node has been started!")

def main(args=None):
    rclpy.init(args=args)           # Initialize ROS 2 communication
    node = MinimalNode()            # Create an instance of your node
    rclpy.spin(node)                # Keep the node alive and processing events
    node.destroy_node()             # Clean up when the node is stopped
    rclpy.shutdown()                # Shut down ROS 2 communication

if __name__ == '__main__':
    main()
```

To make this executable, add an entry point in `setup.py` (inside the `entry_points` dictionary):

```python
    entry_points={
        'console_scripts': [
            'minimal_node = my_python_pkg.minimal_node:main',
        ],
    },
```

Then, build and run:

```bash
cd ~/ros2_ws
colcon build --packages-select my_python_pkg
source install/setup.bash
ros2 run my_python_pkg minimal_node
```

### 2. Creating a Publisher

To publish messages, a node needs to create a publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the message type

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10) # type, topic name, queue size
        self.timer = self.create_timer(0.5, self.timer_callback) # timer for publishing
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2 from Python: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Creating a Subscriber

To subscribe to messages, a node needs to create a subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,             # Message type
            'chatter',          # Topic name
            self.listener_callback, # Callback function
            10                  # Queue size
        )
        self.get_logger().info("Simple Subscriber is listening...")

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Implementing a Service Server

For request/reply communication, a node can act as a service server:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # A standard service type

class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info("AddTwoInts Service Server ready.")

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}, sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Note: You might need to install `example_interfaces` if not already present: `sudo apt install ros-<ROS_DISTRO>-example-interfaces`*

### 5. Implementing a Service Client

A node can call a service provided by another node:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f'Sending request: a={a}, b={b}')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print('Usage: ros2 run my_python_pkg add_two_ints_client <int> <int>')
        return
    
    client = AddTwoIntsClient()
    client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().error(f'Service call failed: {e}')
            else:
                client.get_logger().info(
                    f'Result of add_two_ints: for {client.req.a} + {client.req.b} = {response.sum}'
                )
            break
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Structuring Python ROS 2 Applications

*   **Modular Design**: Break down your application into small, single-responsibility nodes.
*   **Clear Entry Points**: Use `main()` functions and `entry_points` in `setup.py` for clear execution.
*   **Message and Service Types**: Always use the correct message and service types, and if necessary, define your own custom types for complex data structures.
*   **Logging**: Use `self.get_logger().info()`, `warn()`, `error()` for informative output, not `print()`.
*   **Error Handling**: Implement robust error handling, especially for service calls and external dependencies.
*   **Configuration**: Utilize ROS 2 parameters to make your nodes configurable without code changes.
*   **Package Structure**: Keep your Python scripts organized within your package directory.

By following these guidelines and leveraging the power of RCLPY, you can develop efficient, readable, and maintainable ROS 2 applications in Python.