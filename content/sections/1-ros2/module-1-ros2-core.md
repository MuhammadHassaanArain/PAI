# Chapter 1.1: ROS 2 Core Concepts: The Robotic Nervous System

**Chapter ID**: 1.1
**Module**: ROS 2 – Robotic Nervous System
**Authors**: Gemini Agent
**Last Updated**: December 4, 2025

## 🎯 Learning Objectives

By the end of this chapter, you will be able to:
*   Explain the fundamental architecture and design principles of ROS 2.
*   Identify and define key ROS 2 concepts: Nodes, Topics, Services, and Actions.
*   Understand how these concepts enable distributed communication in a robotic system.
*   Write basic ROS 2 Python code to create publishers, subscribers, service clients, and service servers.

## ✨ Core Concepts

### Introduction to ROS 2

The Robot Operating System (ROS) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors across a wide variety of robotic platforms. ROS 2 is the latest iteration, re-engineered to be more robust for production environments, support multiple robot types, and handle real-time requirements. Think of ROS 2 as the nervous system of a robot, allowing different "organs" (components) to communicate and coordinate.

### Nodes: The Brain Cells of Your Robot

In ROS 2, a **Node** is an executable process that performs computations. It's the smallest unit of computation in a ROS graph. Each node should be responsible for a single, modular purpose (e.g., a node to control motors, a node to read sensor data, a node to perform navigation). This modularity is a core principle, promoting code reusability and easier debugging.

#### Key Terms
*   **Node**: An executable process within ROS 2 that performs a specific task.
*   **ROS Graph**: A network of ROS 2 nodes communicating with each other.

### Topics: Broadcasting Information

**Topics** are the most common way for nodes to exchange messages. They implement a publish-subscribe communication model. A node that sends messages to a topic is called a **publisher**, and a node that receives messages from a topic is called a **subscriber**. This is ideal for streaming data, like sensor readings (e.g., camera images, LiDAR scans) or motor commands that are continuously updated.

#### Key Terms
*   **Topic**: A named channel for nodes to exchange messages using a publish-subscribe model.
*   **Publisher**: A node that sends messages to a topic.
*   **Subscriber**: A node that receives messages from a topic.
*   **Message**: A data structure containing information transmitted over a topic.

### Services: Request-Response Communication

While topics are great for continuous data streams, sometimes a node needs to make a specific request and receive a single, immediate response. This is where **Services** come in. Services implement a request-response communication model. A node that offers a service is called a **service server**, and a node that invokes a service is called a **service client**. This is useful for tasks like triggering a robot arm to pick up an object, querying a database, or performing a specific calculation.

#### Key Terms
*   **Service**: A named operation that takes a request and returns a response.
*   **Service Server**: A node that provides a service.
*   **Service Client**: A node that calls a service.

### Actions: Long-Running Goal-Based Tasks

**Actions** are designed for long-running tasks that involve sending a goal, getting continuous feedback on its progress, and ultimately receiving a result. They are built on top of topics and services but provide a more structured way to manage complex, asynchronous operations. Examples include navigating to a distant goal, performing a complex manipulation sequence, or charging a robot's battery.

#### Key Terms
*   **Action**: A long-running, goal-based communication mechanism that provides feedback and a final result.
*   **Goal**: The desired state or outcome of an action.
*   **Feedback**: Continuous updates on the progress of an action.
*   **Result**: The final outcome of an action.

## 💻 Code Examples

All code examples are designed to be reproducible on a fresh Ubuntu 22.04 LTS system with ROS 2 Humble installed. These examples assume you have ROS 2 Humble installed and sourced.

### Example: Simple ROS 2 Publisher (Python)

This example demonstrates how to create a basic ROS 2 node that publishes a string message to a topic every half-second.

**File**: `my_robot_pkg/my_robot_pkg/simple_publisher.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher') # Node name
        self.publisher_ = self.create_publisher(String, 'chatter', 10) # Topic name: chatter, QoS: 10
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # Keep node alive
    simple_publisher.destroy_node()
    rclpy.shutdown() # Shutdown ROS 2

if __name__ == '__main__':
    main()
```

**File**: `my_robot_pkg/setup.py` (for the ROS 2 package)

```python
from setuptools import find_packages, setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_robot_launch.py']), # Example launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A simple ROS 2 package for basic examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_pkg.simple_publisher:main',
        ],
    },
)
```

*   **Expected Output**:
    After building the package (`colcon build`) and sourcing the workspace (`source install/setup.bash`), running `ros2 run my_robot_pkg simple_publisher` should produce output similar to:
    ```
    [INFO] [simple_publisher]: Publishing: "Hello ROS 2! Count: 0"
    [INFO] [simple_publisher]: Publishing: "Hello ROS 2! Count: 1"
    [INFO] [simple_publisher]: Publishing: "Hello ROS 2! Count: 2"
    ...
    ```

### Example: Simple ROS 2 Subscriber (Python)

This example demonstrates how to create a basic ROS 2 node that subscribes to the `chatter` topic and prints received messages.

**File**: `my_robot_pkg/my_robot_pkg/simple_subscriber.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber') # Node name
        self.subscription = self.create_subscription(
            String,
            'chatter', # Topic name
            self.listener_callback,
            10) # QoS: 10
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Update `my_robot_pkg/setup.py`**: Add `simple_subscriber` to `entry_points`:

```python
        'console_scripts': [
            'simple_publisher = my_robot_pkg.simple_publisher:main',
            'simple_subscriber = my_robot_pkg.simple_subscriber:main', # Add this line
        ],
```

*   **Expected Output**:
    After building the package and sourcing the workspace, run `ros2 run my_robot_pkg simple_publisher` in one terminal and `ros2 run my_robot_pkg simple_subscriber` in another. The subscriber terminal should show:
    ```
    [INFO] [simple_subscriber]: I heard: "Hello ROS 2! Count: 0"
    [INFO] [simple_subscriber]: I heard: "Hello ROS 2! Count: 1"
    [INFO] [simple_subscriber]: I heard: "Hello ROS 2! Count: 2"
    ...
    ```

## 🧪 Step-by-Step Lab: [Lab Title]

This chapter provides foundational concepts and simple code examples. A dedicated lab to build a complete publisher/subscriber system will be covered in a subsequent lab chapter.

## ⚠️ Safety Notes

*   While these examples are purely software-based, understanding the flow of control and data is crucial before moving to physical systems. Incorrect message handling in real robots can lead to unexpected and potentially unsafe behaviors.
*   Always test changes in simulation first.

## 📚 Summary

*   ROS 2 provides a modular framework for robot software development, using a graph of communicating nodes.
*   **Nodes** are the fundamental computational units.
*   **Topics** facilitate one-to-many, asynchronous data streaming (publish-subscribe).
*   **Services** enable one-to-one, synchronous request-response interactions.
*   **Actions** are for long-running, goal-based tasks with continuous feedback.

## 📝 Assessment / Mini Project

**Challenge**: Design a simple ROS 2 system for a simulated mobile robot that has a sensor reporting its battery level and a motor controller that can receive speed commands. Describe which ROS 2 communication patterns (topics, services, actions) you would use for each interaction and why.

## 🤖 Agent Interaction Examples

The embedded RAG chatbot can assist with your learning. Here are some examples of how to interact with it for this chapter:

*   **Query**: "What is the main difference between ROS 2 topics and services?"
*   **Query**: "Can you provide a simple analogy for a ROS 2 node?"
*   **Query**: "I'm having trouble with `colcon build`. What are common issues?"
*   **Query**: "Summarize the key communication patterns in ROS 2."
