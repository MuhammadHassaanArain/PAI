# ROS 2 Core Concepts: Nodes, Topics, Services, and Actions

## Learning Objectives

- Understand the fundamental architecture of ROS 2.
- Differentiate between ROS 2 nodes, topics, services, and actions.
- Learn how these communication mechanisms enable modular robotics applications.

## Core Concepts

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's not a true operating system, but rather a set of software libraries, tools, and conventions that aim to simplify the task of creating complex and robust robot applications. ROS 2 builds upon the success of ROS 1, bringing significant improvements in real-time performance, security, and support for multi-robot systems.

The core of ROS 2's modularity lies in its communication patterns:

1.  **Nodes**: A Node is an executable process that performs computation. ROS 2 applications are typically composed of many nodes, each designed to perform a specific task (e.g., a node for reading sensor data, a node for controlling motors, a node for path planning). Nodes are isolated, meaning a crash in one node doesn't necessarily bring down the entire system.

2.  **Topics**: Topics are a fundamental communication mechanism in ROS 2 for publishing data asynchronously. A node can "publish" messages to a topic, and other nodes can "subscribe" to that topic to receive those messages. This is a one-to-many communication model, ideal for streaming data like sensor readings (e.g., camera images, lidar scans, odometry).

    -   **Publisher**: A node that sends messages on a topic.
    -   **Subscriber**: A node that receives messages from a topic.
    -   **Message Type**: Data exchanged over topics must conform to a predefined message type, ensuring data consistency.

3.  **Services**: Services provide a synchronous request/response communication model. A "service client" sends a request to a "service server," which processes the request and sends back a response. This is a one-to-one communication model, ideal for operations that require an immediate result, like querying a robot's current state or triggering a specific action (e.g., "get current robot position", "rotate gripper by 90 degrees").

    -   **Service Server**: A node that provides a service and responds to requests.
    -   **Service Client**: A node that makes requests to a service server.
    -   **Request/Response Types**: Services define specific request and response message types.

4.  **Actions**: Actions are an extension of services that provide long-running, preemptable tasks with feedback. An "action client" sends a goal to an "action server," which works towards achieving that goal, sending periodic feedback (e.g., progress updates) to the client. The client can also cancel the goal if needed. This is a one-to-one communication model suitable for complex tasks like "navigate to a specific point" or "pick up an object," where the task takes time and feedback is necessary.

    -   **Action Server**: A node that executes a long-running goal and provides feedback.
    -   **Action Client**: A node that sends goals to an action server and receives feedback.
    -   **Goal/Result/Feedback Types**: Actions define specific message types for goals, results, and feedback.

## Step-by-Step Lab: ROS 2 Basics - `ros2 run` and `ros2 topic`

This lab will guide you through exploring a running ROS 2 system using command-line tools. We'll use a pre-built Docker image with a basic ROS 2 setup.

### Code Examples

First, launch a ROS 2 Docker container:

```bash
# Pull the ROS 2 Humble Docker image
docker pull osrf/ros:humble-desktop-full

# Run the Docker container in interactive mode
docker run -it --rm osrf/ros:humble-desktop-full bash

# Inside the container, source the ROS 2 setup script
source /opt/ros/humble/setup.bash
```

Now, let's explore some basic ROS 2 concepts using the `ros2` command-line tools within the container:

**1. Running a Publisher Node:**
Open a new terminal on your host machine and run another instance of the Docker container, then execute a publisher:
```bash
# In a NEW host terminal:
docker run -it --rm osrf/ros:humble-desktop-full bash
# Inside this new container:
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
This `talker` node is publishing messages to a topic.

**2. Running a Subscriber Node:**
In your *first* container terminal (where you sourced `setup.bash`), run a subscriber:
```bash
# In your FIRST container terminal:
ros2 run demo_nodes_py listener
```
You should see messages being received by the `listener` node.

**3. Listing ROS 2 Nodes:**
In either container, open another terminal and try:
```bash
ros2 node list
```
You should see both `/talker` and `/listener` nodes listed.

**4. Listing ROS 2 Topics:**
```bash
ros2 topic list
```
You should see the `/chatter` topic, among others.

**5. Echoing Topic Messages:**
To see the messages being published on the `/chatter` topic:
```bash
ros2 topic echo /chatter
```
You'll see the messages from the `talker` node being printed.

### Hardware/Cloud Alternative

This lab is designed for a Dockerized Ubuntu 22.04 environment. If you have ROS 2 Humble installed directly on a native Ubuntu 22.04 system, you can execute the `ros2 run`, `ros2 node`, and `ros2 topic` commands directly without using Docker. For cloud environments, ensure your VM has ROS 2 Humble pre-installed or follow official installation guides for your chosen cloud provider.

## Summary

ROS 2 provides a powerful and modular framework for robotics software development through its core communication concepts: Nodes for computation, Topics for asynchronous data streaming, Services for synchronous request/response operations, and Actions for long-running, preemptable tasks with feedback. Understanding these mechanisms is foundational to building complex robot behaviors.

## Assessment / Mini Project

1.  **Question**: Describe a scenario where you would use a ROS 2 Service instead of a Topic, and explain why.
2.  **Question**: What is the purpose of a "message type" in ROS 2 communication?
3.  **Mini-Project**: Modify the `talker` node (conceptually, no actual coding yet) to publish a different type of message (e.g., an integer counter instead of a string). What changes would be required in the `listener` node? (Hint: Think about message types).
