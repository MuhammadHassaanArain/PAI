# Lab 1.2: Basic ROS 2 Publisher/Subscriber in Python

**Chapter ID**: 1.2
**Module**: ROS 2 – Robotic Nervous System
**Authors**: Gemini Agent
**Last Updated**: December 4, 2025

## 🎯 Learning Objectives

By the end of this lab, you will be able to:
*   Set up a basic ROS 2 workspace and package.
*   Create a ROS 2 Python node that publishes messages to a topic.
*   Create a ROS 2 Python node that subscribes to a topic and processes messages.
*   Verify communication between publisher and subscriber nodes using ROS 2 command-line tools.

## ✨ Core Concepts

This lab applies the core ROS 2 concepts of Nodes, Topics, Publishers, and Subscribers introduced in Chapter 1.1. We will specifically focus on implementing a simple "talker" (publisher) and "listener" (subscriber) system, which is a classic first example in ROS.

## 💻 Code Examples

The full code for the publisher and subscriber nodes is provided within the lab instructions below.

## 🧪 Step-by-Step Lab: ROS 2 Basic Publisher/Subscriber

**Goal**: Implement and verify a simple ROS 2 publisher/subscriber system where a "talker" node sends messages and a "listener" node receives them.
**Duration**: 30 minutes
**Dependencies**: Completed ROS 2 Humble installation on Ubuntu 22.04 LTS. Familiarity with basic Linux command-line operations.

### 🛠️ Hardware / Cloud Alternative

*   **On-Premise**: Any Ubuntu 22.04 LTS machine with ROS 2 Humble installed. No specific hardware is required beyond the host machine.
*   **Cloud**: A virtual machine running Ubuntu 22.04 LTS with ROS 2 Humble installed (e.g., on AWS EC2, Google Cloud, Azure).

### Instructions

#### Step 1: Create a ROS 2 Workspace

A ROS 2 workspace is a directory where you develop your ROS 2 packages.

1.  Open a new terminal.
2.  Create a new directory for your workspace and navigate into it:
    *   `mkdir -p ~/ros2_ws/src`
    *   `cd ~/ros2_ws/src`

#### Step 2: Create a ROS 2 Package

Now, create a new Python package within your workspace.

1.  From `~/ros2_ws/src`, create the package:
    *   `ros2 pkg create --build-type ament_python my_python_pkg --dependencies rclpy std_msgs`
    *   This command creates a new Python package named `my_python_pkg` with dependencies on `rclpy` (ROS Client Library for Python) and `std_msgs` (standard message types).

#### Step 3: Write the Publisher Node

1.  Navigate into your new package's source directory:
    *   `cd my_python_pkg/my_python_pkg`
2.  Create a new Python file named `talker.py` and paste the following code:
    ```python
    # ~/ros2_ws/src/my_python_pkg/my_python_pkg/talker.py
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class Talker(Node):

        def __init__(self):
            super().__init__('talker_node')
            self.publisher_ = self.create_publisher(String, 'chatter', 10)
            self.timer_period = 0.5  # seconds
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            self.i = 0
            self.get_logger().info('Talker node initialized and publishing to /chatter topic.')

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello from Talker! Count: {self.i}'
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

#### Step 4: Write the Subscriber Node

1.  In the same directory (`~/ros2_ws/src/my_python_pkg/my_python_pkg`), create a new Python file named `listener.py` and paste the following code:
    ```python
    # ~/ros2_ws/src/my_python_pkg/my_python_pkg/listener.py
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class Listener(Node):

        def __init__(self):
            super().__init__('listener_node')
            self.subscription = self.create_subscription(
                String,
                'chatter',
                self.listener_callback,
                10)
            self.get_logger().info('Listener node initialized and subscribing to /chatter topic.')
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

#### Step 5: Update `setup.py`

You need to tell ROS 2 about your new executable scripts.

1.  Navigate back to the package root:
    *   `cd ~/ros2_ws/src/my_python_pkg`
2.  Open `setup.py` and add the `listener.py` and `talker.py` scripts to the `entry_points` section. The `entry_points` section should look like this:
    ```python
    # ~/ros2_ws/src/my_python_pkg/setup.py
    # ... (other imports and package info)
        entry_points={
            'console_scripts': [
                'talker = my_python_pkg.talker:main',
                'listener = my_python_pkg.listener:main',
            ],
        },
    # ...
    ```

#### Step 6: Build the Workspace

Now, build your ROS 2 package.

1.  Navigate to the root of your workspace:
    *   `cd ~/ros2_ws`
2.  Build the package:
    *   `colcon build`
    *   If successful, you should see output indicating `my_python_pkg` was built.

#### Step 7: Source the Workspace

After building, you need to source the workspace to make your new executables available in your current terminal session.

1.  From the `~/ros2_ws` directory:
    *   `source install/setup.bash`

#### Step 8: Run the Publisher and Subscriber

You'll need two separate terminals for this step.

1.  **Terminal 1 (Publisher)**:
    *   Open a new terminal (and repeat Step 7 `source install/setup.bash` if you haven't sourced your workspace in this new terminal).
    *   Run the talker node:
        *   `ros2 run my_python_pkg talker`
    *   *Expected Output (Terminal 1)*: You should see messages like `[INFO] [talker_node]: Publishing: "Hello from Talker! Count: 0"`, updating every 0.5 seconds.

2.  **Terminal 2 (Subscriber)**:
    *   Open another new terminal (and repeat Step 7 `source install/setup.bash` if you haven't sourced your workspace in this new terminal).
    *   Run the listener node:
        *   `ros2 run my_python_pkg listener`
    *   *Expected Output (Terminal 2)*: You should see messages like `[INFO] [listener_node]: I heard: "Hello from Talker! Count: 0"`, echoing what the talker publishes.

#### Step 9: Verify with ROS 2 Tools (Optional)

You can use `ros2 topic` to inspect the communication.

1.  While the publisher is running in Terminal 1, open a third terminal (and source the workspace).
2.  List active topics:
    *   `ros2 topic list`
    *   *Expected Output*: You should see `/chatter` in the list.
3.  Echo messages on the `chatter` topic:
    *   `ros2 topic echo /chatter`
    *   *Expected Output*: You should see the messages being published by the talker.

## ⚠️ Safety Notes

*   This lab is purely software-based and does not involve physical hardware. However, it's a fundamental step in understanding how to safely control and monitor robotic systems.
*   Always ensure your `colcon build` and `source` commands are successful before running ROS 2 nodes to avoid unexpected errors.

## 📚 Summary

*   You have successfully created a ROS 2 workspace and package.
*   You implemented a publisher (talker) and subscriber (listener) node.
*   You verified inter-node communication using ROS 2 runtime tools.
*   This establishes a foundational understanding of ROS 2's publish-subscribe model.

## 📝 Assessment / Mini Project

**Challenge**: Modify the `talker.py` node to publish a different message type (e.g., `Int32` from `std_msgs.msg`) and update the `listener.py` node to subscribe to this new message type. Ensure the system still communicates correctly.

## 🤖 Agent Interaction Examples

The embedded RAG chatbot can assist with your learning. Here are some examples of how to interact with it for this chapter:

*   **Query**: "I'm getting a `ModuleNotFoundError` when running `ros2 run`. What should I check?"
*   **Query**: "Explain the purpose of `self.create_publisher` in `talker.py`."
*   **Query**: "How can I check if my topic is active using the command line?"
*   **Query**: "Can you give me more details about `colcon build`?"
