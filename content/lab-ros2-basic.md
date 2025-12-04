# ROS 2 Hands-On Lab: Simple Publisher/Subscriber in Python

## Learning Objectives

-   Create a ROS 2 Python package.
-   Write a simple ROS 2 publisher node.
-   Write a simple ROS 2 subscriber node.
-   Understand how to build and run ROS 2 Python executables.

## Core Concepts

In this lab, you will put the core ROS 2 concepts of nodes, topics, publishers, and subscribers into practice by building a basic communication system. You will create two Python nodes: a "talker" that publishes messages to a topic, and a "listener" that subscribes to that topic and prints the received messages.

This hands-on exercise reinforces the modular nature of ROS 2 applications, where independent nodes communicate via well-defined interfaces (topics and message types).

## Step-by-Step Lab: Building a Talker and Listener

### 1. Set Up Your ROS 2 Workspace

First, ensure you have a working ROS 2 Humble environment (e.g., inside the Docker container from previous labs).

```bash
# Enter your Docker container (if not already in it)
docker run -it --rm osrf/ros:humble-desktop-full bash
source /opt/ros/humble/setup.bash

# Create a new ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Create a New ROS 2 Python Package

We'll create a package named `my_py_pkg`.

```bash
ros2 pkg create --build-type ament_python my_py_pkg
```

### 3. Write the Publisher Node (`talker.py`)

Navigate into your new package's directory: `cd my_py_pkg`.

Create a file named `talker.py` inside the `my_py_pkg/my_py_pkg/` directory (note the nested directory, it's common for Python packages).

```python
# ~/ros2_ws/src/my_py_pkg/my_py_pkg/talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Write the Subscriber Node (`listener.py`)

Create a file named `listener.py` inside the `my_py_pkg/my_py_pkg/` directory.

```python
# ~/ros2_ws/src/my_py_pkg/my_py_pkg/listener.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Update `setup.py`

Modify the `setup.py` file in your `~/ros2_ws/src/my_py_pkg/` directory to include your new executables.
Add the following `entry_points` dictionary.

```python
# ~/ros2_ws/src/my_py_pkg/setup.py
from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch_file.launch.py']), # Example launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A minimal ROS 2 Python package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_py_pkg.talker:main',
            'listener = my_py_pkg.listener:main',
        ],
    },
)
```

### 6. Build Your ROS 2 Package

Navigate back to your workspace root and build:

```bash
cd ~/ros2_ws
colcon build --packages-select my_py_pkg
```

### 7. Run the Nodes

Source your workspace and then run the nodes in separate terminals:

```bash
# In the first terminal:
cd ~/ros2_ws
source install/setup.bash
ros2 run my_py_pkg talker

# In a second terminal (after sourcing workspace):
cd ~/ros2_ws
source install/setup.bash
ros2 run my_py_pkg listener
```

You should see the `talker` publishing messages and the `listener` receiving and printing them.

## Hardware/Cloud Alternative

This lab is designed to run in a Docker environment on Ubuntu 22.04. If you have a native ROS 2 Humble installation, you can perform the same steps directly. For cloud instances, ensure you have a graphical interface configured if you wish to visualize the `rqt_graph` tool (optional visualization step not included here, but useful for debugging).

## Summary

You have successfully created, built, and run your first ROS 2 Python publisher and subscriber nodes. This hands-on experience demonstrates the fundamental communication patterns that form the backbone of any ROS 2 robotics application.

## Assessment / Mini Project

1.  **Question**: Explain how the `entry_points` in `setup.py` connect your Python scripts to `ros2 run`.
2.  **Question**: How would you change the `talker` node to publish messages less frequently (e.g., every 2 seconds)?
3.  **Mini-Project**: Modify the `talker` and `listener` nodes to communicate using a custom message type (e.g., a message containing `x`, `y`, `z` coordinates) instead of `std_msgs/String`. You will need to create a `msg` directory and define a `.msg` file. (Hint: Refer to ROS 2 documentation on custom messages).
