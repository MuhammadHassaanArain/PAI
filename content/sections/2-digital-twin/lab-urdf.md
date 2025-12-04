# Lab 2.2: URDF Modeling: Describing Your Robot

**Chapter ID**: 2.2
**Module**: Digital Twin – Gazebo & Unity
**Authors**: Gemini Agent
**Last Updated**: December 4, 2025

## 🎯 Learning Objectives

By the end of this lab, you will be able to:
*   Understand the fundamental components of a URDF file: links and joints.
*   Create a simple URDF model for a two-link robot arm.
*   Visualize your URDF model using `urdf_to_graphiz` and `rviz2`.
*   Load and spawn your URDF model into Gazebo simulation.

## ✨ Core Concepts

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. This includes its kinematic and dynamic properties, visual appearance, and collision characteristics. A URDF file is essentially a blueprint for your robot, defining its physical structure and how its parts move relative to each other.

*   **Links**: Represent the rigid bodies of the robot (e.g., base, arm segments, end-effector). Each link has mass, inertia, and visual/collision geometries.
*   **Joints**: Describe how links are connected and how they can move relative to each other. Joints define the robot's degrees of freedom.

This lab will guide you through creating a simple URDF and bringing it to life in a simulated environment.

## 💻 Code Examples

The full URDF code is provided within the lab instructions below.

## 🧪 Step-by-Step Lab: Simple Two-Link Robot Arm URDF

**Goal**: Create a URDF file for a simple two-link robot arm and visualize it in `rviz2` and Gazebo.
**Duration**: 45 minutes
**Dependencies**: Completed ROS 2 Humble installation, Gazebo installed and configured for ROS 2.

### 🛠️ Hardware / Cloud Alternative

*   **On-Premise**: Any Ubuntu 22.04 LTS machine with ROS 2 Humble and Gazebo installed.
*   **Cloud**: A virtual machine running Ubuntu 22.04 LTS with ROS 2 Humble and Gazebo installed.

### Instructions

#### Step 1: Create a ROS 2 Package for Your Robot

First, create a new ROS 2 package to hold your URDF files.

1.  Open a new terminal and navigate to your ROS 2 workspace `src` directory:
    *   `cd ~/ros2_ws/src`
2.  Create a new package:
    *   `ros2 pkg create --build-type ament_cmake my_robot_description`
    *   We use `ament_cmake` because URDF files are typically handled as resources for C++ nodes, though Python nodes can also load them.

#### Step 2: Create the URDF File

1.  Navigate into your new package:
    *   `cd my_robot_description`
2.  Create a directory for your URDF files:
    *   `mkdir urdf`
    *   `cd urdf`
3.  Create a new XML file named `simple_arm.urdf` and paste the following content:

    ```xml
    <?xml version="1.0"?>
    <robot name="simple_arm">

      <!-- Base Link -->
      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.1 0.1 0.05"/>
          </geometry>
          <material name="blue">
            <color rgba="0 0 0.8 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="0.1 0.1 0.05"/>
          </geometry>
        </collision>
        <inertial>
          <mass value="0.5"/>
          <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
        </inertial>
      </link>

      <!-- Link 1 -->
      <link name="link1">
        <visual>
          <geometry>
            <cylinder radius="0.02" length="0.2"/>
          </geometry>
          <origin xyz="0 0 0.1" rpy="0 0 0"/>
          <material name="red">
            <color rgba="0.8 0 0 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <cylinder radius="0.02" length="0.2"/>
          </geometry>
          <origin xyz="0 0 0.1" rpy="0 0 0"/>
        </collision>
        <inertial>
          <mass value="0.2"/>
          <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
        </inertial>
      </link>

      <!-- Joint 1: Connects base_link to link1 -->
      <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
      </joint>

      <!-- Link 2 -->
      <link name="link2">
        <visual>
          <geometry>
            <cylinder radius="0.02" length="0.2"/>
          </geometry>
          <origin xyz="0 0 0.1" rpy="0 0 0"/>
          <material name="green">
            <color rgba="0 0.8 0 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <cylinder radius="0.02" length="0.2"/>
          </geometry>
          <origin xyz="0 0 0.1" rpy="0 0 0"/>
        </collision>
        <inertial>
          <mass value="0.1"/>
          <inertia ixx="0.00005" ixy="0.0" ixz="0.0" iyy="0.00005" iyz="0.0" izz="0.00005"/>
        </inertial>
      </link>

      <!-- Joint 2: Connects link1 to link2 -->
      <joint name="joint2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="0 0 0.2" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
      </joint>

    </robot>
    ```

#### Step 3: Visualize URDF (Optional but Recommended)

It's good practice to visualize your URDF to check for errors before loading it into simulation.

1.  Install `urdf_to_graphiz` (if not already installed):
    *   `sudo apt update`
    *   `sudo apt install ros-humble-urdf-to-graphiz`
2.  Generate a visual graph of your URDF:
    *   `ros2 run urdf_to_graphiz urdf_to_graphiz simple_arm.urdf`
    *   This will create a PDF or PNG image (`simple_arm.pdf` or `simple_arm.png`) in your current directory, showing the link-joint tree.
3.  Visualize in `rviz2`:
    *   Open a new terminal and source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
    *   Run `rviz2`
    *   In `rviz2`, click "Add" -> "By topic" -> "/robot_description" (if available, otherwise "By display type" -> "RobotModel").
    *   You might need a `robot_state_publisher` to publish the URDF to `rviz2`.
    *   For this lab, we'll simplify: just using the graphiz tool is sufficient for basic structural check.

#### Step 4: Build Your Package

1.  Navigate back to your workspace root:
    *   `cd ~/ros2_ws`
2.  Build the package (this will make the URDF available to ROS 2):
    *   `colcon build`
    *   `source install/setup.bash`

#### Step 5: Launch Gazebo and Spawn Your Robot

To spawn your robot in Gazebo, you'll typically use a ROS 2 launch file and the `spawn_entity.py` script.

1.  Create a `launch` directory in your `my_robot_description` package:
    *   `cd ~/ros2_ws/src/my_robot_description`
    *   `mkdir launch`
    *   `cd launch`
2.  Create a Python launch file named `display_arm.launch.py`:

    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    def generate_launch_description():
        # Get the path to the URDF file
        urdf_file = os.path.join(
            get_package_share_directory('my_robot_description'),
            'urdf',
            'simple_arm.urdf'
        )

        # Launch arguments
        use_sim_time = LaunchConfiguration('use_sim_time', default='true')

        # Node for publishing robot model to Rviz (optional, for visualization)
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file, 'r').read(),
                         'use_sim_time': use_sim_time}]
        )

        # Node for spawning the robot model into Gazebo
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_arm', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        )

        return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation (Gazebo) clock if true'),
            robot_state_publisher_node,
            spawn_entity_node
        ])
    ```
3.  Update your `~/ros2_ws/src/my_robot_description/setup.py` to include the `launch` files:
    ```python
    from setuptools import find_packages, setup
    import os
    from glob import glob

    package_name = 'my_robot_description'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y]'))),
            (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Your Name',
        maintainer_email='your.email@example.com',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
            ],
        },
    )
    ```
4.  Rebuild your workspace and source it:
    *   `cd ~/ros2_ws`
    *   `colcon build`
    *   `source install/setup.bash`
5.  Launch Gazebo and spawn your robot:
    *   `ros2 launch gazebo_ros gazebo.launch.py` (in one terminal)
    *   `ros2 launch my_robot_description display_arm.launch.py` (in another terminal, after Gazebo starts)

    *   **Expected Output**: Gazebo should launch, and you should see your simple two-link robot arm spawned in the simulator.

## ⚠️ Safety Notes

*   Incorrectly defined collision geometries in URDF can lead to physics engine instability or unexpected behaviors in simulation. Always verify your collision meshes.
*   Manipulating robot models in simulation is safe, but be mindful that the principles transfer to real robots. Always double-check joint limits and ensure stability.

## 📚 Summary

*   URDF is essential for describing robot kinematics, dynamics, visual appearance, and collision properties.
*   Links represent rigid bodies, and joints define their connections and motion.
*   `rviz2` and `urdf_to_graphiz` are useful tools for visualizing URDFs.
*   ROS 2 launch files and `spawn_entity.py` facilitate loading URDF models into Gazebo.

## 📝 Assessment / Mini Project

**Challenge**: Extend the `simple_arm.urdf` to include a simple end-effector (e.g., a gripper represented by two small boxes) connected by a fixed or prismatic joint. Update the launch file to spawn this new robot.

## 🤖 Agent Interaction Examples

The embedded RAG chatbot can assist with your learning. Here are some examples of how to interact with it for this chapter:

*   **Query**: "What is the difference between `<visual>` and `<collision>` elements in URDF?"
*   **Query**: "How can I add a camera sensor to my URDF model?"
*   **Query**: "I'm getting errors when spawning my robot in Gazebo. What should I check?"
*   **Query**: "Explain the concept of `inertial` properties in URDF."
