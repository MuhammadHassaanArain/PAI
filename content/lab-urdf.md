# URDF Robot Modeling Lab: Building a Simple Humanoid

## Learning Objectives

-   Understand the structure of a URDF file.
-   Define links and joints to create a robot model.
-   Visualize your URDF model in Gazebo.

## Core Concepts

URDF (Unified Robot Description Format) is an XML-based file format used in ROS to describe all aspects of a robot. It specifies the robot's kinematic and dynamic properties, visual appearance, and collision geometry. While URDF is great for describing the robot itself, it's often complemented by SDF (Simulation Description Format) for defining environments and richer simulation properties in Gazebo.

**Key Components of URDF:**

1.  **Links**: Represent the rigid bodies of the robot (e.g., torso, head, arm segments). Each link has physical properties (mass, inertia), visual properties (geometry, color, texture), and collision properties (geometry for physics interactions).
2.  **Joints**: Represent the connections between links. Joints define the type of motion allowed between two links (e.g., `revolute` for rotation, `prismatic` for linear motion, `fixed` for no relative motion). Each joint has an `origin` (position and orientation relative to its parent link), `parent` and `child` links, and `axis` of rotation/translation.
3.  **Transmission**: (Optional but important) Describes the relationship between an actuator and a joint.
4.  **Gazebo Extensions**: URDF files can include `<gazebo>` tags to add specific properties for Gazebo simulation, such as materials, friction coefficients, and sensor definitions.

Building a URDF model is a foundational skill for anyone working with robots in simulation, as it's the blueprint for your robot's physical embodiment in the digital twin.

## Step-by-Step Lab: Designing a Simple Two-Link Arm

We will create a very simple two-link arm, which forms the basis of more complex humanoid arms.

### Code Examples

First, ensure you are in your ROS 2 Humble Docker container or native environment.

```bash
# Enter your Docker container (if not already in it)
# docker run -it --rm --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" osrf/ros:humble-desktop-full bash
# source /opt/ros/humble/setup.bash

# Create a package for your robot description
mkdir -p ~/ros2_ws/src/my_robot_description/urdf
cd ~/ros2_ws/src/my_robot_description
ros2 pkg create --build-type ament_cmake --dependencies rclpy my_robot_description
```

Now, create a URDF file named `two_link_arm.urdf` in `~/ros2_ws/src/my_robot_description/urdf/`.

```xml
<?xml version="1.0"?>
<robot name="two_link_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="0.5"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 2 -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
  </joint>

  <!-- Link 2 -->
  <link name="link2">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.02"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

Now, we need a launch file to display this in Gazebo. Create `display_arm.launch.py` in `~/ros2_ws/src/my_robot_description/launch/`.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'two_link_arm.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
```

**Build your package:**

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

**Launch the robot in RViz2 (visualization tool):**

```bash
ros2 launch my_robot_description display_arm.launch.py
```
You should see a new RViz2 window pop up, displaying your two-link arm. You can use the GUI in RViz2 to move the joints and see your robot articulate.

### Hardware/Cloud Alternative

URDF modeling is primarily a descriptive task. Visualizing it in RViz2 or Gazebo requires a graphical environment. The same Docker setup with X-server forwarding will work. For cloud environments, ensure you have a VNC server or similar GUI access set up to view the RViz2 window.

## Summary

URDF is the standard for describing robot kinematics, dynamics, and visuals in ROS. By defining links and joints, you can create a detailed model of your robot, which is essential for accurate simulation and control. Visualization tools like RViz2 help you verify your model.

## Assessment / Mini Project

1.  **Question**: What is the purpose of the `joint_state_publisher_gui` in the launch file?
2.  **Question**: How would you add an additional link (e.g., an end-effector) to the `two_link_arm.urdf`?
3.  **Mini-Project**: Modify the `two_link_arm.urdf` file to change the dimensions or colors of the links. Rebuild and relaunch the RViz2 visualization to observe your changes.
