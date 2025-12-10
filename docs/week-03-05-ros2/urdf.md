---
sidebar_position: 3
title: URDF Robot Descriptions
description: Learn to create robot models using URDF (Unified Robot Description Format) for visualization and simulation
keywords: [urdf, robot model, ros2, xacro, visualization, simulation]
---

# URDF Robot Descriptions

<div className="chapter-meta">
  <div className="chapter-meta-item">
    <strong>Week:</strong> 3-5
  </div>
  <div className="chapter-meta-item">
    <strong>Duration:</strong> ~4 hours
  </div>
  <div className="chapter-meta-item">
    <strong>Level:</strong> Intermediate
  </div>
</div>

## Learning Objectives

<div className="learning-objectives">

By the end of this chapter, you will be able to:

- Understand the URDF XML format and its key elements
- Create robot models with links, joints, and visual/collision geometry
- Use Xacro macros for modular, maintainable robot descriptions
- Visualize robots in RViz and simulate them in Gazebo
- Add sensors and controllers to your robot model

</div>

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML format for describing robots:

- **Links**: Rigid bodies with visual and collision geometry
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Materials**: Colors and textures
- **Sensors**: Cameras, LiDAR, IMU definitions

### Why URDF Matters

| Use Case | Description |
|----------|-------------|
| Visualization | Display robot in RViz |
| Simulation | Physics simulation in Gazebo |
| Motion Planning | Collision checking, IK |
| Control | Joint limits, dynamics |

## Basic URDF Structure

### Minimal Robot Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Wheel Link -->
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting base to wheel -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Link Elements

### Visual Geometry

Defines how the link appears in visualization:

```xml
<link name="arm_link">
  <visual>
    <!-- Basic shapes -->
    <geometry>
      <box size="0.1 0.1 0.5"/>        <!-- x, y, z -->
      <!-- OR -->
      <cylinder radius="0.05" length="0.4"/>
      <!-- OR -->
      <sphere radius="0.08"/>
      <!-- OR mesh file -->
      <mesh filename="package://my_robot/meshes/arm.stl" scale="1.0 1.0 1.0"/>
    </geometry>

    <!-- Position/orientation relative to link origin -->
    <origin xyz="0 0 0.25" rpy="0 0 0"/>

    <!-- Material -->
    <material name="silver">
      <color rgba="0.8 0.8 0.8 1.0"/>
    </material>
  </visual>
</link>
```

### Collision Geometry

Defines the shape used for collision detection (often simplified):

```xml
<collision>
  <geometry>
    <!-- Use simpler geometry for faster collision checking -->
    <box size="0.12 0.12 0.52"/>
  </geometry>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
</collision>
```

### Inertial Properties

Required for physics simulation:

```xml
<inertial>
  <mass value="2.5"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
  <inertia
    ixx="0.05" ixy="0" ixz="0"
    iyy="0.05" iyz="0"
    izz="0.02"/>
</inertial>
```

## Joint Types

### Revolute Joint (Rotation with Limits)

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation axis -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.1"/>
</joint>
```

### Continuous Joint (Unlimited Rotation)

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="chassis_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.2 0.15 0" rpy="-1.5708 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### Prismatic Joint (Linear Motion)

```xml
<joint name="lift_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="platform_link"/>
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Direction of motion -->
  <limit lower="0" upper="0.5" effort="500" velocity="0.1"/>
</joint>
```

### Fixed Joint (No Motion)

```xml
<joint name="sensor_mount" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>
```

## Using Xacro

**Xacro** (XML Macros) makes URDF more maintainable:

### Properties (Variables)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Define properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="chassis_length" value="0.5"/>
  <xacro:property name="chassis_width" value="0.3"/>
  <xacro:property name="chassis_height" value="0.1"/>

  <!-- Use properties -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
    </visual>
  </link>

</robot>
```

### Macros (Reusable Components)

```xml
<!-- Define a wheel macro -->
<xacro:macro name="wheel" params="name parent x y">
  <link name="${name}_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="${name}_wheel_joint" type="continuous">
    <parent link="${parent}"/>
    <child link="${name}_wheel_link"/>
    <origin xyz="${x} ${y} 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</xacro:macro>

<!-- Use the macro to create four wheels -->
<xacro:wheel name="front_left" parent="base_link" x="0.15" y="0.2"/>
<xacro:wheel name="front_right" parent="base_link" x="0.15" y="-0.2"/>
<xacro:wheel name="rear_left" parent="base_link" x="-0.15" y="0.2"/>
<xacro:wheel name="rear_right" parent="base_link" x="-0.15" y="-0.2"/>
```

### Include Files

```xml
<!-- main_robot.urdf.xacro -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <xacro:include filename="$(find my_robot)/urdf/materials.xacro"/>
  <xacro:include filename="$(find my_robot)/urdf/chassis.xacro"/>
  <xacro:include filename="$(find my_robot)/urdf/wheels.xacro"/>
  <xacro:include filename="$(find my_robot)/urdf/sensors.xacro"/>
</robot>
```

## Adding Sensors

### Camera

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.1 0.02"/>
    </geometry>
    <material name="red"/>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.25 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for camera -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Visualization in RViz

### Launch File for Robot Display

```python
# launch/display.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    robot_description = Command(['xacro ', urdf_path])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'display.rviz')]
        ),
    ])
```

### Run Visualization

```bash
ros2 launch my_robot display.launch.py
```

## Summary

URDF provides a standardized way to describe robots:

- **Links** define rigid bodies with geometry and inertia
- **Joints** connect links with various motion types
- **Xacro** enables modular, maintainable descriptions
- **Sensors** can be added with Gazebo plugins
- **RViz** and **Gazebo** use URDF for visualization and simulation

## Exercises

1. **Basic Model**: Create a URDF for a simple two-wheeled robot with a caster.

2. **Xacro Practice**: Convert the basic model to use Xacro with properties and macros.

3. **Arm Model**: Create a 3-DOF robot arm with revolute joints.

4. **Visualization**: Launch your robot in RViz and use joint_state_publisher_gui to test joint motion.
