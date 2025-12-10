---
title: Lesson 4 - Understanding URDF (Unified Robot Description Format) for Humanoids
sidebar_position: 5
description: Describing robot structure, kinematics, and dynamics using URDF
learning_objectives:
  - Define robot structure using URDF
  - Describe kinematic chains for humanoid robots
  - Include visual and collision properties in URDF
  - Use Xacro to simplify complex URDF definitions
duration: 200
---

# Lesson 4 - Understanding URDF (Unified Robot Description Format) for Humanoids

## Learning Objectives

After completing this lesson, you will be able to:
- Define robot structure using URDF
- Describe kinematic chains for humanoid robots
- Include visual and collision properties in URDF
- Use Xacro to simplify complex URDF definitions

## Introduction

The Unified Robot Description Format (URDF) is an XML specification that describes robots in ROS. It defines the physical structure of a robot, including its links, joints, inertial properties, visual representation, and collision properties. For humanoid robots with their complex multi-degree-of-freedom structures, URDF is essential for simulation, visualization, and control.

## URDF Fundamentals

URDF describes robots in terms of:
- **Links**: Rigid components of the robot (e.g., torso, arm segments, head)
- **Joints**: Connections between links that allow motion
- **Materials**: Visual appearance properties
- **Gazebos**: Simulation-specific properties

## Links

Links represent rigid bodies in the robot. Each link has:
- A name
- Inertial properties (mass, center of mass, inertia matrix)
- Visual properties (shape, color, mesh)
- Collision properties (shape for collision detection)

## Joints

Joints connect links and define their relative motion. Joint types include:
- **Fixed**: No motion allowed
- **Revolute**: Single degree of freedom rotation, limited by upper/lower bounds
- **Continuous**: Single degree of freedom rotation, unlimited range
- **Prismatic**: Single degree of freedom translation, limited by upper/lower bounds
- **Planar**: Motion on a plane
- **Floating**: Six degrees of freedom without limits

## URDF for Humanoid Robots

Humanoid robots require special considerations in URDF:

Below is a textual representation of URDF Structure for Humanoid:

```text
# URDF Structure Diagram

## Humanoid Robot Links and Joints:
       head
         |
         | (fixed joint)
         |
      neck (torso_top)
         |
         | (fixed joint)
         |
       torso
      /     \
     /       \
shoulder    shoulder
   |           |
   |           |
upper_arm  upper_arm
   |           |
   |           |
elbow       elbow
   |           |
   |           |
lower_arm lower_arm
   |           |
   |           |
wrist      wrist
   |           |
   |           |
  hand      hand

       torso
         |
         | (revolute joint)
         |
        hip
       /   \
      /     \
upper_leg upper_leg
     |       |
     |       |
   knee    knee
     |       |
     |       |
lower_leg lower-leg
     |       |
     |       |
   ankle   ankle
     |       |
     |       |
   foot    foot

## Key Elements:
- Links: Rigid bodies (boxes)
- Joints: Connections allowing movement (circles)
- Fixed joints: No movement allowed
- Revolute joints: Single axis rotation
```

*Figure 1: Example URDF structure showing links and joints for a humanoid robot.*

### Kinematic Chains
Humanoids typically have multiple kinematic chains:
- Left arm chain (torso → shoulder → arm → forearm → hand)
- Right arm chain (torso → shoulder → arm → forearm → hand)
- Left leg chain (torso → hip → thigh → shank → foot)
- Right leg chain (torso → hip → thigh → shank → foot)
- Head chain (torso → neck → head)

### Degrees of Freedom
Humanoid robots require many joints to replicate human-like motion:
- Multiple joints in the neck for head movement
- Shoulders with multiple degrees of freedom
- Elbows, wrists with appropriate joint types
- Hip, knee, ankle joints for leg movement
- Often some joints in the torso for flexibility

## Xacro: XML Macros for URDF

Xacro is an XML macro language that simplifies complex URDF definitions by allowing:
- Reusable macros for repeated elements
- Mathematical expressions
- Conditional blocks
- File inclusion

Xacro files use the .xacro extension and are processed to generate URDF files.

## Visual and Collision Properties

For proper simulation and visualization:
- Visual models provide the appearance of each link
- Collision models define how the physics engine handles contact
- These models can have different levels of detail

## URDF in Simulation and Control

URDF is used by:
- Gazebo for physics simulation
- RViz for visualization
- Kinematic libraries (like KDL, URDF parser) for forward and inverse kinematics
- Planning algorithms that need to understand robot structure

## Best Practices for Humanoid URDF

- Use consistent naming conventions
- Define appropriate joint limits based on physical constraints
- Include inertial properties for accurate simulation
- Use appropriate mesh scales and origins
- Organize complex URDFs with Xacro
- Validate URDF with tools like check_urdf

## Hands-On Exercise

Create a simplified URDF for a humanoid robot:

1. Create a directory for your URDF files:
   ```bash
   mkdir -p ~/robot_models/humanoid_robot/urdf
   cd ~/robot_models/humanoid_robot/urdf
   ```

2. Create a basic URDF file `simple_humanoid.urdf` with a torso, head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="torso_head_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

3. Create a Xacro version `simple_humanoid.xacro` to demonstrate how Xacro simplifies complex URDFs:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid_xacro">

  <!-- Define constants -->
  <xacro:property name="M_PI" value="3.14159"/>
  <xacro:property name="torso_height" value="0.5"/>
  <xacro:property name="torso_width" value="0.3"/>
  <xacro:property name="torso_depth" value="0.2"/>

  <!-- Macro for defining a limb -->
  <xacro:macro name="limb" params="name parent side_length side_radius joint_origin_x joint_origin_y joint_origin_z joint_axis_xyz">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="${side_length}" radius="${side_radius}"/>
        </geometry>
        <material name="red">
          <color rgba="1 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${side_length}" radius="${side_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>
    <joint name="${name}_joint" type="revolute">
      <parent link="torso"/>
      <child link="${name}"/>
      <origin xyz="${joint_origin_x} ${joint_origin_y} ${joint_origin_z}"/>
      <axis xyz="${joint_axis_xyz}"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="torso_head_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
  </joint>

  <!-- Use the macro to create limbs -->
  <xacro:limb name="left_upper_arm" parent="torso"
           side_length="0.3" side_radius="0.05"
           joint_origin_x="${torso_width/2}" joint_origin_y="0" joint_origin_z="0.1"
           joint_axis_xyz="0 1 0"/>

  <xacro:limb name="right_upper_arm" parent="torso"
           side_length="0.3" side_radius="0.05"
           joint_origin_x="${-torso_width/2}" joint_origin_y="0" joint_origin_z="0.1"
           joint_axis_xyz="0 1 0"/>

  <xacro:limb name="left_upper_leg" parent="torso"
           side_length="0.4" side_radius="0.06"
           joint_origin_x="0.1" joint_origin_y="0" joint_origin_z="${-torso_height/2}"
           joint_axis_xyz="0 1 0"/>

  <xacro:limb name="right_upper_leg" parent="torso"
           side_length="0.4" side_radius="0.06"
           joint_origin_x="-0.1" joint_origin_y="0" joint_origin_z="${-torso_height/2}"
           joint_axis_xyz="0 1 0"/>

</robot>
```

4. Validate the URDF:
   ```bash
   check_urdf simple_humanoid.urdf
   ```

5. Visualize the URDF in RViz:
   ```bash
   # Launch RViz with the robot state publisher
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat simple_humanoid.urdf)
   ros2 run rviz2 rviz2
   ```

6. Observe the simplified humanoid robot model and how the Xacro version reduces repetition.

## Summary

URDF is fundamental to representing humanoid robots in ROS. It provides a standard format for describing robot structure, enabling simulation, visualization, and control. Understanding URDF is essential for working with humanoid robots in ROS-based systems.

## Exercises

1. **Extension Task**: Extend the simple humanoid URDF to include additional joints (e.g., elbows, knees) to create a more complete robot model. How does adding joints change the kinematic model?

2. **Real Robot Modeling**: Choose a real humanoid robot (e.g., NAO, Pepper, or Atlas) and research its physical specifications. Create a simplified URDF model that captures its essential features.

3. **Xacro Challenge**: Convert the entire simple humanoid URDF from the exercise into a more sophisticated Xacro file using additional macros and parameters to make it more reusable and maintainable.

4. **Simulation Task**: Create a simple Gazebo world file to simulate your humanoid robot. What additional elements do you need to include for simulation beyond the basic URDF?

## Self-Assessment

1. What is URDF and what does it describe?
2. What are the main components of a URDF file?
3. What is the difference between visual and collision models?
4. How does Xacro help simplify complex URDFs?