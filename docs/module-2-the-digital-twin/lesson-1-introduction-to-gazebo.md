---
title: Lesson 1 - Introduction to Gazebo
sidebar_position: 2
description: Setting up simulation environments for humanoid robotics
learning_objectives:
  - Install and configure Gazebo for robotics simulation
  - Create basic simulation worlds
  - Spawn and control robots in simulation
  - Understand Gazebo's integration with ROS
duration: 120
---

# Lesson 1 - Introduction to Gazebo

## Learning Objectives

After completing this lesson, you will be able to:
- Install and configure Gazebo for robotics simulation
- Create basic simulation worlds
- Spawn and control robots in simulation
- Understand Gazebo's integration with ROS

## Introduction

Gazebo is a powerful, open-source robotics simulator that provides realistic physics simulation, high-quality 3D rendering, and convenient interfaces for robotics research and development. It's widely used in the robotics community for testing algorithms, validating robot designs, and training AI agents before deployment to real hardware.

## Key Features of Gazebo

Gazebo provides a range of features that make it ideal for robotics simulation:

- **Realistic Physics**: Accurate simulation of rigid body dynamics, collisions, and contact forces
- **High-Quality Rendering**: OpenGL-based 3D visualization with lighting and shadows
- **Sensor Simulation**: Support for various sensors including cameras, LiDAR, IMUs, and GPS
- **Robot Models**: Support for URDF and SDF robot descriptions
- **ROS Integration**: Direct integration with ROS and ROS 2 for seamless simulation
- **Extensibility**: Plugin architecture allows custom sensors and controllers

## Gazebo Architecture

Gazebo's architecture consists of several key components:

- **Server (gzserver)**: Handles physics simulation and model updates
- **Client (gzclient)**: Provides visualization interface
- **Plugins**: Extend functionality for sensors, controllers, and GUI elements
- **Models**: 3D representations of robots and objects
- **Worlds**: Environment descriptions including physical properties and objects

## Installing Gazebo

Gazebo can be installed as part of a ROS distribution or independently. For ROS 2 users, it's typically installed with the desktop variant:

```bash
sudo apt install ros-<ros2-distro>-gazebo-*
```

## Basic Gazebo Workflow

The typical workflow for using Gazebo includes:

1. Creating or obtaining robot models in URDF/SDF format
2. Designing simulation worlds with environments and objects
3. Spawning robots into the simulation
4. Controlling robots through ROS interfaces
5. Collecting sensor data and evaluating performance

## Integration with ROS

Gazebo integrates tightly with ROS through the `gazebo_ros` package, which provides:

- Publishers and subscribers for controlling simulation
- Plugins for common sensors (camera, LiDAR, etc.)
- Tools for spawning and deleting models
- TF publishing for robot states
- Controller interfaces for joint control

## Hands-On Exercise

1. **Launch Gazebo**: Start Gazebo with an empty world:
   ```bash
   gazebo
   ```

2. **Explore the Interface**: Familiarize yourself with the Gazebo interface:
   - Toolbar with simulation controls
   - Main 3D view
   - Model database on the right
   - World properties panel

3. **Add Objects**: Add some basic objects to the world:
   - Use the Insert tab to add shapes (box, sphere, cylinder)
   - Modify their properties like size, color, and position
   - Observe how they interact physically

4. **Simulate Physics**: Play the simulation and observe physics:
   - Pause and resume the simulation
   - Add objects while simulation is running
   - Use the selection tool to move objects during simulation

5. **Save the World**: Save your custom world:
   - File → Save World As
   - This will create a .world file for your environment

## Exercises

1. **World Creation**: Create a simple room environment with walls, floor, and obstacles. How would you model a realistic indoor environment?

2. **Model Integration**: Add a simple robot model to your world. How does it interact with the environment physically?

3. **ROS Interface**: Explore the ROS topics that Gazebo exposes. What topics are available for controlling the simulation?

4. **Physics Parameters**: Experiment with different physics parameters (gravity, friction). How do these affect the simulation?

## Summary

Gazebo provides a powerful platform for simulating robotic systems. Understanding its basic functionality and integration with ROS is essential for developing and testing humanoid robots in a safe, controlled environment.

## Self-Assessment

1. What are the main components of Gazebo's architecture?
2. How does Gazebo integrate with ROS?
3. What are the advantages of using simulation in robotics development?
4. What types of sensors can be simulated in Gazebo?