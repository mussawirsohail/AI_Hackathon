---
title: Lesson 2 - Physics Simulation in Gazebo
sidebar_position: 3
description: Modeling gravity, collisions, and physical interactions in simulation
learning_objectives:
  - Configure physical properties for realistic simulation
  - Implement collision detection and response
  - Model contact forces and friction
  - Optimize simulation for performance
duration: 150
---

# Lesson 2 - Physics Simulation in Gazebo

## Learning Objectives

After completing this lesson, you will be able to:
- Configure physical properties for realistic simulation
- Implement collision detection and response
- Model contact forces and friction
- Optimize simulation for performance

## Introduction

Physics simulation is the core functionality of Gazebo, enabling realistic modeling of robot interactions with the environment. Accurate physics simulation is crucial for humanoid robots, which must interact with the world through complex contact points and dynamic behaviors.

## Physics Engine Fundamentals

Gazebo uses the Open Dynamics Engine (ODE) as its default physics engine, though other options like Bullet and DART are also supported. Key physics concepts include:

### Rigid Body Dynamics
Rigid body dynamics describes the motion of solid objects under applied forces. In Gazebo, every object is treated as a rigid body with:
- Mass and inertia properties
- Position and orientation
- Linear and angular velocities
- Applied forces and torques

### Collision Detection
Collision detection algorithms determine when objects intersect or come into contact. Gazebo uses multiple approaches:
- Broad phase: Quick elimination of non-colliding pairs
- Narrow phase: Precise contact point determination
- Continuous collision detection: Prevents tunneling at high speeds

### Contact Response
When collisions are detected, the physics engine computes contact forces to prevent objects from penetrating each other. This includes:
- Normal forces to prevent penetration
- Friction forces to simulate surface interactions
- Restitution (bounciness) parameters

## Physical Properties Configuration

Accurate simulation requires proper configuration of physical properties:

### Mass Properties
- Mass: Total amount of matter in the object
- Inertia: Resistance to rotational motion
- Center of mass: Point where mass is concentrated

### Material Properties
- Density: Mass per unit volume
- Friction: Resistance to sliding motion
- Restitution: Bounciness coefficient

## Configuring Physics in SDF

Simulation properties are defined in SDF (Simulation Description Format):

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

### Time Step Settings
- **max_step_size**: Size of each simulation step (smaller = more accurate but slower)
- **real_time_factor**: Target simulation speed (1.0 = real-time)
- **real_time_update_rate**: Updates per second

### Gravity Configuration
Gravity is defined as a 3D vector (x, y, z) in m/s². For Earth gravity: `<gravity>0 0 -9.8</gravity>`

## Collision and Visual Models

Objects in Gazebo have separate collision and visual properties:

### Collision Models
- Define the shape used for physics calculations
- Should be simple for computational efficiency
- Can be different from visual representation

### Visual Models
- Define the appearance of objects
- Do not affect physics simulation
- Can be complex with detailed textures

## Contact Mechanics

Accurate modeling of contact forces is crucial for humanoid robots:

### Friction Models
- **Static friction**: Force needed to start motion
- **Dynamic friction**: Force during sliding motion
- Mu (μ) parameters control friction strength

### Contact Parameters
- **kp**: Contact stiffness
- **kd**: Contact damping
- **max_vel**: Maximum contact correction velocity
- **min_depth**: Minimum contact depth for force application

## Optimization Strategies

### Performance Optimization
- Use simple collision geometries (boxes, spheres, cylinders)
- Reduce the number of contact points where possible
- Adjust time step and update rate for desired accuracy/performance trade-off
- Limit the number of active objects in the simulation

### Accuracy Optimization
- Use appropriate time steps for the system being simulated
- Configure contact parameters based on real-world material properties
- Use continuous collision detection for fast-moving objects
- Implement adaptive step size if available

## Simulation Challenges for Humanoids

Humanoid robots present unique simulation challenges:

### Contact Stability
- Multiple contact points (feet, hands, etc.)
- Balancing during dynamic motions
- Transitions between single and double support

### Computational Complexity
- Many degrees of freedom
- Complex kinematic chains
- Real-time performance requirements

## Hands-On Exercise

1. **Physics Parameter Tuning**: Create a simulation with a humanoid robot model and experiment with different physics parameters:
   ```bash
   # Launch Gazebo with your robot
   ros2 launch your_robot_gazebo your_robot_world.launch.py
   ```
   
2. **Collision Detection**: Create objects with different collision properties and observe how they interact. Pay attention to:
   - Friction coefficients
   - Restitution values
   - Mass properties

3. **Contact Analysis**: Use Gazebo's contact sensor plugin to visualize and analyze contact forces between your robot and the environment.

4. **Performance Testing**: Run the simulation with different time step settings and observe the impact on:
   - Real-time factor
   - Computer resource usage
   - Simulation stability

5. **Stability Testing**: Test your humanoid robot's balance by adjusting:
   - Center of mass properties
   - Foot friction parameters
   - Joint stiffness

## Exercises

1. **Parameter Optimization**: For a humanoid robot model, determine optimal physics parameters that provide both stability and performance. Document your findings.

2. **Real-world Comparison**: Compare simulation results with known physical behaviors. How would you calibrate simulation parameters to match reality?

3. **Performance vs. Accuracy**: Analyze the trade-offs between simulation accuracy and computational performance. When would you prioritize one over the other?

4. **Complex Contact Scenarios**: Design a simulation scenario involving multiple simultaneous contacts. How would you configure the physics parameters for this case?

## Summary

Physics simulation in Gazebo forms the foundation for realistic humanoid robot testing. Understanding and properly configuring physical properties, collision detection, and contact response is essential for accurate simulation results.

## Self-Assessment

1. What are the key physics concepts important for humanoid robot simulation?
2. How do collision and visual models differ in Gazebo?
3. What factors affect simulation stability and performance?
4. What are unique challenges for simulating humanoid robots?