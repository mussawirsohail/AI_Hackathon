---
title: Lesson 4 - Nav2 for Bipedal Humanoid Navigation
sidebar_position: 5
description: Path planning and navigation for bipedal humanoid robots using Nav2
learning_objectives:
  - Configure Nav2 for humanoid robot navigation
  - Implement specialized planners for bipedal locomotion
  - Develop human-aware navigation behaviors
  - Optimize navigation for complex indoor environments
duration: 200
---

# Lesson 4 - Nav2 for Bipedal Humanoid Navigation

## Learning Objectives

After completing this lesson, you will be able to:
- Configure Nav2 for humanoid robot navigation
- Implement specialized planners for bipedal locomotion
- Develop human-aware navigation behaviors
- Optimize navigation for complex indoor environments

## Introduction

Navigation 2 (Nav2) is the navigation stack for ROS 2 that provides path planning, obstacle avoidance, and navigation execution capabilities for mobile robots. For bipedal humanoid robots, Nav2 requires special configuration and customization to account for the unique challenges of legged locomotion, including balance requirements, step planning, and social navigation norms. This lesson covers adapting Nav2 for humanoid robots with a focus on stable and socially appropriate movement.

## Nav2 Architecture Overview

### Core Components

Nav2 consists of several key components that work together:

1. **Navigation System**: The top-level system managing navigation lifecycle
2. **Map Server**: Provides static and costmap representations
3. **Local Planner**: Creates short-term trajectories to follow global path
4. **Global Planner**: Computes optimal path from start to goal
5. **Controller**: Converts trajectory into robot commands
6. **Recovery Behaviors**: Actions when robot gets stuck or fails

### Behavior Trees

Nav2 uses behavior trees for navigation decision-making:

- **Action Nodes**: Execute specific navigation tasks
- **Condition Nodes**: Evaluate navigation system state
- **Decorator Nodes**: Modify behavior of other nodes
- **Control Nodes**: Sequence execution of child nodes

## Nav2 Configuration for Humanoids

### Specialized Parameters

Humanoid robots require specific Nav2 parameter adjustments:

#### Footstep Planning
- **Step Size Limits**: Maximum step length and height
- **Balance Constraints**: Maintaining center of mass within support polygon
- **Foot Placement**: Precise foot placement for stable locomotion

#### Kinematic Constraints
- **Turning Radius**: Limited by leg configuration
- **Step Height**: Obstacle clearance capabilities
- **Speed Limits**: Safety and stability considerations

### Example Configuration

```yaml
# humanoid_nav2_config.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.5
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_have_feedback_condition_bt_node
    - nav2_have_global_plan_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transformer_bt_node
    - nav2_get_path_through_poses_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
```

## Global Path Planning for Bipedal Robots

### A* and Dijkstra Algorithms

Standard path planning algorithms adapted for humanoid constraints:

- **Grid-based Planning**: Discretized environment representation
- **Any-angle Path Planning**: Allowing paths that don't follow grid lines
- **Visibility Graphs**: Path planning in continuous space

### Humanoid-Specific Considerations

- **Step Constraints**: Path must consider maximum step length
- **Obstacle Clearance**: Minimum height for step-over ability
- **Balance Zones**: Areas where robot can maintain stable stance

### Custom Global Planners

Developing planners specialized for bipedal locomotion:

1. **Footstep Planner**: Planning discrete foot placements
2. **Balance-Aware Planner**: Incorporating balance constraints
3. **Gait-Adapted Planner**: Adjusting path based on gait pattern

## Local Path Planning and Obstacle Avoidance

### Dynamic Window Approach (DWA)

Adapting DWA for bipedal robots:

- **Feasible Velocity Space**: Limited by balance and step constraints
- **Collision Prediction**: Anticipating foot placement in future steps
- **Stability Regions**: Maintaining center of mass within support polygon

### Trajectory Rollout

Generating and evaluating possible trajectories:

- **Step Sequence Prediction**: Forecasting next several steps
- **Balance Evaluation**: Ensuring stability throughout trajectory
- **Obstacle Avoidance**: Adjusting steps to avoid collisions

### Example Local Planner Configuration

```yaml
# Local costmap configuration for humanoid
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 10.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      transform_tolerance: 0.5
      footprint: "[[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]"
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.6
        cost_scaling_factor: 3.0
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /laser_scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
```

## Socially-Aware Navigation

### Human-Aware Path Planning

Incorporating social norms and human behavior:

- **Personal Space**: Respecting human personal space zones
- **Social Territories**: Understanding social territories and proxemics
- **Gaze Direction**: Avoiding walking through people's visual fields

### Collision Avoidance with Humans

- **Predictive Avoidance**: Predicting human movement patterns
- **Comfortable Distances**: Maintaining socially appropriate distances
- **Right-of-Way**: Following social navigation conventions

### Specialized Behaviors

- **Group Navigation**: Navigating around groups of people
- **Queue Behavior**: Waiting appropriately in lines
- **Doorway Navigation**: Managing doorway interactions

## Recovery Behaviors for Humanoids

### Stuck Recovery

Specialized recovery behaviors for bipedal robots:

- **Step-Based Recovery**: Using precise foot placement for recovery
- **Balance Recovery**: Regaining balance before continuing navigation
- **Alternative Pathfinding**: Finding new paths when stuck

### Fall Recovery

- **Pre-fall Detection**: Identifying when robot might fall
- **Protected Fall**: Minimizing damage during falls
- **Stand-up Capabilities**: Self-recovery after falls (if possible)

## Footstep Planning Integration

### Discrete Planning

- **Footstep Graph**: Planning on discrete footstep graphs
- **Capturability**: Ensuring each step maintains capturability
- **ZMP (Zero Moment Point)**: Maintaining dynamic balance

### Continuous Planning

- **Spline-based Footstep Planning**: Smooth transitions between steps
- **Dynamically Stable Steps**: Ensuring dynamic stability in paths
- **Time-optimal Planning**: Minimizing navigation time while maintaining stability

## Humanoid-Specific Navigation Challenges

### Balance and Stability

- **Dynamic Balance**: Maintaining balance during movement
- **Center of Mass Control**: Managing center of mass position
- **Support Polygon**: Maintaining feet in appropriate positions

### Environmental Constraints

- **Step-Over Obstacles**: Navigating small obstacles that can be stepped over
- **Surface Variations**: Adapting to uneven surfaces
- **Stair Navigation**: Climbing stairs and navigating level changes

## Integration with Hardware

### Sensor Integration

- **IMU Integration**: Using IMU data for balance feedback
- **Force/Torque Sensors**: Monitoring contact forces for balance
- **Stereo Cameras**: Using visual perception for terrain analysis

### Controller Integration

- **Walking Controller**: Interface with walking pattern generators
- **Balance Controller**: Coordinating with balance control systems
- **Step Controller**: Precise foot placement control

## Performance Evaluation

### Metrics for Humanoid Navigation

- **Navigation Success Rate**: Percentage of successful navigations
- **Path Efficiency**: Ratio of actual path length to optimal
- **Social Compliance**: Adherence to social navigation norms
- **Balance Maintenance**: Robot's ability to maintain balance

### Testing Scenarios

- **Cluttered Environments**: Navigation in complex indoor spaces
- **Human Interaction**: Navigation around moving humans
- **Stair Navigation**: Climbing stairs and level changes
- **Narrow Spaces**: Navigation through doorways and corridors

## Hands-On Exercise

1. **Nav2 Installation**: Install and configure Nav2 for your humanoid robot:
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   # Create a custom configuration file for humanoid
   ```

2. **Parameter Configuration**: Configure Nav2 parameters for humanoid constraints:
   - Set appropriate footprint for bipedal robot
   - Adjust costmap inflation for social distance
   - Configure appropriate step limits

3. **Simulation Test**: Test navigation in a Gazebo simulation:
   - Launch Nav2 with your humanoid robot
   - Send navigation goals using RViz
   - Observe path planning and execution

4. **Social Navigation**: Implement social navigation behaviors:
   - Add humans to the simulation environment
   - Test navigation around static and moving humans
   - Evaluate compliance with social norms

5. **Performance Analysis**: Analyze navigation performance:
   - Measure success rate in different environments
   - Evaluate path efficiency
   - Assess balance maintenance during navigation

## Exercises

1. **Parameter Tuning**: Tune Nav2 parameters for optimal humanoid navigation performance in your environment. What parameters have the most significant impact?

2. **Social Behavior**: Design and implement a social navigation behavior for a specific scenario (e.g., passing a person coming in the opposite direction). How would you implement this?

3. **Footstep Planning**: How would you integrate discrete footstep planning with the continuous path planning of Nav2?

4. **Recovery Strategy**: Design a recovery behavior specifically for bipedal humanoid robots that have fallen or lost balance.

## Summary

Nav2 can be adapted for bipedal humanoid navigation by configuring parameters for balance constraints, implementing socially-aware navigation behaviors, and integrating with specialized footstep planning systems. Understanding these adaptations is crucial for effective humanoid robot navigation in human environments.

## Self-Assessment

1. What are the key differences between wheeled robot navigation and humanoid navigation?
2. How do you configure Nav2 for bipedal locomotion constraints?
3. What social navigation behaviors are important for humanoid robots?
4. How do you manage balance and stability during navigation with Nav2?