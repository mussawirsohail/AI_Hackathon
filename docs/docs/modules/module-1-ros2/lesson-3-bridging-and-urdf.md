---
sidebar_position: 3
---

# Lesson 3: Bridging Python Agents to ROS Controllers and Understanding URDF

## Bridging Python Agents to ROS Controllers with rclpy

Python agents can interact with ROS controllers by implementing publishers, subscribers, services, and actions. Let's look at how to bridge AI agents with robotic systems.

### AI Agent Example: Basic Navigation Controller

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Robot state
        self.laser_data = None
        self.target_reached = False

    def scan_callback(self, msg):
        self.laser_data = np.array(msg.ranges)
        
    def control_loop(self):
        if self.laser_data is None:
            return
            
        # Simple AI logic: avoid obstacles and move forward
        min_distance = np.min(self.laser_data)
        
        cmd_vel = Twist()
        
        if min_distance < 1.0:  # Obstacle too close
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn
        else:
            cmd_vel.linear.x = 0.5  # Move forward
            cmd_vel.angular.z = 0.0
            
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Understanding URDF (Unified Robot Description Format)

URDF is an XML format for representing a robot model. It defines the physical and visual properties of a robot, including links, joints, and other elements.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joint connects two links -->
  <joint name="base_to_lidar" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
  
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### URDF for Humanoid Robots

For humanoid robots, URDF becomes more complex with multiple joints and links:

```xml
<?xml version="1.0"?>
<robot name="humanoid">
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.5"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <!-- Add more joints and links for full humanoid model -->
</robot>
```

## Working with URDF in Python

You can load and manipulate URDF models in Python using the `urdf_parser_py` library:

```python
from urdf_parser_py.urdf import URDF
import rospy

# Load URDF from file
robot = URDF.from_xml_file('path/to/robot.urdf')

# Access robot properties
print(f"Robot name: {robot.name}")
print(f"Links: {[link.name for link in robot.links]}")
print(f"Joints: {[joint.name for joint in robot.joints]}")

# Get transforms between links
# (requires robot state publisher and tf2)
```

## Summary

In this module, we've covered:

1. The fundamentals of ROS 2
2. How to create and work with nodes, topics, and services
3. How to bridge AI agents to ROS controllers
4. The structure and purpose of URDF files for humanoid robots

These concepts form the foundational nervous system for robotic applications, enabling communication and coordination between different components of a robotic system.

Continue to [Module 2: The Digital Twin (Gazebo & Unity)](../module-2-digital-twin/lesson-1-gazebo-simulation.md) to learn about physics simulation and environment building.