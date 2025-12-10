---
title: Lesson 1 - Introduction to ROS 2
sidebar_position: 2
description: Understanding the middleware architecture of ROS 2 for robot control
learning_objectives:
  - Explain the concept of middleware in robotics
  - Describe the architecture of ROS 2
  - Identify the key components of a ROS 2 system
  - Understand the differences between ROS 1 and ROS 2
duration: 120
---

# Lesson 1 - Introduction to ROS 2

## Learning Objectives

After completing this lesson, you will be able to:
- Explain the concept of middleware in robotics
- Describe the architecture of ROS 2
- Identify the key components of a ROS 2 system
- Understand the differences between ROS 1 and ROS 2

## Introduction

The Robot Operating System (ROS) is not an operating system but rather a middleware framework that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS 2 is the next generation of ROS, designed to address requirements for commercial robotics applications including real-time support, security, and deployment on resource-constrained systems.

## The Need for Middleware in Robotics

Robotics systems typically involve multiple components working together:
- Sensors (cameras, LIDAR, IMUs, etc.)
- Actuators (motors, grippers, etc.)
- Processors running various algorithms (perception, planning, control)
- Communication systems

Without middleware like ROS, developers would need to write custom communication protocols between all these components, leading to complex, hard-to-maintain code.

## ROS 2 Architecture

ROS 2 is built on top of DDS (Data Distribution Service), a communications middleware that provides a publish-subscribe pattern for data sharing. The key architectural components are:

Below is a textual representation of the ROS 2 architecture:

```text
# ROS 2 Architecture Diagram

## Components Overview:
- Nodes: Computational processes (circles)
- Topics: Communication channels (rectangles)
- Message Flow: Arrowed lines showing data flow

## Architecture:
[Node A] ---------> [Topic 1] ---------> [Node B]
                    (messages)

[Node C] ---------> [Topic 2] ---------> [Node D]
    |                                    |
    |<---------- Service Call ----------- |
(Service: Request/Response Pattern)

## Key Elements:
1. Nodes communicate via Topics using Publish/Subscribe
2. Nodes communicate via Services using Request/Response
3. Communication is managed via DDS (Data Distribution Service)
```

*Figure 1: ROS 2 Architecture showing nodes, topics, and the DDS communication layer.*

### Nodes
Nodes are the fundamental unit of execution in ROS 2. They are processes that perform computation and communicate with other nodes through messages.

### Topics
Topics are named buses over which nodes exchange messages. A publisher node sends messages to a topic, and subscriber nodes receive messages from a topic.

### Services
Services provide a request-response communication pattern. A client sends a request to a service and waits for a response.

### Actions
Actions provide a more advanced request-response pattern that can take a long time to complete and provides feedback during execution.

## Key Features of ROS 2

- **Real-time support**: For applications requiring deterministic behavior
- **Security**: Built-in authentication, encryption, and access control
- **Improved communication**: Based on DDS for reliable message delivery
- **Cross-platform**: Runs on Linux, Windows, and macOS
- **Container-friendly**: Designed to work well with containers like Docker

## Hands-On Exercise

Create a simple ROS 2 workspace and run the basic tutorials:

1. Create a new workspace directory
2. Source your ROS 2 installation
3. Create a package for your examples:
   ```bash
   ros2 pkg create --build-type ament_python my_robot_tutorial
   ```
4. Add the publisher code to `my_robot_tutorial/my_robot_tutorial/publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

5. Add the subscriber code to `my_robot_tutorial/my_robot_tutorial/subscriber_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

6. Run the publisher and subscriber in separate terminals:
   ```bash
   ros2 run my_robot_tutorial publisher_member_function
   ros2 run my_robot_tutorial subscriber_member_function
   ```

7. Observe the communication between nodes, noting how messages are published and received.

## Summary

ROS 2 provides the foundational middleware that allows different components of a robot system to communicate effectively. Understanding its architecture is crucial for building complex robotic systems.

## Exercises

1. **Conceptual Understanding**: Explain the difference between nodes, topics, and services in your own words. When would you use each?

2. **Practical Exercise**: Create a simple ROS 2 package with a publisher that sends messages at a different frequency (e.g., every 2 seconds instead of every 0.5 seconds). How does changing the frequency affect the communication?

3. **Research Task**: Investigate other middleware solutions used in robotics (e.g., ROS 1, YARP, OROCOS) and compare their features with ROS 2.

4. **Challenge Problem**: Design a simple robot system with 3 nodes (sensor, controller, actuator) and specify how they would communicate using ROS 2 concepts. What topics, services, or actions would you use?

## Self-Assessment

1. What is the role of middleware in robotics?
2. What is the difference between topics and services?
3. Why was ROS 2 developed as an evolution of ROS 1?
4. What is DDS and how does it relate to ROS 2?