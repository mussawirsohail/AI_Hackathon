---
title: Lesson 2 - ROS 2 Nodes, Topics, and Services
sidebar_position: 3
description: Understanding the communication patterns in ROS 2
learning_objectives:
  - Create and run ROS 2 nodes
  - Implement publisher-subscriber communication patterns
  - Create and use ROS 2 services
  - Debug communication between nodes
duration: 150
---

# Lesson 2 - ROS 2 Nodes, Topics, and Services

## Learning Objectives

After completing this lesson, you will be able to:
- Create and run ROS 2 nodes
- Implement publisher-subscriber communication patterns
- Create and use ROS 2 services
- Debug communication between nodes

## Introduction

In ROS 2, nodes, topics, and services form the core communication infrastructure. Nodes are computational processes that perform work, while topics and services define how nodes communicate with each other. Understanding these concepts is essential for building distributed robotic applications.

## Nodes

Nodes are the fundamental building blocks of a ROS 2 system. They're processes that perform computation. In a typical robot system, you might have nodes for:

- Sensor drivers
- Control algorithms
- Perception systems
- Planning algorithms
- User interfaces

Nodes are implemented as programs written in languages that have ROS 2 client libraries (C++, Python, etc.).

### Creating a Node

A minimal ROS 2 node typically:
1. Initializes the ROS 2 client library
2. Creates a node object
3. Creates publishers, subscribers, or services
4. Spins (processes callbacks) or runs a loop
5. Cleans up when finished

## Topics - Publisher-Subscriber Communication

Topics implement an asynchronous, many-to-many communication pattern. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic. Messages are distributed to all subscribers.

### Message Types

All messages on a topic must be of the same type. ROS comes with standard message types, and users can define custom message types. Common message types include:
- `std_msgs` - Basic types like integers, floats, strings
- `sensor_msgs` - Messages for sensors like cameras and LIDARs
- `geometry_msgs` - Messages for geometric data like positions and poses

## Services - Request-Response Communication

Services implement synchronous, one-to-one communication. A client sends a request and waits for a response from the server. This is useful for tasks that require a specific response, such as saving a map or setting robot parameters.

### Service Types

Like topics, services use predefined message types that define the request and response structure. The client sends a request of one type and receives a response of another type.

## Example: Simple Robot System

Consider a simple robot with:
- A sensor node that publishes laser scan data
- A navigation node that subscribes to laser scans and publishes velocity commands
- A control node that subscribes to velocity commands and drives the motors

The sensor node publishes to `/scan`, the navigation node subscribes to `/scan` and publishes to `/cmd_vel`, and the control node subscribes to `/cmd_vel`.

## Hands-On Exercise

Implement a simple publisher-subscriber system:

1. Create a package for your examples (if not already done):
   ```bash
   ros2 pkg create --build-type ament_python counter_tutorial
   ```

2. Add a publisher node to `counter_tutorial/counter_tutorial/counter_publisher.py` that publishes a counter message every second:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class CounterPublisher(Node):

    def __init__(self):
        super().__init__('counter_publisher')
        self.publisher_ = self.create_publisher(Int64, 'counter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = Int64()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing counter: %d' % msg.data)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    counter_publisher = CounterPublisher()

    rclpy.spin(counter_publisher)

    counter_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

3. Add a subscriber node to `counter_tutorial/counter_tutorial/counter_subscriber.py` that subscribes to the counter topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class CounterSubscriber(Node):

    def __init__(self):
        super().__init__('counter_subscriber')
        self.subscription = self.create_subscription(
            Int64,
            'counter',
            self.counter_callback,
            10)
        self.subscription  # prevent unused variable warning

    def counter_callback(self, msg):
        self.get_logger().info('Received counter: %d' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    counter_subscriber = CounterSubscriber()

    rclpy.spin(counter_subscriber)

    counter_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

4. Run both nodes in separate terminals:
   ```bash
   ros2 run counter_tutorial counter_publisher
   ros2 run counter_tutorial counter_subscriber
   ```

5. Use `ros2 topic echo` to observe messages on the command line:
   ```bash
   ros2 topic echo /counter std_msgs/msg/Int64
   ```

6. Observe the communication between nodes and how the counter value increments over time.

## Summary

Understanding nodes, topics, and services is crucial for designing robust ROS 2 systems. The publisher-subscriber pattern is ideal for sensor data and continuous control commands, while services are better for one-time requests and operations.

## Exercises

1. **Implementation Challenge**: Modify the counter publisher to publish a sequence of Fibonacci numbers instead of a simple counter. What challenges do you encounter when implementing this?

2. **Design Exercise**: Design a communication architecture for a mobile robot with the following capabilities: laser scanner, camera, wheel odometry, and velocity control. Specify the nodes, topics, and message types you would use.

3. **Quality of Service (QoS) Exploration**: Research and experiment with different QoS profiles in ROS 2 (reliability, durability, etc.). How do these settings affect communication between nodes?

4. **Debugging Task**: Create a simple publisher-subscriber pair with a mismatch in message types or topic names to simulate a common communication error. How would you diagnose and fix this issue?

## Self-Assessment

1. What is the difference between a node, topic, and service?
2. How does the publisher-subscriber pattern work?
3. When would you use a service instead of a topic?
4. What are message types and why are they important?