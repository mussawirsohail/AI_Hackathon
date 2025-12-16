---
sidebar_position: 1
---

# Lesson 1: Introduction to ROS 2

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Unlike traditional operating systems, ROS 2 is not an actual OS but rather a middleware that provides services designed for a heterogeneous computer cluster, including hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

## Key Concepts in ROS 2

### Nodes
A node is an executable that uses the ROS 2 client library. Nodes offer a way to distribute computation across multiple processes or machines. In ROS 2, nodes are designed to be as lightweight as possible. A node is created using one of the client libraries (rclcpp for C++ or rclpy for Python).

### Topics and Messages
Topics are named buses over which nodes exchange messages. Each topic has a specific message type that defines the structure of the data being passed. Nodes can publish messages to a topic or subscribe to messages from a topic.

### Services
Services provide a request/reply mechanism similar to the client/server model. A service request is sent by a client node and processed by a server node, which returns a response.

## Why ROS 2?

ROS 2 addresses several limitations in the original ROS:

- **Real-Time Support**: Improved support for real-time systems
- **Multi-Robot Systems**: Better support for multiple robots
- **Production Deployment**: More robust for deployment in production environments
- **Security**: Built-in security features
- **Quality of Service**: Configurable network behavior for different types of data

## Getting Started with ROS 2

Before working with ROS 2, you'll need to install it on your system. The current recommended version is ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) release.

```bash
# Example: Creating a basic ROS 2 node in Python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

In the next lesson, we'll explore ROS 2 Nodes, Topics, and Services in greater detail and learn how to implement them in your robotic applications.