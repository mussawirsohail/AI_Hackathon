---
sidebar_position: 2
---

# Lesson 2: ROS 2 Nodes, Topics, and Services

## Understanding Nodes

In ROS 2, a node is the fundamental unit of computation. Each node is typically responsible for a specific task such as controlling a sensor, processing data, or controlling actuators. Nodes are designed to be as lightweight as possible and communicate with each other via messages.

### Creating a Node in Python

Here's how to create a basic ROS 2 node using the rclpy client library:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('MyRobotNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Message Passing

Topics enable asynchronous communication between nodes using a publish-subscribe model. This decouples nodes from each other, making the system more modular and flexible.

### Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
```

### Creating a Subscriber

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
```

## Services

Services provide synchronous request/response communication. A node offering a service waits for requests and sends back responses.

### Creating a Service Server

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Creating a Service Client

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Quality of Service (QoS)

ROS 2 provides Quality of Service profiles that allow you to configure how data is delivered between publishers and subscribers, which is important for real-time and safety-critical applications.

```python
from rclpy.qos import QoSProfile

# Configure QoS for real-time applications
qos_profile = QoSProfile(depth=10)
qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
qos_profile.durability = rclpy.qos.DurabilityPolicy.VOLATILE

# Create publisher with custom QoS
publisher = self.create_publisher(String, 'topic', qos_profile)
```

In the next lesson, we'll learn how to bridge Python agents to ROS controllers using rclpy.