# ROS 2 Service Server Example

This is a simple service server example that demonstrates the basic structure of a ROS 2 service server.

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


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Explanation:

1. Import the service definition (in this case AddTwoInts)
2. Create a class that inherits from Node
3. Initialize service server
4. Define callback function that processes requests and returns responses
5. Initialize and run the node in main function