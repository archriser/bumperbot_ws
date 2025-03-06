import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class EnhancedPublisher(Node):
    def __init__(self):
        super().__init__('enhanced_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publishing every 1 second

    def timer_callback(self):
        # Randomly choose between a number and a message
        if random.choice([True, False]):
            msg = String()
            msg.data = f"Hello, ROS 2! Counter: {self.counter}"
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
        else:
            msg = String()
            msg.data = f"Random string: {random.choice(['apple', 'banana', 'cherry', 'date'])}"
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
