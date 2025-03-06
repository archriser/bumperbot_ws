import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class EnhancedSubscriber(Node):
    def __init__(self):
        super().__init__('enhanced_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        received_msg = msg.data
        if "Counter" in received_msg:
            # Extract the counter number and do some processing
            number = int(received_msg.split(':')[-1].strip())
            self.get_logger().info(f'Received a counter message: {number}, double it: {number * 2}')
        else:
            # Process a random string and make it uppercase
            processed_msg = received_msg.upper()
            self.get_logger().info(f'Received a random string: {processed_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
