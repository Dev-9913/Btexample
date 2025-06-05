# depth_publisher/publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')
        self.publisher = self.create_publisher(Float32, '/depth_data', 10)
        self.timer = self.create_timer(1.0, self.publish_depth)  # Publish at 1 Hz
        self.get_logger().info("DepthPublisher node started, publishing to /depth_data")

    def publish_depth(self):
        depth = random.uniform(1.0, 6.0)
        msg = Float32()
        msg.data = depth
        self.publisher.publish(msg)
        self.get_logger().info(f'Published depth: {depth:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = DepthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
