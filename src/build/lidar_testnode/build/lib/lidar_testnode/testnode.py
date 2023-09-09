#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("test_node")
        self.get_logger().info("Hello from ROS2")


def main(args=None):
    # Initialize communications
    rclpy.init(args=args)
    # Create node
    node = MyNode()
    # Continue to run the node
    rclpy.spin(node)
    # Shutdown communications
    rclpy.shutdown()

if __name__ == '__main__':
    main()
