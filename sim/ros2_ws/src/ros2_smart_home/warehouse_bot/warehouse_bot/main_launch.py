# warehouse_bot/main_launch.py

import rclpy
from rclpy.node import Node


class MainLauncher(Node):
    def __init__(self):
        super().__init__("main_launcher")
        self.get_logger().info("🚀 warehouse_bot main_launch 실행 중...")


def main(args=None):
    rclpy.init(args=args)
    node = MainLauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
