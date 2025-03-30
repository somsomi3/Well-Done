import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import time


class AutoMapper(Node):
    def __init__(self):
        super().__init__("auto_mapper")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.map_data = None
        self.prev_map = None
        self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.timer = self.create_timer(0.1, self.timer_callback)

        # 커버리지 바운더리
        self.min_x, self.max_x = float("inf"), float("-inf")
        self.min_y, self.max_y = float("inf"), float("-inf")

        # 주행 상태
        self.direction = 1  # 1이면 정방향, -1이면 역방향
        self.state = "FORWARD"
        self.step = 0

        self.forward_duration = 3.0
        self.turn_duration = 1.5
        self.last_action_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.get_logger().info("🚗 자동 매핑 시작!")

    def map_callback(self, msg):
        # 맵 업데이트
        new_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        if self.prev_map is not None:
            diff = np.abs(new_map - self.prev_map)
            change_rate = np.count_nonzero(diff) / diff.size

            coverage = (
                np.count_nonzero((0 <= new_map) & (new_map <= 100)) / new_map.size
            )
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if (
                now - self.last_update_time > 5.0
                and change_rate < 0.01
                and coverage > 0.90
            ):
                self.get_logger().info(
                    f"✅ 매핑 종료 - 변화율={change_rate:.4f}, 커버리지={coverage:.2%}"
                )
                self.stop_robot()
                self.destroy_node()
        self.prev_map = new_map
        self.map_data = new_map
        self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]

    def timer_callback(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.last_action_time

        msg = Twist()

        if self.state == "FORWARD":
            if elapsed < self.forward_duration:
                msg.linear.x = 0.2
            else:
                self.state = "TURN"
                self.last_action_time = now
        elif self.state == "TURN":
            if elapsed < self.turn_duration:
                msg.angular.z = 0.5 * self.direction
            else:
                self.direction *= -1
                self.state = "FORWARD"
                self.last_action_time = now
                self.step += 1

        self.pub.publish(msg)

    def stop_robot(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
