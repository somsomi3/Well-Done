import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math


class TracePathNode(Node):
    def __init__(self):
        super().__init__("trace_path_node")

        # 경로 누적 메시지 초기화
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"  # 좌표계: map or odom

        # 경로 퍼블리셔
        self.path_pub = self.create_publisher(Path, "/trace_path", 10)

        # odom 구독 (필요시 /odom_true로 바꿔도 됨)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, 10
        )

        self.last_x = None
        self.last_y = None
        self.min_dist = 0.05

        self.get_logger().info(
            "TracePathNode started. Subscribing to /odom and publishing to /trace_path"
        )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 첫 위치는 무조건 저장
        if self.last_x is None or self.last_y is None:
            should_add = True
        else:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            should_add = dist >= self.min_dist

        if should_add:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose = msg.pose.pose

            self.path_msg.poses.append(pose)
            self.path_msg.header.stamp = pose.header.stamp
            self.path_pub.publish(self.path_msg)

            self.last_x = x
            self.last_y = y


def main(args=None):
    rclpy.init(args=args)
    node = TracePathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
