import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from math import sqrt
from ssafy_msgs.msg import StatusStamped
from warehouse_bot.utils.logger_utils import print_log


class AStarLocalPath(Node):
    def __init__(self):
        super().__init__("a_star_local_path")

        self.file_tag = "a_star_local_path"

        # Publisher & Subscriber
        self.local_path_pub = self.create_publisher(Path, "/local_path", 10)
        self.sub_path = self.create_subscription(
            Path, "/global_path", self.path_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            StatusStamped, "/goal_reached", self.goal_callback, 10
        )
        self.goal_reached_pub = self.create_publisher(
            StatusStamped, "/goal_reached", 10
        )

        self.odom_msg = None
        self.global_path_msg = None
        self.is_odom = False
        self.is_path = False
        self.goal_reached = False
        self.global_path_received_time = None

        self.local_path_size = 30
        self.timer = self.create_timer(0.05, self.timer_callback)

        print_log(
            "info",
            self.get_logger(),
            "AStarLocalPath node initialized.",
            file_tag=self.file_tag,
        )

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.is_odom = True

    def path_callback(self, msg):
        self.global_path_msg = msg
        self.is_path = True
        self.goal_reached = False
        self.global_path_received_time = self.get_clock().now()
        print_log(
            "info",
            self.get_logger(),
            f"✅ Received global path with {len(msg.poses)} poses.",
            file_tag=self.file_tag,
        )

    def goal_callback(self, msg):
        self.goal_reached = msg.status
        if self.goal_reached:
            print_log(
                "info",
                self.get_logger(),
                "🛑 Goal reached. Local path generation halted.",
                file_tag=self.file_tag,
            )

    def timer_callback(self):
        if not (self.is_odom and self.is_path):
            return

        if self.goal_reached:
            return  # 도달했으면 지역 경로 생성 안 함

        if self.global_path_received_time:
            now = self.get_clock().now()
            elapsed = (
                now - self.global_path_received_time
            ).nanoseconds / 1e9  # 초 단위

            if elapsed > 30.0:
                print_log(
                    "warn",
                    self.get_logger(),
                    f"⏰ 동일 경로 시도 시간 초과 ({elapsed:.1f}초) → 새로운 전역 경로 필요",
                    file_tag=self.file_tag,
                )
                self.goal_reached = True  # 지역 경로 생성 멈추기

                # 메시지 생성 및 퍼블리시
                msg = StatusStamped()
                msg.stamp = now.to_msg()
                msg.tag = "timeout_reached"
                msg.status = True
                self.goal_reached_pub.publish(msg)
                return

        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y

        # 현재 위치와 가장 가까운 경로 포인트 탐색
        min_dist = float("inf")
        nearest_idx = -1
        for i, pose in enumerate(self.global_path_msg.poses):
            dx = x - pose.pose.position.x
            dy = y - pose.pose.position.y
            dist = sqrt(dx * dx + dy * dy)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        if nearest_idx == -1:
            print_log(
                "warn",
                self.get_logger(),
                "❌ No nearest point found on global path.",
                file_tag=self.file_tag,
            )
            return

        end_idx = min(
            nearest_idx + self.local_path_size, len(self.global_path_msg.poses)
        )
        local_path_msg = Path()
        local_path_msg.header.frame_id = "map"
        local_path_msg.header.stamp = self.get_clock().now().to_msg()
        local_path_msg.poses = self.global_path_msg.poses[nearest_idx:end_idx]

        self.local_path_pub.publish(local_path_msg)
        print_log(
            "info",
            self.get_logger(),
            f"📤 Published local path: {end_idx - nearest_idx} poses (from idx {nearest_idx})",
            file_tag=self.file_tag,
        )


def main(args=None):
    rclpy.init(args=args)
    node = AStarLocalPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
