import rclpy
from rclpy.node import Node

from enum import Enum
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
from math import cos, sin, atan2, sqrt
import numpy as np
import random
from ssafy_msgs.msg import StatusStamped

from warehouse_bot.utils.logger_utils import print_log
from warehouse_bot.utils.msg_utils import make_status_msg


class PathTrackingState(Enum):
    IDLE = 0
    FOLLOW_PATH = 1
    AVOIDANCE_STAGE1 = 2
    AVOIDANCE_STAGE2_BACKWARD = 3
    AVOIDANCE_STAGE2_ROTATE = 4
    GOAL_REACHED = 5


class PathTracking(Node):
    def __init__(self):
        super().__init__("path_tracking")
        self.file_tag = "path_tracking"

        # Publisher & Subscriber 설정
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.fail_pub = self.create_publisher(StatusStamped, "/goal_failed", 1)
        self.goal_reached_pub = self.create_publisher(StatusStamped, "/goal_reached", 1)

        self.sub_odom = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, 10
        )
        self.sub_path = self.create_subscription(
            Path, "/local_path", self.path_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.sub_mode = self.create_subscription(
            String, "/current_mode", self.mode_callback, 10
        )
        self.sub_stop = self.create_subscription(
            Bool, "/stop_all", self.stop_all_callback, 10
        )

        # 타이머 콜백 (제어 주기 10ms)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # 상태 관련 변수
        self.state = PathTrackingState.IDLE
        self.stopped = False
        self.is_active = True

        # 내부 상태 변수
        self.odom_msg = Odometry()
        self.path_msg = Path()
        self.forward_min_dist = float("inf")
        self.is_odom = False
        self.is_path = False
        self.is_scan = False
        self.goal_reached = False
        self.goal_reach_time = None
        self.robot_yaw = 0.0
        self.lfd = 0.5
        # self.min_lfd = 0.1
        # self.max_lfd = 2.0
        # self.lfd_gain = 1.0
        self.goal_reach_dist = 0.05

        self.recovery_direction = 1
        self.recovery_start_time = None

        print_log(
            "info",
            self.get_logger(),
            "✅ FSM 기반 PathTracking 시작",
            file_tag=self.file_tag,
        )

    def mode_callback(self, msg):
        self.is_active = msg.data == "PICK_AND_PLACE"

    def stop_all_callback(self, msg):
        self.stopped = msg.data
        if self.stopped:
            self.stop_robot()
            self.state = PathTrackingState.IDLE
            print_log(
                "warn",
                self.get_logger(),
                "🛑 시스템 정지 수신 → IDLE 전환",
                file_tag=self.file_tag,
            )

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.is_odom = True
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = Quaternion(q.w, q.x, q.y, q.z).to_euler()

    def path_callback(self, msg):
        if not self.is_same_path(msg):
            self.path_msg = msg
            self.goal_reached = False
            self.state = PathTrackingState.FOLLOW_PATH
        self.is_path = True

    def scan_callback(self, msg):  # 🔧 전방 장애물 거리 계산
        num_ranges = len(msg.ranges)
        mid = num_ranges // 2
        half_fov = 20  # ±10도 (총 20도)
        front_ranges = [
            r
            for r in msg.ranges[mid - half_fov : mid + half_fov]
            if msg.range_min < r < msg.range_max
        ]
        self.forward_min_dist = min(front_ranges) if front_ranges else float("inf")
        self.is_scan = True

    def timer_callback(self):
        if not (
            self.is_active
            and not self.stopped
            and self.is_odom
            and self.is_path
            and self.is_scan
        ):
            return

        if self.state == PathTrackingState.FOLLOW_PATH:
            self.run_follow_path()
        elif self.state == PathTrackingState.AVOIDANCE_STAGE1:
            self.run_avoidance_stage1()
        elif self.state == PathTrackingState.AVOIDANCE_STAGE2_BACKWARD:
            self.run_avoidance_stage2_backward()
        elif self.state == PathTrackingState.AVOIDANCE_STAGE2_ROTATE:
            self.run_avoidance_stage2_rotate()
        elif self.state == PathTrackingState.GOAL_REACHED:
            self.stop_robot()

    def run_follow_path(self):
        robot_x = self.odom_msg.pose.pose.position.x
        robot_y = self.odom_msg.pose.pose.position.y

        # 장애물 감지 시
        if self.forward_min_dist < 0.2:
            print_log(
                "warn",
                self.get_logger(),
                "⚠️ 장애물 감지 → 회피 전환",
                file_tag=self.file_tag,
            )
            self.state = PathTrackingState.AVOIDANCE_STAGE1
            self.recovery_start_time = self.get_clock().now().nanoseconds / 1e9
            self.recovery_direction = random.choice([-1, 1])
            return

        # 이미 도착한 것과 마찬가지
        if len(self.path_msg.poses) < 1:
            self.fail_pub.publish(
                make_status_msg(
                    self, "path_empty", True, self.get_clock().now().to_msg()
                )
            )
            self.state = PathTrackingState.IDLE
            return

        goal = self.path_msg.poses[-1].pose.position
        dist_to_goal = sqrt((goal.x - robot_x) ** 2 + (goal.y - robot_y) ** 2)
        if not self.goal_reached and dist_to_goal < self.goal_reach_dist:
            self.goal_reached = True
            self.goal_reach_time = self.get_clock().now().to_msg()
            self.goal_reached_pub.publish(
                make_status_msg(self, "goal_reached", True, self.goal_reach_time)
            )
            print_log(
                "info",
                self.get_logger(),
                "🎯 목표 도달 → GOAL_REACHED 상태 전환",
                file_tag=self.file_tag,
            )
            self.state = PathTrackingState.GOAL_REACHED
            return

        # 경로 따라 제어 명령 계산
        if self.calculate_cmd_vel(robot_x, robot_y):
            self.cmd_pub.publish(self.cmd_msg)

    def run_avoidance_stage1(self):
        now = self.get_clock().now().nanoseconds / 1e9
        print_log(
            "info", self.get_logger(), "🔙 회피 1단계: 후진 중", file_tag=self.file_tag
        )
        self.cmd_msg = Twist()
        self.cmd_msg.linear.x = -0.3
        self.cmd_pub.publish(self.cmd_msg)

        if now - self.recovery_start_time > 1.0:
            self.recovery_start_time = now
            self.state = PathTrackingState.AVOIDANCE_STAGE2_BACKWARD

    def run_avoidance_stage2_backward(self):
        now = self.get_clock().now().nanoseconds / 1e9
        print_log(
            "info", self.get_logger(), "↩️ 회피 2단계: 후진 중", file_tag=self.file_tag
        )
        self.cmd_msg = Twist()
        self.cmd_msg.linear.x = -0.3
        self.cmd_pub.publish(self.cmd_msg)

        if now - self.recovery_start_time > 1.0:
            self.recovery_start_time = now
            self.state = PathTrackingState.AVOIDANCE_STAGE2_ROTATE

    def run_avoidance_stage2_rotate(self):
        now = self.get_clock().now().nanoseconds / 1e9
        print_log(
            "info", self.get_logger(), "🔁 회피 2단계: 회전 중", file_tag=self.file_tag
        )
        self.cmd_msg = Twist()
        self.cmd_msg.angular.z = 0.3 * self.recovery_direction
        self.cmd_pub.publish(self.cmd_msg)

        if now - self.recovery_start_time > 1.0:
            self.recovery_start_time = now
            self.state = PathTrackingState.FOLLOW_PATH

    def calculate_cmd_vel(self, robot_x, robot_y):
        min_dist = float("inf")
        self.is_look_forward_point = False
        for waypoint in self.path_msg.poses:
            pt = waypoint.pose.position
            dist = sqrt((pt.x - robot_x) ** 2 + (pt.y - robot_y) ** 2)
            if abs(dist - self.lfd) < min_dist:
                min_dist = abs(dist - self.lfd)
                self.forward_point = pt
                self.is_look_forward_point = True

        if not self.is_look_forward_point:
            return False

        global_pt = [self.forward_point.x, self.forward_point.y, 1]
        corrected_yaw = self.robot_yaw + np.pi / 2
        T = np.array(
            [
                [cos(corrected_yaw), -sin(corrected_yaw), robot_x],
                [sin(corrected_yaw), cos(corrected_yaw), robot_y],
                [0, 0, 1],
            ]
        )
        local_pt = np.linalg.inv(T) @ np.array(global_pt).reshape(3, 1)
        theta = -atan2(local_pt[1][0], local_pt[0][0])

        vel = max(0.0, 1.0 * cos(theta))
        omega = max(-1.0, min(1.0, 1.5 * theta))

        goal = self.path_msg.poses[-1].pose.position
        dist_to_goal = sqrt((goal.x - robot_x) ** 2 + (goal.y - robot_y) ** 2)
        if dist_to_goal < 0.5:
            vel *= 0.3
        elif dist_to_goal < 1.0:
            vel *= 0.6

        self.cmd_msg = Twist()
        self.cmd_msg.linear.x = float(vel)
        self.cmd_msg.angular.z = float(omega)
        return True

    def is_same_path(self, new_path):
        if len(new_path.poses) != len(self.path_msg.poses):
            return False
        for p1, p2 in zip(new_path.poses, self.path_msg.poses):
            dx = p1.pose.position.x - p2.pose.position.x
            dy = p1.pose.position.y - p2.pose.position.y
            if dx * dx + dy * dy > 0.01:
                return False
        return True

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PathTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
