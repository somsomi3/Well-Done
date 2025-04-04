# ROS 2 노드: 로컬 경로(local_path)를 따라가는 제어 노드
# /odom_true, /local_path를 받아 /cmd_vel로 속도 명령 퍼블리시

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
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

        # 타이머 콜백 (제어 주기 10ms)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # 상태 플래그
        self.is_odom = False
        self.is_path = False
        self.is_scan = False
        self.recovery_sent = False
        self.prev_position = None
        self.stuck_start_time = None
        self.stuck_timeout = 30.0
        self.is_blocked = False
        self.goal_reach_time = None  # 도달 시간 저장 변수

        # 메시지 초기화
        self.odom_msg = Odometry()
        self.path_msg = Path()
        self.forward_min_dist = float("inf")

        # 현재 로봇 방향(yaw)
        self.robot_yaw = 0.0

        # 전방 주시 거리 설정
        self.lfd = 0.5  # Look-Forward Distance (고정값)
        self.min_lfd = 0.1
        self.max_lfd = 2.0
        self.lfd_gain = 1.0

        # goal 도달 판별 기준
        self.goal_reach_dist = 0.3
        self.goal_reached = False

        # 장애물 블로킹 지속 시간 측정
        self.blocked_start_time = None
        self.blocked_timeout = 5.0

        # 제어 명령 메시지
        self.cmd_msg = Twist()

        print_log(
            "info",
            self.get_logger(),
            "🚀 PathTracking node started",
            file_tag=self.file_tag,
        )

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.is_odom = True

        # Orientation → Euler 변환
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = Quaternion(q.w, q.x, q.y, q.z).to_euler()

    def path_callback(self, msg):
        if self.goal_reached and self.is_same_path(msg):
            print_log(
                "info",
                self.get_logger(),
                "🚫 도달 상태 + 동일한 경로 → 무시",
                file_tag=self.file_tag,
            )

            return
        
        if not self.is_same_path(msg):
            self.goal_reached = False
            self.goal_reach_time = None
            
        self.path_msg = msg
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
        if not (self.is_odom and self.is_scan and self.is_path):
            return

        now = self.get_clock().now().nanoseconds / 1e9  # 현재 시간 (초)
        robot_x = self.odom_msg.pose.pose.position.x
        robot_y = self.odom_msg.pose.pose.position.y
        current_pos = (robot_x, robot_y)

        # 정체 여부 갱신
        self.check_stuck_status(current_pos, now)

        # 장애물 회피 (정체 or 센서 거리 기준)
        if self.forward_min_dist < 0.2 or self.is_blocked:
            if self.handle_obstacle_avoidance(now):
                return
        else:
            # ✅ 장애물 없을 경우 회피 관련 상태 초기화
            self.blocked_start_time = None
            self.recovery_sent = False
            self.is_blocked = False
            self.recovery_direction = 0

        # 경로 유효성 확인 및 goal 도달 여부 체크
        if self.check_path_validity_and_goal(robot_x, robot_y):
            return

        # 경로 따라 전방 주시 포인트 기준 제어 명령 계산
        if self.calculate_cmd_vel(robot_x, robot_y):
            self.cmd_pub.publish(self.cmd_msg)
        else:
            self.stop_robot()

    def check_stuck_status(self, current_pos, now):
        # 현재 로봇이 움직이려는 의지가 있는지 판단 (선속도)
        linear_cmd = self.cmd_msg.linear.x

        if self.prev_position is None:
            self.prev_position = current_pos
            self.stuck_start_time = now
            return

        dx = current_pos[0] - self.prev_position[0]
        dy = current_pos[1] - self.prev_position[1]
        moved = sqrt(dx**2 + dy**2)

        # 선속도가 거의 0일 때는 "정체 판단을 아예 건너뜀"
        if abs(linear_cmd) < 0.01:
            self.stuck_start_time = None
            self.is_blocked = False
            self.prev_position = current_pos
            return

        if moved < 0.05:
            if self.stuck_start_time is None:
                self.stuck_start_time = now
            elif now - self.stuck_start_time > self.stuck_timeout:
                print_log(
                    "warn",
                    self.get_logger(),
                    "⛔️ 로봇이 움직이려 했지만 움직이지 않음 → 회피 시작",
                    file_tag=self.file_tag,
                )

                self.is_blocked = True
        else:
            self.stuck_start_time = None
            self.is_blocked = False

        self.prev_position = current_pos

    def handle_obstacle_avoidance(self, now):
        if self.blocked_start_time is None:
            self.blocked_start_time = now
            self.recovery_direction = random.choice([-1, 1])
            print_log(
                "warn",
                self.get_logger(),
                "🛑 장애물 감지됨 - 회피 시작",
                file_tag=self.file_tag,
            )

        blocked_duration = now - self.blocked_start_time

        if blocked_duration < 2.0:
            print_log(
                "info",
                self.get_logger(),
                "⬅️ 장애물 회피 1단계: 후진 중",
                file_tag=self.file_tag,
            )
            self.cmd_msg.linear.x = -0.1
            self.cmd_msg.angular.z = 0.0
        elif blocked_duration < 4.0:
            print_log(
                "info",
                self.get_logger(),
                "🔁 장애물 회피 2단계: 회전 중",
                file_tag=self.file_tag,
            )
            self.cmd_msg.linear.x = 0.0
            self.cmd_msg.angular.z = 0.3 * self.recovery_direction
        else:
            if not self.recovery_sent:
                print_log(
                    "warn",
                    self.get_logger(),
                    "❌ 회피 실패 - goal_failed 퍼블리시",
                    file_tag=self.file_tag,
                )
                self.fail_pub.publish(
                    make_status_msg(
                        self, "avoidance_failed", True, self.get_clock().now().to_msg()
                    )
                )
                self.recovery_sent = True
            self.stop_robot()
            return True

        self.cmd_pub.publish(self.cmd_msg)
        return True

    def check_path_validity_and_goal(self, robot_x, robot_y):
        if len(self.path_msg.poses) < 1:
            if not self.goal_reached:
                print_log(
                    "warn",
                    self.get_logger(),
                    "❌ 경로 없음 + 도달 상태도 아님 → goal_failed",
                    file_tag=self.file_tag,
                )
                self.fail_pub.publish(
                    make_status_msg(
                        self, "path_empty", True, self.get_clock().now().to_msg()
                    )
                )
            self.stop_robot()
            return True

        goal = self.path_msg.poses[-1].pose.position
        dist_to_goal = sqrt((goal.x - robot_x) ** 2 + (goal.y - robot_y) ** 2)

        if not self.goal_reached and dist_to_goal < self.goal_reach_dist:
            self.goal_reached = True
            if self.goal_reach_time is None:
                self.goal_reach_time = self.get_clock().now().to_msg()
            self.path_msg = Path()

            print_log(
                "info",
                self.get_logger(),
                f"✅ 목표 지점에 도달했습니다. [stamp={self.goal_reach_time.sec}.{str(self.goal_reach_time.nanosec).zfill(9)}]",
                file_tag=self.file_tag,
            )
            msg = make_status_msg(self, "goal_reached", True, self.goal_reach_time)
            self.goal_reached_pub.publish(msg)
            self.stop_robot()
            return True

        return False

    def calculate_cmd_vel(self, robot_x, robot_y):
        self.is_look_forward_point = False
        min_dist = float("inf")

        for waypoint in self.path_msg.poses:
            pt = waypoint.pose.position
            dist = sqrt((pt.x - robot_x) ** 2 + (pt.y - robot_y) ** 2)
            if abs(dist - self.lfd) < min_dist:
                min_dist = abs(dist - self.lfd)
                self.forward_point = pt
                self.is_look_forward_point = True

        if not self.is_look_forward_point:
            print_log(
                "warn",
                self.get_logger(),
                "⚠️ 전방 주시 포인트를 찾을 수 없음",
                file_tag=self.file_tag,
            )

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

        print_log(
            "info",
            self.get_logger(),
            f"[TRACKING] Robot: ({robot_x:.2f}, {robot_y:.2f}) | "
            f"Yaw: {np.degrees(self.robot_yaw):.2f}° | "
            f"ForwardPt: ({self.forward_point.x:.2f}, {self.forward_point.y:.2f}) | "
            f"Local: ({local_pt[0][0]:.2f}, {local_pt[1][0]:.2f}) | "
            f"Theta: {np.degrees(theta):.2f}°",
            file_tag=self.file_tag,
        )

        vel = max(0.0, 1.0 * cos(theta))
        omega = max(-1.0, min(1.0, 1.5 * theta))

        self.cmd_msg.linear.x = float(vel)
        self.cmd_msg.angular.z = float(omega)
        return True

    def is_same_path(self, new_path):
        if len(new_path.poses) != len(self.path_msg.poses):
            return False
        for p1, p2 in zip(new_path.poses, self.path_msg.poses):
            dx = p1.pose.position.x - p2.pose.position.x
            dy = p1.pose.position.y - p2.pose.position.y
            if dx * dx + dy * dy > 0.01:  # 오차 1cm
                return False
        return True

    def stop_robot(self):
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        self.cmd_pub.publish(self.cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
