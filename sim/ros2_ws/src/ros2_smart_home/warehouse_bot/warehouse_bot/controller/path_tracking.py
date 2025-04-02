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


class PathTracking(Node):
    def __init__(self):
        super().__init__("path_tracking")

        # Publisher & Subscriber 설정
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.fail_pub = self.create_publisher(Bool, "/goal_failed", 1)
        self.goal_reached_pub = self.create_publisher(Bool, "/goal_reached", 1)

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

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.is_odom = True

        # Orientation → Euler 변환
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = Quaternion(q.w, q.x, q.y, q.z).to_euler()

    def path_callback(self, msg):
        self.path_msg = msg
        self.is_path = True
        self.goal_reached = False  # 새 경로 수신 시 도달 여부 초기화

    def scan_callback(self, msg):  # 🔧 전방 장애물 거리 계산
        num_ranges = len(msg.ranges)
        mid = num_ranges // 2
        half_fov = 10  # ±10도 (총 20도)
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

        if self.forward_min_dist < 0.3:
            if self.blocked_start_time is None:
                self.blocked_start_time = now
                self.get_logger().warn("🛑 장애물 감지됨 - 회피 시작")

            blocked_duration = now - self.blocked_start_time

            # 일정 시간 동안 장애물이 제거되지 않음 → goal_failed 퍼블리시
            if blocked_duration > self.blocked_timeout:
                if not self.recovery_sent:
                    self.get_logger().warn(
                        "❌ 장애물로 인한 경로 실패 - goal_failed 퍼블리시"
                    )
                    fail_msg = Bool()
                    fail_msg.data = True
                    self.fail_pub.publish(fail_msg)
                    self.recovery_sent = True

            # ✅ 회피 동작: 천천히 제자리에서 회전 시도
            if not self.recovery_sent:
                self.get_logger().info("🔁 장애물 회피 시도 중 (회전)")
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.3  # 좌회전
                self.cmd_pub.publish(self.cmd_msg)
            else:
                self.get_logger().info("🔁 회피 실패, 다음 goal을 기다리는 중...")
                self.stop_robot()
            return

        # 장애물이 사라짐 → 정상 상태로 복구
        self.blocked_start_time = None
        self.recovery_sent = False

        # 현재 위치
        robot_x = self.odom_msg.pose.pose.position.x
        robot_y = self.odom_msg.pose.pose.position.y

        # 경로가 없을 경우
        if len(self.path_msg.poses) < 1:
            if self.goal_reached:
                self.get_logger().info("ℹ️ 도달 후 경로 비워짐 → 무시")
            else:
                self.get_logger().warn("❌ 경로 없음 + 도달 상태도 아님 → goal_failed")
                self.fail_pub.publish(Bool(data=True))

            self.stop_robot()
            return

        # 경로가 있을 경우 → goal 도달 여부 확인
        goal = self.path_msg.poses[-1].pose.position
        dist_to_goal = sqrt((goal.x - robot_x) ** 2 + (goal.y - robot_y) ** 2)
        if not self.goal_reached and dist_to_goal < self.goal_reach_dist:
            self.get_logger().info("✅ 목표 지점에 도달했습니다.")
            self.goal_reached = True
            self.path_msg = Path()  # 경로 초기화
            self.goal_reached_pub.publish(Bool(data=True))
            self.stop_robot()
            return

        # 선속도 기준 전방 주시 거리 동적 조정
        # linear_speed = self.status_msg.twist.linear.x
        # self.lfd = max(self.min_lfd, min(self.max_lfd, linear_speed * self.lfd_gain))

        # 전방 주시 포인트 탐색
        self.is_look_forward_point = False
        min_dist = float("inf")

        for waypoint in self.path_msg.poses:
            self.current_point = waypoint.pose.position
            dist = sqrt(
                pow(self.current_point.x - robot_x, 2)
                + pow(self.current_point.y - robot_y, 2)
            )
            if abs(dist - self.lfd) < min_dist:
                min_dist = abs(dist - self.lfd)
                self.forward_point = self.current_point
                self.is_look_forward_point = True

        if self.is_look_forward_point:
            # 전방 주시 포인트를 로봇 좌표계로 변환
            global_pt = [self.forward_point.x, self.forward_point.y, 1]
            corrected_yaw = (
                self.robot_yaw + np.pi / 2
            )  # 왜인지 yaw를 90도 보정 해줘야 함?
            T = np.array(
                [
                    [cos(corrected_yaw), -sin(corrected_yaw), robot_x],
                    [sin(corrected_yaw), cos(corrected_yaw), robot_y],
                    [0, 0, 1],
                ]
            )
            local_pt = np.linalg.inv(T) @ np.array(global_pt).reshape(3, 1)
            theta = -atan2(local_pt[1][0], local_pt[0][0])

            # 🔍 디버깅 로그 출력
            self.get_logger().info(
                f"[TRACKING] Robot: ({robot_x:.2f}, {robot_y:.2f}) | "
                f"Yaw: {np.degrees(self.robot_yaw):.2f}° | "
                f"ForwardPt: ({self.forward_point.x:.2f}, {self.forward_point.y:.2f}) | "
                f"Local: ({local_pt[0][0]:.2f}, {local_pt[1][0]:.2f}) | "
                f"Theta: {np.degrees(theta):.2f}°"
            )

            # 선속도, 각속도 계산
            vel = max(0.0, 1.0 * cos(theta))
            omega = max(-1.0, min(1.0, 1.5 * theta))  # 각속도 제한

            self.cmd_msg.linear.x = float(vel)
            self.cmd_msg.angular.z = float(omega)

        else:
            self.get_logger().warn("⚠️ 전방 주시 포인트를 찾을 수 없음")
            self.stop_robot()
            return

        self.cmd_pub.publish(self.cmd_msg)

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
