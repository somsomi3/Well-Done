# ROS 2 노드: 로컬 경로(local_path)를 따라가는 제어 노드
# /odom_true, /turtlebot_status, /local_path를 받아 /cmd_vel로 속도 명령 퍼블리시

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus
from nav_msgs.msg import Odometry, Path
from squaternion import Quaternion
from math import cos, sin, atan2, sqrt
import numpy as np


class PathTracking(Node):
    def __init__(self):
        super().__init__("path_tracking")

        # Publisher & Subscriber 설정
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_odom = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, 10
        )
        self.sub_status = self.create_subscription(
            TurtlebotStatus, "/turtlebot_status", self.status_callback, 10
        )
        self.sub_path = self.create_subscription(
            Path, "/local_path", self.path_callback, 10
        )

        # 타이머 콜백 (제어 주기 10ms)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # 상태 플래그
        self.is_odom = False
        self.is_status = False
        self.is_path = False

        # 메시지 초기화
        self.odom_msg = Odometry()
        self.path_msg = Path()
        self.status_msg = TurtlebotStatus()

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

        # 제어 명령 메시지
        self.cmd_msg = Twist()

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.is_odom = True

        # Orientation → Euler 변환
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = Quaternion(q.w, q.x, q.y, q.z).to_euler()

    def status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

    def path_callback(self, msg):
        self.path_msg = msg
        self.is_path = True
        self.goal_reached = False  # 새 경로 수신 시 도달 여부 초기화

    def timer_callback(self):
        if not (self.is_odom and self.is_status and self.is_path):
            return

        if len(self.path_msg.poses) < 2:
            self.get_logger().warn("⚠️ 경로가 충분하지 않음")
            self.stop_robot()
            return

        # 현재 위치
        robot_x = self.odom_msg.pose.pose.position.x
        robot_y = self.odom_msg.pose.pose.position.y

        # goal 도달 여부 확인
        goal = self.path_msg.poses[-1].pose.position
        dist_to_goal = sqrt((goal.x - robot_x) ** 2 + (goal.y - robot_y) ** 2)
        if dist_to_goal < self.goal_reach_dist:
            if not self.goal_reached:
                self.get_logger().info("✅ 목표 지점에 도달했습니다.")
                self.goal_reached = True
                self.path_msg = Path()  # 경로 초기화
            self.stop_robot()
            return

        # 선속도 기준 전방 주시 거리 동적 조정
        linear_speed = self.status_msg.twist.linear.x
        self.lfd = max(self.min_lfd, min(self.max_lfd, linear_speed * self.lfd_gain))

        # 전방 주시 포인트 탐색
        self.is_look_forward_point = False
        min_diff = float("inf")

        for waypoint in self.path_msg.poses:
            wp_x = waypoint.pose.position.x
            wp_y = waypoint.pose.position.y
            dist = sqrt((wp_x - robot_x) ** 2 + (wp_y - robot_y) ** 2)

            if abs(dist - self.lfd) < min_diff:
                min_diff = abs(dist - self.lfd)
                self.forward_point = waypoint.pose.position
                self.is_look_forward_point = True

        if self.is_look_forward_point:
            # 전방 주시 포인트를 로봇 좌표계로 변환
            global_pt = [self.forward_point.x, self.forward_point.y, 1]
            T = np.array(
                [
                    [cos(self.robot_yaw), -sin(self.robot_yaw), robot_x],
                    [sin(self.robot_yaw), cos(self.robot_yaw), robot_y],
                    [0, 0, 1],
                ]
            )
            local_pt = np.linalg.inv(T) @ np.array(global_pt).reshape(3, 1)
            theta = -atan2(local_pt[1][0], local_pt[0][0])

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
