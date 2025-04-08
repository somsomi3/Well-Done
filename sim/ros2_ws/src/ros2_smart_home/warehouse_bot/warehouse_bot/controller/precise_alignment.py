import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from squaternion import Quaternion
from math import atan2, sqrt, pi
import numpy as np


class PreciseAlignment(Node):
    def __init__(self):
        super().__init__("precise_alignment")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.done_pub = self.create_publisher(Bool, "/alignment_done", 10)

        self.odom_sub = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, 10
        )
        self.target_sub = self.create_subscription(
            PoseStamped, "/target_pose", self.target_callback, 10
        )

        self.timer = self.create_timer(0.02, self.control_loop)

        self.odom = None
        self.target_pose = None
        self.aligning = False
        self.stage = 0  # 0: 회전, 1: 전진, 2: 방향 정렬

        self.pos_tolerance = 0.05
        self.yaw_tolerance = 0.052

    def odom_callback(self, msg):
        self.odom = msg

    def target_callback(self, msg):
        self.target_pose = msg
        self.aligning = True
        self.stage = 0
        self.get_logger().info(
            "📍 [START] 정밀 정렬 명령 수신 → Stage 0 시작 (목표 방향 회전)"
        )

    def control_loop(self):
        if not self.aligning or self.odom is None or self.target_pose is None:
            return

        # 현재 위치 및 자세
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        _, _, yaw = Quaternion(q.w, q.x, q.y, q.z).to_euler()
        # yaw += pi / 2  # 시뮬 기준 보정

        # 목표 위치 및 자세
        gx = self.target_pose.pose.position.x
        gy = self.target_pose.pose.position.y
        gq = self.target_pose.pose.orientation
        _, _, goal_yaw = Quaternion(gq.w, gq.x, gq.y, gq.z).to_euler()
        # goal_yaw -= pi / 2

        # 오차 계산
        dx = gx - x
        dy = gy - y
        dist = sqrt(dx**2 + dy**2)
        yaw_to_goal = atan2(dy, dx)
        angle_to_goal = self.normalize_angle(yaw_to_goal - yaw)
        final_yaw_error = self.normalize_angle(goal_yaw - yaw)

        # 로그 공통 출력
        self.get_logger().info(
            f"[DEBUG] Robot Pose: ({x:.2f}, {y:.2f}, {np.degrees(yaw):.2f}°) | "
            f"Target: ({gx:.2f}, {gy:.2f}, {np.degrees(goal_yaw):.2f}°) | "
            f"Dist: {dist:.3f} m | YawErr: {np.degrees(angle_to_goal):.2f}° | FinalYawErr: {np.degrees(final_yaw_error):.2f}°"
        )

        cmd = Twist()

        # Stage 0: 방향 먼저 정렬
        if self.stage == 0:
            # 목표 지점과 너무 가까우면 바로 Stage 2로 전환
            if dist < self.pos_tolerance:
                self.stage = 2
                self.get_logger().info(
                    "⚠️ Stage 0 Skip: 목표 위치와 너무 가까움 → 바로 Stage 2로 전환"
                )
                return
            if abs(angle_to_goal) > 0.017:
                cmd.angular.z = -0.8 * angle_to_goal
                # 최소 회전 속도 보장 (0.1 이상)
                if 0 < abs(cmd.angular.z) < 0.1:
                    cmd.angular.z = 0.1 * np.sign(cmd.angular.z)
                self.get_logger().info(
                    f"🔄 Stage 0: 회전 중... (angle_to_goal = {np.degrees(angle_to_goal):.2f}°, angular.z = {cmd.angular.z:.2f})"
                )
            else:
                self.stage = 1
                self.get_logger().info(
                    "✅ Stage 0 완료: 목표 방향 정렬 → Stage 1로 전환 (전진)"
                )
                return

        # Stage 1: 전진
        elif self.stage == 1:
            # ⛔ 전진 전에 방향이 많이 틀어졌다면 Stage 0으로 되돌아감
            if abs(angle_to_goal) > self.yaw_tolerance:
                self.get_logger().warn(
                    "⚠️ Stage 1 진입 시 yaw 오차 초과 → Stage 0으로 되돌아감"
                )
                self.stage = 0
                return

            if dist > self.pos_tolerance:
                linear_speed = 0.2 * dist
                linear_speed = max(min(linear_speed, 0.2), -0.2)  # 속도 제한
                cmd.linear.x = linear_speed
                self.get_logger().info(
                    f"🚶 Stage 1: 전진 중... (dist = {dist:.3f} m, speed = {linear_speed:.2f})"
                )
            else:
                self.stage = 2
                self.get_logger().info(
                    "✅ Stage 1 완료: 위치 도달 → Stage 2로 전환 (최종 각도 정렬)"
                )
                return

        # Stage 2: 도착 후 최종 방향 정렬
        elif self.stage == 2:
            if abs(final_yaw_error) > self.yaw_tolerance:
                angular_speed = -0.8 * final_yaw_error

                if 0 < abs(angular_speed) < 0.3:
                    angular_speed = -0.3 if angular_speed < 0 else 0.3
                cmd.angular.z = angular_speed
                self.get_logger().info(
                    f"🧭 Stage 2: 최종 방향 정렬 중... (error = {np.degrees(final_yaw_error):.2f}°)"
                )
            else:
                self.aligning = False
                self.stage = 0
                self.stop_robot()
                self.done_pub.publish(Bool(data=True))
                self.get_logger().info(
                    "🎉 Stage 2 완료: 위치 + 방향 정렬 성공 → 정렬 종료"
                )
                return

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        self.get_logger().info("🛑 정지 명령 전송 (cmd_vel = 0)")

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PreciseAlignment()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
