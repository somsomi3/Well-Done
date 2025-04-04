import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
import numpy as np
import time


def get_distance(p1, p2):
    return np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))


def get_heading(odom_msg):
    q = odom_msg.pose.pose.orientation
    quat = Quaternion(q.w, q.x, q.y, q.z)
    _, _, heading = quat.to_euler()
    return heading


class AutoMapper(Node):
    def __init__(self):
        super().__init__("auto_mapper")

        # === 💡 CONFIGURATION ===
        self.LINEAR_SPEED = 0.8  # 전진 속도
        self.ANGULAR_SPEED = 0.5  # 회전 속도
        self.OBSTACLE_DISTANCE_THRESHOLD = 1.0  # 장애물 감지 거리
        self.FORWARD_DISTANCE = 1.0  # 직진 거리
        self.TURN_ANGLE = np.pi / 2  # 회전 각도 (90도)
        self.AVOID_ANGLE = np.pi / 4  # 회피 각도 (45도)
        self.FRONT_ANGLE_DEG = 45  # 라이다로 검사할 전방 각도 범위
        self.MAP_CHANGE_THRESHOLD = 0.01  # 맵 변화율 임계값
        self.MAP_COVERAGE_THRESHOLD = 0.90  # 커버리지 임계값
        self.MAP_IDLE_DURATION = 5.0  # 변화 없는 시간 초과 시 종료

        # 퍼블리셔 및 서브스크라이버 초기화
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, 10
        )

        # 맵 정보
        self.map_data = None
        self.prev_map = None
        self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.last_change_time = self.last_update_time

        # 상태 및 타이머
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.direction = 1  # 정방향
        self.state = "FORWARD"  # FORWARD, TURN, AVOID
        self.step = 0

        self.rotation_start_angle = None
        self.start_pose = None
        self.current_pose = None

        # 장애물 회피 관련
        self.is_obstacle_detected = False

        self.get_logger().info("Auto mapping started.")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = get_heading(msg)
        self.current_pose = [x, y, theta]
        if self.start_pose is None:
            self.start_pose = self.current_pose.copy()

    def map_callback(self, msg):
        new_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        now = self.get_clock().now().seconds_nanoseconds()[0]

        if self.prev_map is not None:
            diff = np.abs(new_map - self.prev_map)
            change_rate = np.count_nonzero(diff) / diff.size
            self.get_logger().info(f"[MAP] Change rate: {change_rate:.4f}")

            observed = (new_map == 0) | (new_map == 100)
            coverage = np.count_nonzero(observed) / new_map.size
            self.get_logger().info(f"[MAP] Coverage: {coverage:.2%}")

            # 변화량이 크면 시간 초기화
            if change_rate >= self.MAP_CHANGE_THRESHOLD:
                self.last_change_time = now
                self.get_logger().info(
                    f"[MAP] Significant change detected. Resetting last_change_time."
                )

            duration = now - self.last_change_time
            self.get_logger().info(
                f"[MAP] Duration since last change: {duration:.2f} sec"
            )

            # 매핑 종료 조건
            if (
                duration > self.MAP_IDLE_DURATION
                and coverage > self.MAP_COVERAGE_THRESHOLD
            ):
                self.get_logger().info(
                    f"✅ [MAP] Mapping complete - change rate = {change_rate:.4f}, coverage = {coverage:.2%}"
                )
                self.stop_robot()
                self.destroy_node()
            else:
                self.get_logger().info("[MAP] Mapping continues...")
        else:
            self.get_logger().info("[MAP] First map received.")

        self.prev_map = new_map
        self.map_data = new_map
        self.last_update_time = now

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        num_ranges = len(ranges)
        angle_half = self.FRONT_ANGLE_DEG / 360 * num_ranges
        front_indices = range(
            int(num_ranges / 2 - angle_half), int(num_ranges / 2 + angle_half)
        )

        front_ranges = ranges[list(front_indices)]
        valid_ranges = front_ranges[
            (front_ranges > msg.range_min) & (front_ranges < msg.range_max)
        ]

        self.is_obstacle_detected = (
            len(valid_ranges) > 0
            and np.min(valid_ranges) < self.OBSTACLE_DISTANCE_THRESHOLD
        )

    def timer_callback(self):
        if self.current_pose is None:
            self.get_logger().warn(
                "[TIMER] Current pose is None. Skipping control step."
            )
            return

        msg = Twist()

        if self.is_obstacle_detected and self.state != "AVOID":
            self.state = "AVOID"
            self.rotation_start_angle = None
            self.stop_robot()
            self.get_logger().warn(
                "⚠️ [TIMER] 🚧 Obstacle detected! Switching to AVOID state."
            )

        self.get_logger().info(f"[TIMER] Current state: {self.state}")

        if self.state == "AVOID":
            if self.rotation_start_angle is None:
                self.rotation_start_angle = self.current_pose[2]
                self.get_logger().info(
                    f"[TIMER][AVOID] Starting avoidance rotation from {np.degrees(self.rotation_start_angle):.2f} deg"
                )
            else:
                angle_diff = self.normalize_angle(
                    self.current_pose[2] - self.rotation_start_angle
                )
                self.get_logger().info(
                    f"[TIMER][AVOID] Rotating... Δθ = {np.degrees(angle_diff):.2f}°"
                )
                if abs(angle_diff) < self.AVOID_ANGLE:
                    msg.angular.z = self.ANGULAR_SPEED
                else:
                    self.get_logger().info(
                        "✅ [TIMER][AVOID] Avoidance complete. Returning to FORWARD state."
                    )
                    self.stop_robot()
                    self.state = "FORWARD"
                    self.rotation_start_angle = None
                    self.start_pose = self.current_pose.copy()

        elif self.state == "FORWARD":
            if self.is_obstacle_detected:
                self.get_logger().warn(
                    "[TIMER][FORWARD] Obstacle detected during FORWARD. Halting."
                )
                msg.linear.x = 0.0
                self.pub.publish(msg)
                return
            dist = get_distance(self.current_pose, self.start_pose)
            self.get_logger().info(f"[TIMER][FORWARD] Distance traveled: {dist:.3f} m")
            if dist < self.FORWARD_DISTANCE:
                msg.linear.x = self.LINEAR_SPEED
            else:
                self.get_logger().info(
                    "[TIMER][FORWARD] Reached target distance. Switching to TURN."
                )
                self.state = "TURN"
                self.last_action_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.start_pose = self.current_pose.copy()

        elif self.state == "TURN":
            if self.rotation_start_angle is None:
                self.rotation_start_angle = self.current_pose[2]
                self.get_logger().info(
                    f"[TIMER][TURN] Starting rotation from {np.degrees(self.rotation_start_angle):.2f} deg"
                )
            else:
                angle_diff = self.normalize_angle(
                    self.current_pose[2] - self.rotation_start_angle
                )
                self.get_logger().info(
                    f"[TIMER][TURN] Rotating... angle diff: {np.degrees(angle_diff):.2f} deg"
                )
                if abs(angle_diff) < self.turn_angle:
                    msg.angular.z = self.ANGULAR_SPEED * self.direction
                else:
                    self.get_logger().info(
                        f"✅ [TIMER][TURN] Turn complete: {np.degrees(angle_diff):.2f}°. Switching to FORWARD."
                    )
                    self.rotation_start_angle = None
                    self.direction *= -1
                    self.state = "FORWARD"
                    self.last_action_time = (
                        self.get_clock().now().seconds_nanoseconds()[0]
                    )
                    self.start_pose = self.current_pose.copy()
                    self.step += 1
                    self.get_logger().info(
                        f"[TIMER][TURN] Step count updated to {self.step}"
                    )

        self.pub.publish(msg)

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def stop_robot(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
