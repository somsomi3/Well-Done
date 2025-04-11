import rclpy
from rclpy.node import Node

from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from squaternion import Quaternion
from math import cos, sin, sqrt, radians, pi
import tf2_ros
import geometry_msgs.msg


class IMUOdomNode(Node):
    """
    IMU + 속도 기반 odometry 노드
    - IMU에서 방향(theta) 추정
    - /turtlebot_status에서 선속도, 각속도 수신
    - 상대 위치를 추정하고 odom 메시지 및 TF 전송

    (동시에 절대 위치 기반 Odometry도 퍼블리시함)
    - turtlebot_status의 twist.angular.x, y, z 기반 위치 및 heading 사용
    - /odom_true, /odom_est 두 가지 퍼블리셔 운영
    """

    def __init__(self):
        super().__init__("imu_odom_node")
        self.log_count = 0
        self.use_abs = True  # ✅ 절대 좌표 기반 odom 사용 여부

        # Subscribers
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.create_subscription(
            TurtlebotStatus, "/turtlebot_status", self.status_callback, 10
        )

        # Publishers
        self.odom_pub_true = self.create_publisher(
            Odometry, "odom_true", 10
        )  # 절대 위치 odom
        self.odom_pub_est = self.create_publisher(
            Odometry, "odom_est", 10
        )  # 예측 기반 odom

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 상태 변수
        self.x_est = 0.0
        self.y_est = 0.0
        self.theta_est = 0.0
        self.prev_time = None

        self.x_true = 0.0
        self.y_true = 0.0
        self.theta_true = 0.0

        self.imu_offset = 0.0
        self.imu_initialized = False

        self.odom_msg_est = Odometry()
        self.odom_msg_est.header.frame_id = "odom"
        self.odom_msg_est.child_frame_id = "base_link"

        self.odom_msg_true = Odometry()
        self.odom_msg_true.header.frame_id = "odom"
        self.odom_msg_true.child_frame_id = "base_link"

        self.base_tf = geometry_msgs.msg.TransformStamped()
        self.base_tf.header.frame_id = "odom"
        self.base_tf.child_frame_id = "base_link"

        self.laser_tf = geometry_msgs.msg.TransformStamped()
        self.laser_tf.header.frame_id = "base_link"
        self.laser_tf.child_frame_id = "laser"
        self.laser_tf.transform.translation.z = 0.10
        self.laser_tf.transform.rotation.w = 1.0

        self.map_tf = geometry_msgs.msg.TransformStamped()
        self.map_tf.header.frame_id = "map"
        self.map_tf.child_frame_id = "odom"
        self.map_tf.transform.rotation.w = 1.0

        self.get_logger().info("IMUOdomNode initialized.")

    def broadcast_map_to_odom(self, stamp=None):
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        self.map_tf.header.stamp = stamp
        self.tf_broadcaster.sendTransform(self.map_tf)

    def imu_callback(self, msg):
        q = Quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        _, _, yaw = q.to_euler()

        if not self.imu_initialized:
            self.imu_offset = yaw
            self.imu_initialized = True
        else:
            self.theta_est = yaw - self.imu_offset + pi

    def status_callback(self, msg):
        if not self.imu_initialized:
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()

        dt = (now - self.prev_time).nanoseconds / 1e9 if self.prev_time else 0.0
        self.prev_time = now

        linear = msg.twist.linear.x
        angular = -msg.twist.angular.z

        # --- 상대 좌표 기반 odom (예측 기반) ---
        self.theta_est += angular * dt
        self.x_est += linear * cos(self.theta_est) * dt
        self.y_est += linear * sin(self.theta_est) * dt

        q_est = Quaternion.from_euler(0, 0, self.theta_est - pi / 2)
        self.odom_msg_est.header.stamp = now_msg
        self.odom_msg_est.pose.pose.position.x = self.x_est
        self.odom_msg_est.pose.pose.position.y = self.y_est
        self.odom_msg_est.pose.pose.orientation.x = q_est.x
        self.odom_msg_est.pose.pose.orientation.y = q_est.y
        self.odom_msg_est.pose.pose.orientation.z = q_est.z
        self.odom_msg_est.pose.pose.orientation.w = q_est.w
        self.odom_msg_est.twist.twist.linear.x = linear
        self.odom_msg_est.twist.twist.angular.z = angular
        self.odom_pub_est.publish(self.odom_msg_est)

        # --- 절대 좌표 기반 odom (turtlebot_status) ---
        self.x_true = msg.twist.angular.x
        self.y_true = msg.twist.angular.y
        theta_deg = (msg.twist.linear.z - 90 + 180) % 360 - 180  # ✅ -180~180 유지
        self.theta_true = radians(theta_deg)

        q_true = Quaternion.from_euler(0, 0, self.theta_true)
        self.odom_msg_true.header.stamp = now_msg
        self.odom_msg_true.pose.pose.position.x = self.x_true
        self.odom_msg_true.pose.pose.position.y = self.y_true
        self.odom_msg_true.pose.pose.orientation.x = q_true.x
        self.odom_msg_true.pose.pose.orientation.y = q_true.y
        self.odom_msg_true.pose.pose.orientation.z = q_true.z
        self.odom_msg_true.pose.pose.orientation.w = q_true.w
        self.odom_msg_true.twist.twist.linear.x = linear
        self.odom_msg_true.twist.twist.angular.z = angular
        self.odom_pub_true.publish(self.odom_msg_true)

        # --- TF ---
        self.base_tf.header.stamp = now_msg
        if self.use_abs:
            self.base_tf.transform.translation.x = self.x_true
            self.base_tf.transform.translation.y = self.y_true
            q = Quaternion.from_euler(0, 0, self.theta_true)
        else:
            self.base_tf.transform.translation.x = self.x_est
            self.base_tf.transform.translation.y = self.y_est
            q = Quaternion.from_euler(0, 0, self.theta_est - pi / 2)
        self.base_tf.transform.rotation.x = q.x
        self.base_tf.transform.rotation.y = q.y
        self.base_tf.transform.rotation.z = q.z
        self.base_tf.transform.rotation.w = q.w
        self.tf_broadcaster.sendTransform(self.base_tf)

        self.laser_tf.header.stamp = now_msg
        self.tf_broadcaster.sendTransform(self.laser_tf)

        self.broadcast_map_to_odom(now_msg)

        # --- 오차 출력 ---
        self.log_count += 1
        if self.log_count % 10 == 0:
            self.log_count = 0
            error = sqrt(
                (self.x_est - self.x_true) ** 2 + (self.y_est - self.y_true) ** 2
            )
            self.get_logger().info(
                f"[ODOM] Predicted=({self.x_est:.2f}, {self.y_est:.2f}) | Sim=({self.x_true:.2f}, {self.y_true:.2f}) | Error={error:.4f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = IMUOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
