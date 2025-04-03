import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from math import pi, cos, sin
import tf2_ros
import geometry_msgs.msg
import time

"""
📌 odom 노드 설명
- IMU로부터 회전값을 받아 상대적인 위치를 추정하고,
  선속도와 각속도를 이용해 odom 프레임 기준 위치를 계산함.
- odom → base_link, base_link → laser, map → odom 변환을 브로드캐스트하여
  RViz에서 전체 로봇과 경로를 함께 확인 가능하게 함.
"""


class odom(Node):

    def __init__(self):
        super().__init__("odom")

        # 🔌 Subscriber & Publisher 설정
        self.subscription = self.create_subscription(
            TurtlebotStatus, "/turtlebot_status", self.listener_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)

        # 🔄 TF Broadcaster 설정
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 🧭 상태 변수 초기화
        self.is_status = False
        self.is_imu = False
        self.is_calc_theta = False

        # 📍 로봇 상태 초기화
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_offset = 0.0
        self.prev_time = 0

        # 📤 Odometry 메시지 초기화
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

        # 🔀 TF: base_link → odom
        self.base_link_transform = geometry_msgs.msg.TransformStamped()
        self.base_link_transform.header.frame_id = "odom"
        self.base_link_transform.child_frame_id = "base_link"

        # 🔀 TF: base_link → laser
        self.laser_transform = geometry_msgs.msg.TransformStamped()
        self.laser_transform.header.frame_id = "base_link"
        self.laser_transform.child_frame_id = "laser"
        """ 
        transform.translation은 이동 변환(x,y,z로 표현)
        transform.rotation은 회전 변환:쿼터니언(x,y,z,w로 표현)
        """
        self.laser_transform.transform.translation.x = 0.0  # LiDAR 위치 조정 가능
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 0.15  # LiDAR 높이 설정
        self.laser_transform.transform.rotation.w = 1.0  # 기본 회전 값

        # 🔀 TF: map → odom
        self.map_to_odom_transform = geometry_msgs.msg.TransformStamped()
        self.map_to_odom_transform.header.frame_id = "map"
        self.map_to_odom_transform.child_frame_id = "odom"
        self.map_to_odom_transform.transform.translation.x = 0.0
        self.map_to_odom_transform.transform.translation.y = 0.0
        self.map_to_odom_transform.transform.translation.z = 0.0
        self.map_to_odom_transform.transform.rotation.w = 1.0

        # ✅ 주기적으로 map → odom TF broadcast
        self.map_tf_timer = self.create_timer(1.0, self.broadcast_map_to_odom_tf)

    def broadcast_map_to_odom_tf(self):
        self.map_to_odom_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_to_odom_transform)

    def imu_callback(self, msg):
        """IMU 데이터를 이용해 로봇의 방향(theta) 추정"""
        imu_q = Quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        euler = imu_q.to_euler(degrees=False)  # 라디안 단위

        if not self.is_imu:
            self.is_imu = True
            self.imu_offset = euler[2]  # Yaw 값 저장
        else:
            self.theta = euler[2] - self.imu_offset  # 초기 yaw 기준 보정

    def listener_callback(self, msg):
        """로봇의 속도 데이터를 이용해 위치 추정 및 TF/Odom 메시지 퍼블리시"""
        print(
            f"linear_vel : {msg.twist.linear.x},  angular_vel : {-msg.twist.angular.z}"
        )

        if self.is_imu:
            if not self.is_status:
                self.is_status = True
                # self.prev_time = rclpy.clock.Clock().now()
                self.prev_time = (
                    self.get_clock().now()
                )  # ROS 자체와 동기화된 시간 (권장)
            else:
                # self.current_time=rclpy.clock.Clock().now()
                current_time = self.get_clock().now()
                self.period = (
                    current_time - self.prev_time
                ).nanoseconds / 1e9  # 초 단위 변환

                # 속도 추출
                linear_x = msg.twist.linear.x
                angular_z = -msg.twist.angular.z  # 방향 보정 필요

                # 위치 계산
                self.x += linear_x * cos(self.theta) * self.period
                self.y += linear_x * sin(self.theta) * self.period
                self.theta += angular_z * self.period

                # 쿼터니언 변환
                q = Quaternion.from_euler(0, 0, self.theta)

                # TF: base_link → odom
                self.base_link_transform.header.stamp = current_time.to_msg()
                self.base_link_transform.transform.translation.x = self.x
                self.base_link_transform.transform.translation.y = self.y
                self.base_link_transform.transform.translation.z = 0.0
                self.base_link_transform.transform.rotation.x = q.x
                self.base_link_transform.transform.rotation.y = q.y
                self.base_link_transform.transform.rotation.z = q.z
                self.base_link_transform.transform.rotation.w = q.w

                # TF: base_link → laser
                self.laser_transform.header.stamp = current_time.to_msg()

                # Odometry 메시지 설정
                self.odom_msg.header.stamp = current_time.to_msg()
                self.odom_msg.pose.pose.position.x = self.x
                self.odom_msg.pose.pose.position.y = self.y
                self.odom_msg.pose.pose.position.z = 0.0
                self.odom_msg.pose.pose.orientation.x = q.x
                self.odom_msg.pose.pose.orientation.y = q.y
                self.odom_msg.pose.pose.orientation.z = q.z
                self.odom_msg.pose.pose.orientation.w = q.w
                self.odom_msg.twist.twist.linear.x = linear_x
                self.odom_msg.twist.twist.angular.z = angular_z

                # TF 및 Odometry 전송
                self.tf_broadcaster.sendTransform(self.base_link_transform)
                self.tf_broadcaster.sendTransform(self.laser_transform)
                self.odom_publisher.publish(self.odom_msg)

                # 이전 시간 업데이트
                self.prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    odom_node = odom()
    rclpy.spin(odom_node)

    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
