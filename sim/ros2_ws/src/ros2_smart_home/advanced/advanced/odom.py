import rclpy
from rclpy.node import Node

from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from squaternion import Quaternion
from math import cos, sin
import tf2_ros
import geometry_msgs.msg

class IMUOdomNode(Node):
    """
    IMU + 속도 기반 odometry 노드
    - IMU에서 방향(theta) 추정
    - /turtlebot_status에서 선속도, 각속도 수신
    - 상대 위치를 추정하고 odom 메시지 및 TF 전송
    """
    def __init__(self):
        super().__init__('imu_odom_node')

        # Subscriber & Publisher
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_offset = 0.0
        self.prev_time = None
        self.imu_initialized = False

        # Odometry 메시지 초기화
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'

        # base_link TF
        self.base_tf = geometry_msgs.msg.TransformStamped()
        self.base_tf.header.frame_id = 'odom'
        self.base_tf.child_frame_id = 'base_link'

        # laser TF
        self.laser_tf = geometry_msgs.msg.TransformStamped()
        self.laser_tf.header.frame_id = 'base_link'
        self.laser_tf.child_frame_id = 'laser'
        self.laser_tf.transform.translation.z = 0.15
        self.laser_tf.transform.rotation.w = 1.0

        # map → odom (고정 변환)
        self.map_tf = geometry_msgs.msg.TransformStamped()
        self.map_tf.header.frame_id = 'map'
        self.map_tf.child_frame_id = 'odom'
        self.map_tf.transform.rotation.w = 1.0
        self.map_timer = self.create_timer(1.0, self.broadcast_map_to_odom)

    def broadcast_map_to_odom(self):
        self.map_tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_tf)

    def imu_callback(self, msg):
        q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        _, _, yaw = q.to_euler()

        if not self.imu_initialized:
            self.imu_offset = yaw
            self.imu_initialized = True
        else:
            self.theta = yaw - self.imu_offset

    def status_callback(self, msg):
        if not self.imu_initialized:
            return
        
        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return
        
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        linear = msg.twist.linear.x
        angular = -msg.twist.angular.z  # 방향 보정

        # 위치 업데이트
        self.x += linear * cos(self.theta) * dt
        self.y += linear * sin(self.theta) * dt
        self.theta += angular * dt

        q = Quaternion.from_euler(0, 0, self.theta)

        # TF: odom → base_link
        self.base_tf.header.stamp = now.to_msg()
        self.base_tf.transform.translation.x = self.x
        self.base_tf.transform.translation.y = self.y
        self.base_tf.transform.rotation.x = q.x
        self.base_tf.transform.rotation.y = q.y
        self.base_tf.transform.rotation.z = q.z
        self.base_tf.transform.rotation.w = q.w

        # TF: base_link → laser
        self.laser_tf.header.stamp = now.to_msg()

        # Odometry 메시지 구성
        self.odom_msg.header.stamp = now.to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.x = q.x
        self.odom_msg.pose.pose.orientation.y = q.y
        self.odom_msg.pose.pose.orientation.z = q.z
        self.odom_msg.pose.pose.orientation.w = q.w
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular

        # 퍼블리시
        self.tf_broadcaster.sendTransform(self.base_tf)
        self.tf_broadcaster.sendTransform(self.laser_tf)
        self.odom_pub.publish(self.odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
