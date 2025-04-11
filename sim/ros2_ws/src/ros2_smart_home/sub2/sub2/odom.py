import rclpy
from rclpy.node import Node

from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from math import pi,cos,sin
import tf2_ros
import geometry_msgs.msg
import time

class odom(Node):
    """
    sub2 odom 노드는 /turtlebot_status에서 제공하는 절대 위치(x, y)를 기반으로
    odom 메시지 및 TF를 퍼블리시합니다.
    방향(heading)은 IMU의 orientation을 이용하여 계산합니다.
    """
    def __init__(self):
        super().__init__('odom')
        
        # Publisher & Subscriber 생성
        self.subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # TF Broadcaster (base_link & laser)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # map -> odom broadcaster (static)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.map_tf_timer = self.create_timer(1.0, self.broadcast_map_to_odom_tf)

        # 상태 변수
        self.theta = 0.0

        # Odometry 메시지 초기화 (odom frame 사용)
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'

        # base_link TF 설정 (odom → base_link)
        self.base_link_tf = geometry_msgs.msg.TransformStamped()
        self.base_link_tf.header.frame_id = 'odom'
        self.base_link_tf.child_frame_id = 'base_link'

        # laser TF 설정 (base_link → laser)
        self.laser_tf = geometry_msgs.msg.TransformStamped()
        self.laser_tf.header.frame_id = 'base_link'
        self.laser_tf.child_frame_id = 'laser'
        self.laser_tf.transform.translation.x = 0.0  # LiDAR 위치 (시뮬레이터 기준)
        self.laser_tf.transform.translation.y = 0.0
        self.laser_tf.transform.translation.z = 0.10
        self.laser_tf.transform.rotation.w = 1.0  # 회전 없음

        # map → odom 정적 TF 설정
        self.map_to_odom = geometry_msgs.msg.TransformStamped()
        self.map_to_odom.header.frame_id = 'map'
        self.map_to_odom.child_frame_id = 'odom'
        self.map_to_odom.transform.translation.x = 0.0
        self.map_to_odom.transform.translation.y = 0.0
        self.map_to_odom.transform.translation.z = 0.0
        self.map_to_odom.transform.rotation.w = 1.0

    def broadcast_map_to_odom_tf(self):
        self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self.static_broadcaster.sendTransform(self.map_to_odom)

    def imu_callback(self, msg):
        q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        _, _, yaw = q.to_euler()
        self.theta = yaw

    def listener_callback(self, msg):
        print('linear_vel : {}  angular_vel : {}'.format(msg.twist.linear.x,-msg.twist.angular.z))
        current_time = self.get_clock().now()

        # 절대 위치 (turtlebot_status의 twist.angular.x, twist.angular.y 사용)
        x = msg.twist.angular.x
        y = msg.twist.angular.y

        # 속도 정보 (방향 보정 적용)
        linear_x = msg.twist.linear.x
        angular_z = -msg.twist.angular.z  # 시뮬레이터 기준 방향 보정

        q = Quaternion.from_euler(0, 0, self.theta)
        
        # odom 메시지 구성
        self.odom_msg.header.stamp = current_time.to_msg()
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        self.odom_msg.pose.pose.orientation.x = q.x
        self.odom_msg.pose.pose.orientation.y = q.y
        self.odom_msg.pose.pose.orientation.z = q.z
        self.odom_msg.pose.pose.orientation.w = q.w
        self.odom_msg.twist.twist.linear.x = linear_x
        self.odom_msg.twist.twist.angular.z = angular_z

        # base_link TF 구성
        self.base_link_tf.header.stamp = current_time.to_msg()
        self.base_link_tf.transform.translation.x = x
        self.base_link_tf.transform.translation.y = y
        self.base_link_tf.transform.translation.z = 0.0
        self.base_link_tf.transform.rotation.x = q.x
        self.base_link_tf.transform.rotation.y = q.y
        self.base_link_tf.transform.rotation.z = q.z
        self.base_link_tf.transform.rotation.w = q.w

        # laser TF 타임스탬프 업데이트
        self.laser_tf.header.stamp = current_time.to_msg()

        # TF와 odom 메시지 publish
        self.tf_broadcaster.sendTransform(self.base_link_tf)
        self.tf_broadcaster.sendTransform(self.laser_tf)
        self.odom_publisher.publish(self.odom_msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = odom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()