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
    sub2 odom 노드는 IMU 없이, /turtlebot_status 토픽의 선속도와 각속도를 이용해
    로봇의 위치를 추정하고 odom 메시지 및 TF를 퍼블리시합니다.
    """

    def __init__(self):
        super().__init__('odom')
        
        # Publisher & Subscriber
        self.subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # TF Broadcaster (base_link & laser)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # map -> odom broadcaster (static)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.map_tf_timer = self.create_timer(1.0, self.broadcast_map_to_odom_tf)

        # 상태 변수
        self.is_first = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = 0.0

        # 메시지 초기화
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'

        self.base_link_tf = geometry_msgs.msg.TransformStamped()
        self.base_link_tf.header.frame_id = 'odom'
        self.base_link_tf.child_frame_id = 'base_link'

        self.laser_tf = geometry_msgs.msg.TransformStamped()
        self.laser_tf.header.frame_id = 'base_link'
        self.laser_tf.child_frame_id = 'laser'
        self.laser_tf.transform.translation.x = 0.01  # LiDAR 위치 (시뮬레이터 기준)
        self.laser_tf.transform.translation.y = 0.0
        self.laser_tf.transform.translation.z = 0.19
        self.laser_tf.transform.rotation.w = 1.0  # 회전 없음

        # map -> odom 초기 설정
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

    def listener_callback(self, msg):
        print('linear_vel : {}  angular_vel : {}'.format(msg.twist.linear.x,-msg.twist.angular.z))
        current_time = self.get_clock().now()

        if not self.is_first:
            self.prev_time = current_time
            self.is_first = True
            return
        
        # 주기 계산
        period = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        # 속도 정보
        linear_x = msg.twist.linear.x
        angular_z = -msg.twist.angular.z  # 시뮬레이터 기준 방향 보정

        # 위치 추정 (상대 이동 누적)
        self.x += linear_x * cos(self.theta) * period
        self.y += linear_x * sin(self.theta) * period
        self.theta += angular_z * period
        
        q = Quaternion.from_euler(0, 0, self.theta)
        
        # odom 메시지 구성
        self.odom_msg.header.stamp = current_time.to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.x = q.x
        self.odom_msg.pose.pose.orientation.y = q.y
        self.odom_msg.pose.pose.orientation.z = q.z
        self.odom_msg.pose.pose.orientation.w = q.w
        self.odom_msg.twist.twist.linear.x = linear_x
        self.odom_msg.twist.twist.angular.z = angular_z

        # TF: base_link -> odom
        self.base_link_tf.header.stamp = current_time.to_msg()
        self.base_link_tf.transform.translation.x = self.x
        self.base_link_tf.transform.translation.y = self.y
        self.base_link_tf.transform.translation.z = 0.0
        self.base_link_tf.transform.rotation.x = q.x
        self.base_link_tf.transform.rotation.y = q.y
        self.base_link_tf.transform.rotation.z = q.z
        self.base_link_tf.transform.rotation.w = q.w

        # TF: laser -> base_link
        self.laser_tf.header.stamp = current_time.to_msg()

        # Publish
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




       
   
