import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from math import pi,cos,sin
import tf2_ros
import geometry_msgs.msg
import time

"""💡 로봇의 속도를 이용해 odom 프레임 기준의 상대 위치를 추정하고, 
이를 다른 노드들이 사용할 수 있도록 Odometry 메시지와 TF를 브로드캐스트하는 역할을 한다."""

class odom(Node):

    def __init__(self):
        super().__init__('odom')
        
        # ✅ publisher, subscriber, broadcaster 만들기
        self.subscription = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.imu_sub = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)


        # ✅ 로봇의 pose를 저장할 Odometry 메시지 생성
        self.odom_msg = Odometry()

        # ✅ TF 브로드캐스터 설정 (좌표 변환을 위해 사용)
        self.base_link_transform = geometry_msgs.msg.TransformStamped()
        self.laser_transform = geometry_msgs.msg.TransformStamped()

        # ✅ 초기 상태 변수 설정
        self.is_status=False
        self.is_imu=False
        self.is_calc_theta=False

        # ✅ 초기 로봇 위치 및 방향
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_offset = 0  # 초기 orientation 저장
        self.prev_time = 0


        # ✅ publish, broadcast 할 메시지 설정
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

        self.base_link_transform.header.frame_id = "odom"
        self.base_link_transform.child_frame_id = "base_link"

        self.laser_transform.header.frame_id = "base_link"
        self.laser_transform.child_frame_id = "laser"

        # transform.translation은 이동 변환(x,y,z로 표현)
        # transform.rotation은 회전 변환:쿼터니언(x,y,z,w로 표현)
        self.laser_transform.transform.translation.x = 0.0  # LiDAR 위치 조정 가능
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 0.15  # LiDAR 높이 설정
        self.laser_transform.transform.rotation.w = 1.0  # 기본 회전 값          

    def imu_callback(self,msg):
        # ✅ IMU에서 받은 quaternion을 euler angle로 변환해서 사용
        imu_q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        euler = imu_q.to_euler(degrees=False)  # 라디안 단위
        
        if not self.is_imu:
            self.is_imu = True
            self.imu_offset = euler[2]  # Yaw 값 저장
        else:
            self.theta = euler[2] - self.imu_offset  # 초기 offset 보정

    def listener_callback(self, msg):
        # ✅ 속도 데이터를 이용해 로봇의 위치 추정
        print(f'linear_vel : {msg.twist.linear.x},  angular_vel : {-msg.twist.angular.z}')
        
        if self.is_imu:
            if not self.is_status:
                self.is_status = True
                # self.prev_time = rclpy.clock.Clock().now()
                self.prev_time = self.get_clock().now() # ROS 자체와 동기화된 시간 (권장)
            else :
                # self.current_time=rclpy.clock.Clock().now()
                current_time = self.get_clock().now()
                self.period=(current_time - self.prev_time).nanoseconds / 1e9 # 초 단위 변환

                # ✅ 로봇의 선속도, 각속도 저장
                linear_x = msg.twist.linear.x
                angular_z = -msg.twist.angular.z  # ROS2 시뮬레이터에서 방향이 반대라 보정 필요
                
                # ✅ 로봇 위치 추정
                # x = x + (속도 * cos(현재 방향) * 시간)
                self.x += linear_x * cos(self.theta) * self.period
                # y = y + (속도 * sin(현재 방향) * 시간)
                self.y += linear_x * sin(self.theta) * self.period
                # theta = theta + (각속도 * 시간)
                self.theta += angular_z * self.period

                # ✅ Transform 설정
                self.base_link_transform.header.stamp = current_time.to_msg()
                self.laser_transform.header.stamp = current_time.to_msg()
                
                # ✅ 추정한 로봇 위치를 메시지에 담아 publish, broadcast
                q = Quaternion.from_euler(0, 0, self.theta)

                # 좌표 변환 정보 업데이트
                self.base_link_transform.transform.translation.x = self.x
                self.base_link_transform.transform.translation.y = self.y
                self.base_link_transform.transform.rotation.x = q.x
                self.base_link_transform.transform.rotation.y = q.y
                self.base_link_transform.transform.rotation.z = q.z
                self.base_link_transform.transform.rotation.w = q.w

                # Odometry 메시지 업데이트
                self.odom_msg.header.stamp = current_time.to_msg()
                self.odom_msg.pose.pose.position.x = self.x
                self.odom_msg.pose.pose.position.y = self.y
                self.odom_msg.pose.pose.orientation.x = q.x
                self.odom_msg.pose.pose.orientation.y = q.y
                self.odom_msg.pose.pose.orientation.z = q.z
                self.odom_msg.pose.pose.orientation.w = q.w
                self.odom_msg.twist.twist.linear.x = linear_x
                self.odom_msg.twist.twist.angular.z = angular_z
                
                # ✅ 메시지 브로드캐스트 및 퍼블리시
                self.broadcaster.sendTransform(self.base_link_transform)
                self.broadcaster.sendTransform(self.laser_transform)
                self.odom_publisher.publish(self.odom_msg)

                # 이전 시간 업데이트
                self.prev_time = current_time

        
def main(args=None):
    rclpy.init(args=args)
    odom_node = odom()
    rclpy.spin(odom_node)

    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()