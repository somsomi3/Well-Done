import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from math import pi, cos, sin, sqrt, atan2
import numpy as np

class followTheCarrot(Node):

    """ 
    📌 경로 추종 노드 (Path Tracking)
    - 로봇의 현재 위치(/odom), 속도(/turtlebot_status), 경로(/local_path)를 받아
      전방 주시 포인트를 기준으로 속도 및 방향을 결정하여 /cmd_vel 퍼블리시
    """

    def __init__(self):
        super().__init__('path_tracking')

        # 🔌 Publisher & Subscriber 등록
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)

        # ⏱️ 제어 주기 타이머 설정 (주기: 0.01초 = 100Hz)
        self.time_period = 0.01
        self.timer = self.create_timer(self.time_period, self.timer_callback)

        # 🧭 상태 플래그 및 메시지 객체 초기화
        self.is_odom = False
        self.is_path = False
        self.is_status = False

        self.odom_msg = Odometry()
        self.path_msg = Path()
        self.robot_yaw = 0.0
        self.prev_theta = 0.0  # 이전 조향각 (출렁임 방지용)
        self.cmd_msg = Twist()

        # 📏 전방 주시 거리(Look-Forward Distance) 설정
        self.lfd = 0.1
        self.min_lfd = 0.1
        self.max_lfd = 2.0

    def timer_callback(self):
        if self.is_odom and self.is_path and self.is_status:
            if len(self.path_msg.poses) > 1:
                self.is_look_forward_point = False

                # 📍 로봇 현재 위치
                robot_x = self.odom_msg.pose.pose.position.x
                robot_y = self.odom_msg.pose.pose.position.y

                # 📏 lateral error로부터 전방 주시 거리 계산
                lateral_error = sqrt(
                    (self.path_msg.poses[0].pose.position.x - robot_x) ** 2 +
                    (self.path_msg.poses[0].pose.position.y - robot_y) ** 2
                )
                # self.lfd = min(self.max_lfd, max(self.min_lfd, lateral_error))
                self.lfd = 0.5 # 고정된 전방 주시 거리 사용

                min_dis = float('inf')

                # 🔍 전방 주시 포인트 탐색
                for waypoint in self.path_msg.poses:
                    point = waypoint.pose.position
                    dis = sqrt((point.x - self.path_msg.poses[0].pose.position.x) ** 2 + 
                    (point.y - self.path_msg.poses[0].pose.position.y) ** 2)

                    if abs(dis - self.lfd) < min_dis:
                        min_dis = abs(dis - self.lfd)
                        self.forward_point = point
                        self.is_look_forward_point = True

                if self.is_look_forward_point:
                    global_fp = [self.forward_point.x, self.forward_point.y, 1]

                    # 🔄 로컬 좌표계로 변환 (2D 동차 좌표계 이용)
                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_x],
                        [sin(self.robot_yaw),  cos(self.robot_yaw), robot_y],
                        [0,                   0,                    1]
                    ])
                    det_trans = np.linalg.inv(trans_matrix)
                    local_fp = det_trans.dot(np.array(global_fp).reshape(3, 1))

                    # 🎯 조향 각도(theta) 계산
                    theta = -atan2(local_fp[1][0], local_fp[0][0])

                    # 📉 저역 필터를 이용한 조향 각도 변화 완화
                    alpha = 0.5  # 0에 가까울수록 반응이 느림
                    theta = alpha * theta + (1 - alpha) * self.prev_theta
                    self.prev_theta = theta

                    # 🚗 선속도 계산 (cos(theta)로 전방 정렬 시 최대 속도)
                    out_vel = max(0.0, 1 * cos(theta))
                    
                    # 🔄 각속도 계산 (Kp 게인 조정 및 제한)
                    Kp = 1.5
                    out_rad_vel = Kp * theta                       # 감쇠된 각속도
                    out_rad_vel = max(-1.0, min(1.0, out_rad_vel))

                    self.cmd_msg.linear.x = float(out_vel)
                    self.cmd_msg.angular.z = float(out_rad_vel)
                else:
                    print("⚠️ 전방 주시 포인트를 찾을 수 없음")
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0

                # 📨 최종 속도 명령 퍼블리시
                self.cmd_pub.publish(self.cmd_msg)

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg

        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = Quaternion(q.w, q.x, q.y, q.z).to_euler()

    def path_callback(self, msg):
        self.is_path = True
        self.path_msg = msg

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = followTheCarrot()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
