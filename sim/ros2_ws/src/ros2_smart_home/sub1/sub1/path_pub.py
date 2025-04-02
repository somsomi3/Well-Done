import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path

from math import pi, cos, sin, sqrt
import tf2_ros
import os


# 🛤️ path_pub 노드
# - make_path에서 생성한 경로 데이터를 읽어와 전역 경로(global_path)를 publish
# - 현재 로봇 위치를 기반으로 지역 경로(local_path)를 생성하여 publish
#
# 📌 실행 방법:
# $ ros2 run sub1 path_pub
# ※ 실행 시 터미널 경로를 `C:\Users\SSAFY\Desktop\temp\S12P21E102\sim\ros2_ws\src\ros2_smart_home/sub1/sub1`로 설정해야 합니다.
class pathPub(Node):

    def __init__(self):
        super().__init__("path_pub")

        # ✅ Publisher & Subscriber 생성
        self.global_path_pub = self.create_publisher(Path, "global_path", 10)
        self.local_path_pub = self.create_publisher(Path, "local_path", 10)
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.listener_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            Odometry, "/odom", self.listener_callback, 10
        )

        self.odom_msg = Odometry()
        self.is_odom = False  # Odometry 데이터 수신 여부

        # ✅ 전역 경로 초기화
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = "map"

        # ✅ 경로 파일 읽기
        full_path = os.path.join(os.getcwd(), "path.txt")
        self.f = open(full_path, "r")

        lines = self.f.readlines()
        for line in lines:
            tmp = line.strip().split("\t")  # 파일에서 읽어온 좌표 (탭으로 구분됨)
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1.0  # 기본 회전 값
            self.global_path_msg.poses.append(read_pose)

        self.f.close()

        # ✅ 타이머 함수 설정 (20ms마다 실행)
        time_period = 0.02
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size = 20  # 지역 경로의 길이
        self.count = 0  # global_path 업데이트 카운트

    def listener_callback(self, msg):
        """Odometry 데이터 수신 및 저장"""
        self.is_odom = True
        self.odom_msg = msg

    def timer_callback(self):
        """주기적으로 지역 경로 생성 및 publish"""
        if self.is_odom == True:

            local_path_msg = Path()
            local_path_msg.header.frame_id = "map"

            x = self.odom_msg.pose.pose.position.x
            y = self.odom_msg.pose.pose.position.y
            print(f"현재 위치: x={x}, y={y}")

            current_waypoint = -1

            # ✅ 현재 로봇과 가장 가까운 경로점 찾기
            min_dis = float("inf")
            for i, waypoint in enumerate(self.global_path_msg.poses):
                distance = sqrt(
                    (x - waypoint.pose.position.x) ** 2
                    + (y - waypoint.pose.position.y) ** 2
                )
                if distance < min_dis:
                    min_dis = distance
                    current_waypoint = i

            # ✅ local_path 예외 처리 및 생성
            if current_waypoint != -1:
                if current_waypoint + self.local_path_size < len(
                    self.global_path_msg.poses
                ):
                    local_path_msg.poses = self.global_path_msg.poses[
                        current_waypoint : current_waypoint + self.local_path_size
                    ]
                else:
                    local_path_msg.poses = self.global_path_msg.poses[current_waypoint:]

            # ✅ 지역 경로 publish
            self.local_path_pub.publish(local_path_msg)

        # ✅ global_path는 10회마다 한 번씩 업데이트
        if self.count % 10 == 0:
            self.global_path_pub.publish(self.global_path_msg)
        self.count += 1


def main(args=None):
    """노드 실행 함수"""
    rclpy.init(args=args)
    path_publisher = pathPub()
    rclpy.spin(path_publisher)

    path_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
