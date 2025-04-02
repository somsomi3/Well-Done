import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
import os
from math import sqrt
import sub2

# 🛤️ make_path 노드 설명
# - 로봇의 위치(Odometry)를 받아 **0.1m 간격**으로 (x, y) 좌표를 기록합니다.
# - 기록된 경로는 **Path 메시지**로 Publish되며, RViz에서 시각화할 수 있습니다.
# - 생성된 텍스트 파일(`path.txt`)은 추후 경로 추종 노드(`path_pub`)에서 사용됩니다.
#
# 📌 실행 방법:
# $ ros2 run sub1 make_path
# ※ 실행 시 터미널 경로를 `C:\Users\SSAFY\Desktop\temp\S12P21E102\sim\ros2_ws\src\ros2_smart_home/sub1/sub1`로 설정해야 합니다.


class makePath(Node):

    def __init__(self):
        super().__init__("make_path")

        # ✅ Publisher & Subscriber 생성
        self.path_pub = self.create_publisher(Path, "global_path", 10)
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.listener_callback, 10
        )

        # ✅ 경로 저장할 파일 설정
        full_path = os.path.join(os.getcwd(), "path.txt")
        self.f = open(full_path, "w")

        self.is_odom = False
        self.prev_x = 0.0
        self.prev_y = 0.0

        # ✅ Path 메시지 설정
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

    def listener_callback(self, msg):
        """Odometry 데이터를 받아와 경로를 기록하는 콜백 함수"""
        print(f"x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}")

        if not self.is_odom:
            self.is_odom = True
            self.prev_x = msg.pose.pose.position.x
            self.prev_y = msg.pose.pose.position.y
            return

        # ✅ 현재 위치 및 거리 계산
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        distance = sqrt((x - self.prev_x) ** 2 + (y - self.prev_y) ** 2)

        # ✅ 0.1m 이상 이동 시 경로 업데이트
        if distance > 0.1:
            print("🟢 0.1m 이상 이동 감지됨! 경로 저장 중...")
            waypint_pose = PoseStamped()
            waypint_pose.pose.position.x = x
            waypint_pose.pose.position.y = y
            waypint_pose.pose.orientation.w = 1.0

            # 경로 메시지 업데이트 및 전송
            self.path_msg.poses.append(waypint_pose)
            self.path_pub.publish(self.path_msg)

            # 경로 데이터를 파일에 저장
            self.f.write(f"{x}\t{y}\n")
            # self.f.flush()  # ✅ 즉시 디스크에 저장

            # 이전 위치 업데이트
            self.prev_x = x
            self.prev_y = y


def main(args=None):
    """노드 실행 함수"""
    rclpy.init(args=args)
    odom_based_make_path = makePath()
    try:
        rclpy.spin(odom_based_make_path)  # 실행 중
    except KeyboardInterrupt:
        print("🛑 노드 종료 중...")
    finally:
        # ✅ 종료 시 파일 닫기
        odom_based_make_path.f.close()
        odom_based_make_path.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
