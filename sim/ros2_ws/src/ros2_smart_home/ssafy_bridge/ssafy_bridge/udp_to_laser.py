import numpy as np
import cv2
import socket
import time

import rclpy
from rclpy.node import Node
from ssafy_bridge.utils import UDP_LIDAR_Parser

from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from ssafy_msgs.msg import ScanWithPose  # ✅ 커스텀 메시지 추가


params_lidar = {
    "CHANNEL": int(1),
    "localIP": "127.0.0.1",
    "localPort": 9094,
    "Block_SIZE": int(1206),
}


class PCPublisher(Node):

    def __init__(self):
        super().__init__(node_name="pointcloud_convertor")

        self.udp_parser = UDP_LIDAR_Parser(
            ip=params_lidar["localIP"],
            port=params_lidar["localPort"],
            params_lidar=params_lidar,
        )

        if params_lidar["CHANNEL"] == int(1):
            self.publisher_laser = self.create_publisher(LaserScan, "/scan", 5)
            self.publisher_pose = self.create_publisher(
                Float32MultiArray, "/pose_from_sim", 10
            )
            self.publisher_scan_with_pose = self.create_publisher(
                ScanWithPose, "/scan_with_pose", 5
            )
        else:
            self.publisher_ = self.create_publisher(PointCloud, "/point_cloud", 5)

        self.timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.prev_time = time.time()  # ✅ scan_time 계산용 시간 초기화

    def timer_callback(self):

        # ✅ now를 한 번만 호출해서 모든 메시지와 TF 시간 일치
        now = self.get_clock().now()
        now_msg = now.to_msg()

        if params_lidar["CHANNEL"] == int(1):
            ranges, intens, aux_data = self.udp_parser.recv_udp_data()

            if len(ranges) == 0:
                return

            # ✅ scan_time 계산
            current_time = time.time()
            scan_time = current_time - self.prev_time
            self.prev_time = current_time

            pose_msg = Float32MultiArray()
            pose_msg.data = aux_data.astype(np.float32).tolist()

            # ✅ LaserScan 메시지 매번 새로 생성
            laser_msg = LaserScan()
            laser_msg.header.stamp = now_msg  # ✅ odom과 동일한 기준 시간 사용
            laser_msg.header.frame_id = "laser"

            laser_msg.angle_min = -np.pi / 2  # 스캔 시작 각도 (rad)
            laser_msg.angle_max = np.pi + np.pi / 2  # 스캔 끝 각도 (rad)
            laser_msg.angle_increment = np.pi / 180  # 각 포인트 간 각도 (rad)

            laser_msg.range_min = 0.0  # 스캔 가능한 최소 거리 (m)
            laser_msg.range_max = 10.0  # 스캔 가능한 최대 거리 (m)
            laser_msg.scan_time = float(
                scan_time
            )  # 한 번 스캔하는 데 걸리는 시간 (sec)
            laser_msg.time_increment = (
                float(scan_time) / 360.0
            )  # 포인트 간 시간차 (sec)

            laser_msg.ranges = ranges.astype(np.float32).tolist()
            laser_msg.ranges[-1] = 0.0  # 마지막 값 무효화
            laser_msg.intensities = intens.astype(np.float32).tolist()

            scan_with_pose_msg = ScanWithPose()
            scan_with_pose_msg.header = Header()
            scan_with_pose_msg.header.stamp = now_msg
            scan_with_pose_msg.header.frame_id = "laser"

            scan_with_pose_msg.angle_min = -np.pi / 2
            scan_with_pose_msg.angle_max = np.pi + np.pi / 2
            scan_with_pose_msg.angle_increment = np.pi / 180

            scan_with_pose_msg.range_min = 0.0
            scan_with_pose_msg.range_max = 10.0
            scan_with_pose_msg.scan_time = float(scan_time)
            scan_with_pose_msg.time_increment = float(scan_time) / 360.0

            scan_with_pose_msg.ranges = ranges.astype(np.float32).tolist()
            scan_with_pose_msg.ranges[-1] = 0.0
            scan_with_pose_msg.intensities = intens.astype(np.float32).tolist()

            scan_with_pose_msg.pose_x = float(aux_data[0])
            scan_with_pose_msg.pose_y = float(aux_data[1])
            scan_with_pose_msg.pose_theta = float(np.deg2rad(aux_data[2]))  # 각도 보정

            self.publisher_pose.publish(pose_msg)  # ✅ pose 따로 퍼블리시
            self.publisher_laser.publish(laser_msg)
            self.publisher_scan_with_pose.publish(scan_with_pose_msg)

        else:
            self.pc_msg = PointCloud()
            self.pc_msg.header.frame_id = "map"
            self.pc_msg.header.stamp = now_msg

            X, Y, Z, Intensity = self.udp_parser.recv_udp_data()

            for x, y, z, intens in zip(
                X.tolist(), Y.tolist(), Z.tolist(), Intensity.tolist()
            ):

                tmp_point = Point32()
                tmp_point.x = x
                tmp_point.y = y
                tmp_point.z = z

                tmp_channelfloat = ChannelFloat32()
                tmp_channelfloat.name = "intensity"
                tmp_channelfloat.values = [intens]

                self.pc_msg.points.append(tmp_point)
                self.pc_msg.channels.append(tmp_channelfloat)

            self.publisher_.publish(self.pc_msg)


def main(args=None):
    rclpy.init(args=args)
    pc_parser = PCPublisher()
    rclpy.spin(pc_parser)


if __name__ == "__main__":

    main()
