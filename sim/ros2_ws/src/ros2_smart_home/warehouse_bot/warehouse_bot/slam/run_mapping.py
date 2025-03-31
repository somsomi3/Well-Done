import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from math import pi, cos, sin
from geometry_msgs.msg import Pose
from ssafy_msgs.msg import ScanWithPose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from squaternion import Quaternion
import warehouse_bot.slam.utils as utils
from warehouse_bot.utils.sim_config import params_map


def createLineIterator(P1, P2, img):
    imageH, imageW = img.shape
    P1X, P1Y = P1
    P2X, P2Y = P2

    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)

    # 직선상 픽셀 좌표를 저장할 버퍼 생성
    itbuffer = np.empty(shape=(np.maximum(dYa, dXa), 3), dtype=np.float32)
    itbuffer.fill(np.nan)

    # 방향 판별 (음수 방향인지 체크)
    negX = P1X > P2X
    negY = P1Y > P2Y

    if P1X == P2X:  # 수직선인 경우
        itbuffer[:, 0] = P1X
        itbuffer[:, 1] = (
            np.arange(P1Y - 1, P1Y - dYa - 1, -1)
            if negY
            else np.arange(P1Y + 1, P1Y + dYa + 1)
        )
    elif P1Y == P2Y:  # 수평선인 경우
        itbuffer[:, 1] = P1Y
        itbuffer[:, 0] = (
            np.arange(P1X - 1, P1X - dXa - 1, -1)
            if negX
            else np.arange(P1X + 1, P1X + dXa + 1)
        )
    else:  # 대각선인 경우
        steepSlope = dYa > dXa
        if steepSlope:  # 기울기가 1보다 큰 경우
            slope = dX.astype(np.float32) / dY.astype(np.float32)
            itbuffer[:, 1] = (
                np.arange(P1Y - 1, P1Y - dYa - 1, -1)
                if negY
                else np.arange(P1Y + 1, P1Y + dYa + 1)
            )
            itbuffer[:, 0] = (slope * (itbuffer[:, 1] - P1Y)).astype(int) + P1X
        else:  # 기울기가 1보다 작은 경우
            slope = dY.astype(np.float32) / dX.astype(np.float32)
            itbuffer[:, 0] = (
                np.arange(P1X - 1, P1X - dXa - 1, -1)
                if negX
                else np.arange(P1X + 1, P1X + dXa + 1)
            )
            itbuffer[:, 1] = (slope * (itbuffer[:, 0] - P1X)).astype(int) + P1Y

    # 이미지 바깥 좌표 제거
    colX, colY = itbuffer[:, 0], itbuffer[:, 1]
    itbuffer = itbuffer[(colX >= 0) & (colY >= 0) & (colX < imageW) & (colY < imageH)]

    # 픽셀 강도 저장 (나중에 사용될 가능성에 대비)
    itbuffer[:, 2] = img[itbuffer[:, 1].astype(np.uint), itbuffer[:, 0].astype(np.uint)]
    return itbuffer


class Mapping:
    def __init__(self, params):
        self.map_resolution = params["MAP_RESOLUTION"]
        self.map_size = (np.array(params["MAP_SIZE"]) / self.map_resolution).astype(int)
        self.map_center = params["MAP_CENTER"]
        self.map = np.ones(self.map_size) * 0.5
        self.occu_up = params["OCCUPANCY_UP"]
        self.occu_down = params["OCCUPANCY_DOWN"]
        self.map_filename = params["MAP_FILENAME"]
        self.map_vis_resize_scale = params["MAPVIS_RESIZE_SCALE"]
        self.T_r_l = np.array(
            [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        )  # 반시계 90도 회전 보정

    def update(self, pose, laser):
        n_points = laser.shape[1]
        pose_mat = utils.xyh2mat2D(pose) @ self.T_r_l
        # pose_mat = utils.xyh2mat2D(pose)  # 보정 없음
        laser_mat = np.vstack((laser, np.ones((1, n_points))))
        laser_global = pose_mat @ laser_mat

        pose_grid = (
            (
                pose[:2].flatten()
                - self.map_center
                + (np.array(self.map_size) * self.map_resolution) / 2
            )
            / self.map_resolution
        ).astype(int)

        laser_grid = (
            (
                laser_global[:2, :].T
                - self.map_center
                + (np.array(self.map_size) * self.map_resolution) / 2
            )
            / self.map_resolution
        ).astype(int)

        for end in laser_grid:
            line_iter = createLineIterator(pose_grid, end, self.map)
            if line_iter.shape[0] == 0:
                continue
            avail_x = line_iter[:, 0].astype(int)
            avail_y = line_iter[:, 1].astype(int)
            self.map[avail_y[:-1], avail_x[:-1]] += self.occu_down
            self.map[avail_y[-1], avail_x[-1]] -= self.occu_up

        # occupancy 값 안정화 (0~1 범위로)
        self.map = np.clip(self.map, 0.0, 1.0)
        # self.show_pose_and_points(pose, laser_global)

    def show_pose_and_points(self, pose, laser_global):
        map_bgr = cv2.cvtColor(self.map.astype(np.float32), cv2.COLOR_GRAY2BGR)

        pose_pixel = (
            (
                pose[:2].flatten()
                - self.map_center
                + (np.array(self.map_size) * self.map_resolution) / 2
            )
            / self.map_resolution
        ).astype(int)
        laser_pixel = (
            (
                laser_global[:2, :].T
                - self.map_center
                + (np.array(self.map_size) * self.map_resolution) / 2
            )
            / self.map_resolution
        ).astype(int)

        for l in laser_pixel:
            cv2.circle(map_bgr, tuple(l), 1, (0, 255, 0), -1)
        cv2.circle(map_bgr, tuple(pose_pixel), 2, (0, 0, 255), -1)

        map_bgr = cv2.resize(
            map_bgr,
            dsize=(0, 0),
            fx=self.map_vis_resize_scale,
            fy=self.map_vis_resize_scale,
        )
        cv2.namedWindow("Sample Map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Sample Map", 800, 800)  # 원하는 크기로
        cv2.imshow("Sample Map", map_bgr)
        cv2.waitKey(1)


class Mapper(Node):
    def __init__(self):
        super().__init__("Mapper")
        use_absolute_pose = True
        if use_absolute_pose:
            # 절대 좌표 기반 매핑
            self.sub = self.create_subscription(
                ScanWithPose,
                "/scan_with_pose",
                self.scan_with_pose_callback_absolute,
                10,
            )
        else:
            # 상대 좌표 기반 매핑
            self.odom_sub = self.create_subscription(
                Odometry, "/odom_est", self.odom_callback, 10
            )
            self.swp_sub = self.create_subscription(
                ScanWithPose,
                "/scan_with_pose",
                self.scan_with_pose_callback_relative,
                10,
            )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"
        m = MapMetaData()
        m.resolution = params_map["MAP_RESOLUTION"]
        m.width = int(params_map["MAP_SIZE"][0] / params_map["MAP_RESOLUTION"])
        m.height = int(params_map["MAP_SIZE"][1] / params_map["MAP_RESOLUTION"])
        m.origin = Pose()
        m.origin.position.x = (
            params_map["MAP_CENTER"][0] - params_map["MAP_SIZE"][0] / 2
        )
        m.origin.position.y = (
            params_map["MAP_CENTER"][1] - params_map["MAP_SIZE"][1] / 2
        )
        self.map_msg.info = m
        self.map_size = m.width * m.height

        self.latest_pose = None
        self.mapping = Mapping(params_map)
        self.get_logger().info("Mapper initialized. Waiting for scan_with_pose...")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quat = Quaternion(q.w, q.x, q.y, q.z)
        _, _, heading = quat.to_euler()
        self.latest_pose = np.array(
            [[msg.pose.pose.position.x], [msg.pose.pose.position.y], [heading]]
        )

    def scan_with_pose_callback_absolute(self, msg):
        self.get_logger().info("Received scan_with_pose. Processing...")
        pose = np.array([[msg.pose_x], [msg.pose_y], [msg.pose_theta]])

        distance = np.array(msg.ranges)
        angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
        x = distance * np.cos(angles)
        y = distance * np.sin(angles)
        laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

        self.mapping.update(pose, laser)

        self.publish_map()
        self.get_logger().info("Map updated.")

    def scan_with_pose_callback_relative(self, msg):
        if self.latest_pose is None:
            self.get_logger().warn("No odometry yet. Skipping scan.")
            return

        distance = np.array(msg.ranges)
        angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
        x = distance * np.cos(angles)
        y = distance * np.sin(angles)
        laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

        self.mapping.update(self.latest_pose, laser)

        self.publish_map()

    def publish_map(self):
        np_map_data = self.mapping.map.reshape(1, self.map_size)
        list_map_data = [
            max(0, min(100, 100 - int(val * 100))) for val in np_map_data[0]
        ]
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_msg.data = list_map_data
        self.map_pub.publish(self.map_msg)


def save_all_map(node, file_name_txt="map.txt", file_name_png="map.png"):
    # ✅ 현재 파일(run_mapping.py)의 절대 경로 기준으로 map 폴더 설정
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    folder_path = os.path.join(
        curr_dir, "..", "..", "..", "..", "map"
    )  # 경로 상대 이동

    # 경로가 없다면 생성
    os.makedirs(folder_path, exist_ok=True)

    # OccupancyGrid 데이터 -> .txt 저장
    full_txt_path = os.path.join(folder_path, file_name_txt)
    node.get_logger().info(f"Saving map data to: {full_txt_path}")
    with open(full_txt_path, "w") as f:
        data = " ".join(str(pixel) for pixel in node.map_msg.data)
        f.write(data)

    # 2D 맵 이미지 -> .png 저장
    full_png_path = os.path.join(folder_path, file_name_png)
    node.get_logger().info(f"Saving map image to: {full_png_path}")
    map_image = (node.mapping.map.copy() * 255).astype(np.uint8)
    cv2.imwrite(full_png_path, map_image)


def main(args=None):
    rclpy.init(args=args)
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected.")
    except Exception as e:
        node.get_logger().error(f"Exception occurred in main(): {e}")
    finally:
        node.get_logger().info("Saving map...")
        save_all_map(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
