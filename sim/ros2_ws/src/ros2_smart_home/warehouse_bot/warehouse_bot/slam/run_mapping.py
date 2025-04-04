import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from math import pi, cos, sin
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from ssafy_msgs.msg import ScanWithPose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from squaternion import Quaternion
import warehouse_bot.slam.utils as utils
from warehouse_bot.utils.sim_config import params_map, MAP_PATH
from warehouse_bot.utils.logger_utils import print_log


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
    def __init__(self, params, reset_map=True, logger=None):
        self.logger = logger
        self.map_resolution = params["MAP_RESOLUTION"]
        self.map_size = (np.array(params["MAP_SIZE"]) / self.map_resolution).astype(int)
        self.map_center = params["MAP_CENTER"]
        self.map_filename = params["MAP_FILENAME"]
        self.map_vis_resize_scale = params["MAPVIS_RESIZE_SCALE"]
        self.occu_up = params["OCCUPANCY_UP"]
        self.occu_down = params["OCCUPANCY_DOWN"]
        self.T_r_l = np.array(
            [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        )  # 반시계 90도 회전 보정

        if not reset_map:
            if self.load_map():
                print_log(
                    "info", self.logger, "Loaded previous map.", file_tag="mapping"
                )
            else:
                print_log(
                    "warn",
                    self.logger,
                    "Failed to load previous map. Creating new map.",
                    file_tag="mapping",
                )
                self.map = np.ones(self.map_size) * 0.5
        else:
            print_log(
                "info",
                self.logger,
                "Creating new map from scratch.",
                file_tag="mapping",
            )
            self.map = np.ones(self.map_size) * 0.5

    def load_map(self):
        txt_path = os.path.join(MAP_PATH, self.map_filename + ".txt")
        if not os.path.exists(txt_path):
            return False

        try:
            with open(txt_path, "r") as f:
                data = list(map(int, f.read().split()))
                self.map = 1.0 - np.array(data).reshape(self.map_size) / 100.0
                self.map = np.clip(self.map, 0.0, 1.0)
            return True
        except Exception as e:
            if self.logger:
                print_log(
                    "error",
                    self.logger,
                    f"[MAP] Failed to load map: {e}",
                    file_tag="mapping",
                )
            return False

    def update(self, pose, laser):
        n_points = laser.shape[1]
        pose_mat = utils.xyh2mat2D(pose) @ self.T_r_l
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

        num_skipped = 0
        for i, end in enumerate(laser_grid):
            dist = np.linalg.norm(laser[:, i])
            if dist >= 10:  # 예: 라이다 최대 사거리보다 크면 무시
                num_skipped += 1
                continue

            line_iter = createLineIterator(pose_grid, end, self.map)
            if line_iter.shape[0] == 0:
                continue
            avail_x = line_iter[:, 0].astype(int)
            avail_y = line_iter[:, 1].astype(int)
            self.map[avail_y[:-1], avail_x[:-1]] += self.occu_down
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    px = avail_x[-1] + dx
                    py = avail_y[-1] + dy
                    if 0 <= px < self.map.shape[1] and 0 <= py < self.map.shape[0]:
                        self.map[py, px] -= self.occu_up
        if self.logger:
            print_log(
                "info",
                self.logger,
                f"Skipped {num_skipped}/{laser.shape[1]} laser points",
                file_tag="mapping",
            )
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
    def __init__(self, reset_map=True):
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
        self.reset_sub = self.create_subscription(
            Bool, "/reset_mapping", self.reset_callback, 1
        )

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self.map_inflated_pub = self.create_publisher(OccupancyGrid, "/map_inflated", 1)

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
        self.mapping = Mapping(
            params_map, reset_map=reset_map, logger=self.get_logger()
        )

        self.explored_mask = np.zeros(self.mapping.map.shape, dtype=bool)

        print_log(
            "info",
            self.get_logger(),
            "Mapper initialized. Waiting for scan_with_pose...",
            file_tag="mapper",
        )

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quat = Quaternion(q.w, q.x, q.y, q.z)
        _, _, heading = quat.to_euler()
        self.latest_pose = np.array(
            [[msg.pose.pose.position.x], [msg.pose.pose.position.y], [heading]]
        )

    def scan_with_pose_callback_absolute(self, msg):
        print_log(
            "info",
            self.get_logger(),
            "Received scan_with_pose. Processing...",
            file_tag="mapper",
        )
        pose = np.array([[msg.pose_x], [msg.pose_y], [msg.pose_theta]])

        distance = np.array(msg.ranges)
        angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
        x = distance * np.cos(angles)
        y = distance * np.sin(angles)
        laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

        self.mapping.update(pose, laser)

        self.publish_map()
        print_log("info", self.get_logger(), "Map updated.", file_tag="mapper")

    def scan_with_pose_callback_relative(self, msg):
        if self.latest_pose is None:
            print_log(
                "warn",
                self.get_logger(),
                "No odometry yet. Skipping scan.",
                file_tag="mapper",
            )
            return

        distance = np.array(msg.ranges)
        angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
        x = distance * np.cos(angles)
        y = distance * np.sin(angles)
        laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

        self.mapping.update(self.latest_pose, laser)

        self.publish_map()

    def reset_callback(self, msg):
        if msg.data:
            print_log(
                "warn",
                self.get_logger(),
                "🧼 Resetting the map by external request.",
                file_tag="mapper",
            )
            self.explored_mask = np.zeros(self.mapping.map.shape, dtype=bool)
            self.mapping = Mapping(params_map, reset_map=True, logger=self.get_logger())

    def publish_map(self):
        # start = self.get_clock().now().nanoseconds / 1e9
        # self.get_logger().info("Starting map publish...")

        
        np_map = self.mapping.map
        np_map_data = np_map.reshape(1, self.map_size)

        # 현재 업데이트된 셀은 explored로 표시
        self.explored_mask |= (np_map != 0.5)  # 0.5는 미개척 상태 (float)

        list_map_data = []
        for idx, val in enumerate(np_map_data[0]):
            y = idx // self.map_msg.info.width
            x = idx % self.map_msg.info.width

            if not self.explored_mask[y, x]:
                list_map_data.append(-1)  # 아직도 미개척
            elif val >= 1.0:
                list_map_data.append(0)   # 자유 공간
            elif val <= 0.0:
                list_map_data.append(100) # 장애물
            else:
                list_map_data.append(0)   # 기타는 자유 공간 처리

        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_msg.data = list_map_data
        self.map_pub.publish(self.map_msg)

        # inflated map (경로 생성용)
        np_int_map = np.array(list_map_data).reshape(self.mapping.map.shape)
        inflated = utils.inflate_map(np_int_map, 5)

        # map_inflated 생성 및 퍼블리시 (경로용)
        inflated_msg = OccupancyGrid()
        inflated_msg.header = self.map_msg.header
        inflated_msg.info = self.map_msg.info
        inflated_msg.data = inflated.flatten().tolist()
        self.map_inflated_pub.publish(inflated_msg)

        # end = self.get_clock().now().nanoseconds / 1e9
        # self.get_logger().info(f"Map published in {end - start:.3f} seconds.")


def save_all_map(node, file_name_txt="map.txt", file_name_png="map.png"):
    # 경로가 없다면 생성
    os.makedirs(MAP_PATH, exist_ok=True)

    # OccupancyGrid 데이터 -> .txt 저장
    full_txt_path = os.path.join(MAP_PATH, file_name_txt)
    print_log(
        "info",
        node.get_logger(),
        f"Saving map data to: {full_txt_path}",
        file_tag="mapper",
    )
    with open(full_txt_path, "w") as f:
        data = " ".join(str(pixel) for pixel in node.map_msg.data)
        f.write(data)

    # 2D 맵 이미지 -> .png 저장
    full_png_path = os.path.join(MAP_PATH, file_name_png)
    print_log(
        "info",
        node.get_logger(),
        f"Saving map image to: {full_png_path}",
        file_tag="mapper",
    )

    # 기본 float 맵 가져오기 (0~1 사이)
    np_map = node.mapping.map.copy()

    # OccupancyGrid 기준으로 -1인 곳은 회색으로 표시
    data_np = np.array(node.map_msg.data).reshape(np_map.shape)
    np_map[data_np == -1] = 0.5  # 회색 (미탐색)

    map_image = (np_map * 255).astype(np.uint8)
    cv2.imwrite(full_png_path, map_image)


def main(args=None):
    rclpy.init(args=args)

    # True: 맵 초기화 / False: 기존 맵 이어서 갱신
    node = Mapper(reset_map=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print_log(
            "info", node.get_logger(), "KeyboardInterrupt detected.", file_tag="mapper"
        )
    except Exception as e:
        print_log(
            "error",
            node.get_logger(),
            f"Exception occurred in main(): {e}",
            file_tag="mapper",
        )
    finally:
        print_log("info", node.get_logger(), "Saving map...", file_tag="mapper")
        save_all_map(
            node,
            file_name_txt=params_map["MAP_FILENAME"] + ".txt",
            file_name_png=params_map["MAP_FILENAME"] + ".png",
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
