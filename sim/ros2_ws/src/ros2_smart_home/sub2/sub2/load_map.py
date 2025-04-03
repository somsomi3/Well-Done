import rclpy
import numpy as np
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData


class loadMap(Node):
    """
    load_map 노드는 map.txt에 저장된 맵 데이터를 OccupancyGrid 메시지로 가공하여
    ROS2의 'map' 토픽에 publish합니다.

    - 맵 크기: 350 x 350 셀
    - 해상도: 0.05m per 셀
    - offset: 맵 기준 좌표계(map frame)와의 상대 위치
    """

    def __init__(self):
        super().__init__("load_map")
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1)

        time_period = 1
        self.timer = self.create_timer(time_period, self.timer_callback)

        # 맵 메타데이터 설정
        self.map_size_x = 350
        self.map_size_y = 350
        self.map_resolution = 0.05
        self.map_offset_x = -8 - 8.75
        self.map_offset_y = -4 - 8.75

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"

        # Map 메타데이터 설정
        map_meta = MapMetaData()
        map_meta.resolution = self.map_resolution
        map_meta.width = self.map_size_x
        map_meta.height = self.map_size_y
        map_meta.origin = Pose()
        map_meta.origin.position.x = self.map_offset_x
        map_meta.origin.position.y = self.map_offset_y
        self.map_msg.info = map_meta

        # map.txt 파일 읽기
        package_path = get_package_share_directory("sub2")
        full_path = os.path.join(package_path, "map", "map.txt")
        with open(full_path, "r") as f:
            line_data = f.readline().split()
        map_data = [int(val) for val in line_data]

        # 2차원 grid 변환
        grid = np.reshape(np.array(map_data), (350, 350))
        for y in range(350):
            for x in range(350):
                if grid[x][y] == 100:
                    for dy in range(-3, 4):
                        for dx in range(-3, 4):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < 350 and 0 <= ny < 350:
                                if grid[nx][ny] != 100:  # 이미 장애물은 제외
                                    grid[nx][ny] = 80  # 장애물 근처 처리

        self.map_msg.data = grid.reshape(-1).tolist()
        self.get_logger().info("Map data loaded.")

    def timer_callback(self):
        self.map_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = loadMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
