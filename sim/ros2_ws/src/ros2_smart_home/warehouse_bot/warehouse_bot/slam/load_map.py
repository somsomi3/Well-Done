import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from warehouse_bot.utils.sim_config import params_map, MAP_PATH
import os


class loadMap(Node):
    """
    저장된 map.txt를 OccupancyGrid로 변환하여 /map 토픽으로 퍼블리시하는 노드
    """

    def __init__(self):
        super().__init__("load_map")
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1)

        self.timer = self.create_timer(1.0, self.timer_callback)

        # --- 맵 메타데이터 설정 ---
        resolution = params_map["MAP_RESOLUTION"]
        width = int(params_map["MAP_SIZE"][0] / resolution)
        height = int(params_map["MAP_SIZE"][1] / resolution)
        center_x, center_y = params_map["MAP_CENTER"]
        offset_x = center_x - params_map["MAP_SIZE"][0] / 2
        offset_y = center_y - params_map["MAP_SIZE"][1] / 2

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"

        # Map 메타데이터 설정
        map_meta = MapMetaData()
        map_meta.resolution = resolution
        map_meta.width = width
        map_meta.height = height
        map_meta.origin = Pose()
        map_meta.origin.position.x = offset_x
        map_meta.origin.position.y = offset_y
        self.map_msg.info = map_meta

        # --- map.txt 읽기 ---
        map_filename = params_map["MAP_FILENAME"] + ".txt"
        map_path = os.path.join(MAP_PATH, map_filename)
        if not os.path.exists(map_path):
            self.get_logger().error(f"Map file not found: {map_path}")
            return

        with open(map_path, "r") as f:
            data = list(map(int, f.read().split()))

        if len(data) != width * height:
            self.get_logger().error(
                f"Map size mismatch: expected {width*height}, got {len(data)}"
            )
            return
        grid = np.array(data).reshape((height, width))

        # --- 장애물 근처 확장 처리 ---
        for y in range(height):
            for x in range(width):
                if grid[y][x] == 100:
                    for dy in range(-3, 4):
                        for dx in range(-3, 4):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                if grid[ny][nx] != 100:
                                    grid[ny][nx] = 80

        self.map_msg.data = grid.reshape(-1).tolist()
        self.get_logger().info(f"Map loaded from {map_path}")

    def timer_callback(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = loadMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
