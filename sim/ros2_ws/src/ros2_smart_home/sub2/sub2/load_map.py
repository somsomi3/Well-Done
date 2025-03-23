import rclpy
import numpy as np
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Pose
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData
from math import pi

class loadMap(Node):
    """
    load_map 노드는 map.txt 파일에 저장된 맵 데이터를 OccupancyGrid 형식으로 가공하여
    ROS2의 'map' 토픽으로 publish 하는 역할을 합니다.

    - 맵 크기: 350 x 350 셀
    - 해상도: 1셀당 0.05m
    - offset: 지도 기준 좌표계 (map 프레임)와의 상대 위치
    - 처리 과정:
        1. 맵 파일 읽기
        2. 2차원 grid로 변환
        3. 장애물 주변(5x5 범위)을 127로 마킹하여 경로 탐색 시 회피 유도
    """

    def __init__(self):
        super().__init__('load_map')
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        
        time_period = 1  
        self.timer = self.create_timer(time_period, self.timer_callback)

        # 맵 메타데이터 설정
        self.map_size_x=350 
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-8-8.75
        self.map_offset_y=-4-8.75

        self.map_msg=OccupancyGrid()
        self.map_msg.header.frame_id="map"

        map_meta = MapMetaData()
        map_meta.resolution = self.map_resolution
        map_meta.width = self.map_size_x
        map_meta.height = self.map_size_y
        map_meta.origin = Pose()
        map_meta.origin.position.x = self.map_offset_x
        map_meta.origin.position.y = self.map_offset_y
        self.map_msg.info=map_meta
        
        # map.txt 읽기
        package_path = get_package_share_directory('sub2')
        full_path = os.path.join(package_path, 'map', 'map.txt')
        with open(full_path, 'r') as f:
            line_data = f.readline().split()
        map_data = [int(val) for val in line_data]

        # 2차원 grid 변환
        grid = np.reshape(np.array(map_data), (350, 350))

        # 장애물 주변 5x5 범위 필터링 (127로 마킹)
        for y in range(350):
            for x in range(350):
                if grid[x][y]==100 :
                    for dy in range(-2, 3):
                        for dx in range(-2, 3):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < 350 and 0 <= ny < 350:
                                if grid[nx][ny] != 100:  # 이미 장애물은 제외
                                    grid[nx][ny] = 127  # 장애물 근처 처리

        # OccupancyGrid에 넣을 리스트 데이터로 변환
        self.map_msg.data = grid.reshape(-1).tolist()
        print('read_complete')


    def timer_callback(self):
        self.map_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    load_map = loadMap()
    rclpy.spin(load_map)
    load_map.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()