import rclpy
from rclpy.node import Node
import ros2pkg
from geometry_msgs.msg import Twist,PoseStamped,Pose,TransformStamped
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path,OccupancyGrid,MapMetaData
from math import pi,cos,sin,sqrt
import tf2_ros
import os
import advanced.utils as utils
import numpy as np
import cv2
import time

"""이 노드는 C:\\Users\\SSAFY\\Desktop\\temp\\S12P21E102\\sim\\ros2_ws\\src\\ros2_smart_home\\advanced 에서 실행할 것"""

params_map = {
    "MAP_RESOLUTION": 0.01,
    "OCCUPANCY_UP": 1,
    "OCCUPANCY_DOWN": 0.3,
    "MAP_CENTER": (-50.0, -50.0),
    "MAP_SIZE": (17.5, 17.5),
    "MAP_FILENAME": 'test.png',
    "MAPVIS_RESIZE_SCALE": 1.0
}

def createLineIterator(P1, P2, img):

    # Bresenham's 알고리즘을 이용하여 P1 → P2 직선상의 모든 픽셀 좌표를 구하는 함수
    imageH = img.shape[0]
    imageW = img.shape[1]
    P1Y = P1[1]
    P1X = P1[0]
    P2X = P2[0]
    P2Y = P2[1]

    # 로직 1: 두 점 사이의 거리 및 방향 계산
    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)

    # 로직 2: 직선상 픽셀 좌표를 저장할 버퍼 생성
    itbuffer = np.empty(shape=(np.maximum(dYa,dXa), 3), dtype=np.float32)
    itbuffer.fill(np.nan)

    # 로직 3: 방향 판별 (음수 방향인지 체크)
    negY = P1Y > P2Y
    negX = P1X > P2X
    
    # 로직 4: 수직선인 경우
    if P1X == P2X:
        itbuffer[:, 0] = P1X
        if negY:
            itbuffer[:, 1] = np.arange(P1Y - 1, P1Y - dYa - 1, -1)
        else:
            itbuffer[:, 1] = np.arange(P1Y + 1, P1Y + dYa + 1)

    # 로직 5: 수평선인 경우
    elif P1Y == P2Y:
        itbuffer[:, 1] = P1Y
        if negX:
            itbuffer[:, 0] = np.arange(P1X - 1, P1X - dXa - 1, -1)
        else:
            itbuffer[:, 0] = np.arange(P1X + 1, P1X + dXa + 1)

    # 로직 6: 대각선인 경우
    else:
        steepSlope = dYa > dXa
        if steepSlope: # 기울기가 1보다 큰 경우
            slope = dX.astype(np.float32) / dY.astype(np.float32)
            if negY:
                itbuffer[:, 1] = np.arange(P1Y - 1, P1Y - dYa - 1, -1)
            else:
                itbuffer[:, 1] = np.arange(P1Y + 1, P1Y + dYa + 1)
            itbuffer[:, 0] = (slope * (itbuffer[:, 1] - P1Y)).astype(np.int) + P1X
        else: # 기울기가 1보다 작은 경우
            slope = dY.astype(np.float32) / dX.astype(np.float32)
            if negX:
                itbuffer[:, 0] = np.arange(P1X - 1, P1X - dXa - 1, -1)
            else:
                itbuffer[:, 0] = np.arange(P1X + 1, P1X + dXa + 1)
            itbuffer[:, 1] = (slope * (itbuffer[:, 0] - P1X)).astype(np.int) + P1Y

    # 로직 7: 이미지 바깥 좌표 제거
    colX = itbuffer[:, 0]
    colY = itbuffer[:, 1]
    itbuffer = itbuffer[(colX >= 0) & (colY >= 0) & (colX < imageW) & (colY < imageH)]

    # 픽셀 강도 저장 (나중에 사용될 가능성에 대비)
    itbuffer[:, 2] = img[itbuffer[:, 1].astype(np.uint), itbuffer[:, 0].astype(np.uint)]

    return itbuffer


class Mapping:

    def __init__(self, params_map):

        # 로직 3. 맵의 resolution, 중심좌표, occupancy에 대한 threshold 등의 설정들을 받습니다
        self.map_resolution = params_map["MAP_RESOLUTION"]
        self.map_size = np.array(params_map["MAP_SIZE"]) / self.map_resolution
        self.map_center = params_map["MAP_CENTER"]
        self.map = np.ones((self.map_size[0].astype(np.int), self.map_size[1].astype(np.int)))*0.5
        self.occu_up = params_map["OCCUPANCY_UP"]
        self.occu_down = params_map["OCCUPANCY_DOWN"]

        self.map_filename = params_map["MAP_FILENAME"]
        self.map_vis_resize_scale = params_map["MAPVIS_RESIZE_SCALE"]

        self.T_r_l = np.array([[0,-1,0],[1,0,0],[0,0,1]])

    def update(self, pose, laser):

        # 로직 7. pose 값을 받아서 좌표변환 행렬로 정의
        n_points = laser.shape[1]
        pose_mat = utils.xyh2mat2D(pose)

        # 로직 8. laser scan 데이터 좌표 변환
        pose_mat = np.matmul(pose_mat,self.T_r_l)
        laser_mat = np.ones((3, n_points))
        laser_mat[:2, :] = laser

        laser_global = np.matmul(pose_mat, laser_mat)

        # 로직 9. pose와 laser의 grid map index 변환
        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution
        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y = (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        # 로직 10. laser scan 공간을 맵에 표시
        for i in range(laser_global.shape[1]):
            p1 = np.array([pose_x, pose_y]).reshape(-1).astype(np.int)
            p2 = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
        
            line_iter = createLineIterator(p1, p2, self.map)
        
            if (line_iter.shape[0] is 0):
                continue
        
            avail_x = line_iter[:, 0].astype(np.int)
            avail_y = line_iter[:, 1].astype(np.int)
        
            # Empty (beam 경로 상의 셀)
            self.map[avail_y[:-1], avail_x[:-1]] += self.occu_down
        
            # Occupied (최종 충돌 지점)
            self.map[avail_y[-1], avail_x[-1]] -= self.occu_up

        # occupancy 값 안정화 (0~1 범위로)
        self.map = np.clip(self.map, 0.0, 1.0)
        
        self.show_pose_and_points(pose, laser_global)

    def show_pose_and_points(self, pose, laser_global):
        tmp_map = self.map.astype(np.float32)
        map_bgr = cv2.cvtColor(tmp_map, cv2.COLOR_GRAY2BGR)

        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y =  (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        for i in range(laser_global.shape[1]):
            (l_x, l_y) = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
            center = (l_x, l_y)
            cv2.circle(map_bgr, center, 1, (0,255,0), -1)

        center = (pose_x.astype(np.int32)[0], pose_y.astype(np.int32)[0])
        
        cv2.circle(map_bgr, center, 2, (0,0,255), -1)

        map_bgr = cv2.resize(map_bgr, dsize=(0, 0), fx=self.map_vis_resize_scale, fy=self.map_vis_resize_scale)
        
        cv2.namedWindow("Sample Map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Sample Map", 800, 800)  # 원하는 크기로
        cv2.imshow('Sample Map', map_bgr)
        cv2.waitKey(1)



        
class Mapper(Node):

    def __init__(self):
        super().__init__('Mapper')
        
        # 로직 1 : publisher, subscriber, msg 생성
        self.subscription = self.create_subscription(LaserScan, '/scan',self.scan_callback,10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        
        self.map_msg=OccupancyGrid()
        self.map_msg.header.frame_id="map"
        self.map_size=int(params_map["MAP_SIZE"][0]\
            /params_map["MAP_RESOLUTION"]*params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        
        m = MapMetaData()
        m.resolution = params_map["MAP_RESOLUTION"]
        m.width = int(params_map["MAP_SIZE"][0]/params_map["MAP_RESOLUTION"])
        m.height = int(params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        quat = np.array([0, 0, 0, 1])
        m.origin = Pose()
        m.origin.position.x = params_map["MAP_CENTER"][0] - params_map["MAP_SIZE"][0]/2
        m.origin.position.y = params_map["MAP_CENTER"][1] - params_map["MAP_SIZE"][1]/2
        self.map_meta_data = m

        self.map_msg.info=self.map_meta_data
        # 로직 2 : mapping 클래스 생성
        self.mapping = Mapping(params_map)


    def scan_callback(self,msg):
        
        pose_x=msg.range_min
        pose_y=msg.scan_time
        heading=msg.time_increment
        Distance=np.array(msg.ranges)

        # 각도 범위를 -π ~ π로 설정해서 정면 = 180도 방향 반영
        angles = np.linspace(-np.pi, np.pi, 360)

        # 정면 기준 각도로 라이다 포인트 계산
        x = Distance * np.cos(angles)
        y = Distance * np.sin(angles)
        laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

        pose = np.array([[pose_x],[pose_y],[heading]])
        self.mapping.update(pose, laser)

        np_map_data=self.mapping.map.reshape(1,self.map_size) 
        list_map_data=np_map_data.tolist()
        for i in range(self.map_size):
            list_map_data[0][i]=100-int(list_map_data[0][i]*100)
            if list_map_data[0][i] >100 :
                list_map_data[0][i]=100

            if list_map_data[0][i] <0 :
                list_map_data[0][i]=0

        self.map_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.map_msg.data=list_map_data[0]
        self.map_pub.publish(self.map_msg)

def save_all_map(node, file_name_txt='map.txt', file_name_png='map.png'):

    # 로직 12 : 맵 저장
    pkg_path =os.getcwd()
    back_folder='..'
    folder_name='map'
    folder_path=os.path.join(pkg_path,back_folder,folder_name)

    # 경로가 없다면 생성
    os.makedirs(folder_path, exist_ok=True)

    # OccupancyGrid 데이터 -> .txt 저장
    full_txt_path = os.path.join(folder_path, file_name_txt)
    print(f"Saving map data to: {full_txt_path}")
    with open(full_txt_path, 'w') as f:
        data = ' '.join(str(pixel) for pixel in node.map_msg.data)
        f.write(data)

    # 2D 맵 이미지 -> .png 저장
    full_png_path = os.path.join(folder_path, file_name_png)
    print(f"Saving map image to: {full_png_path}")
    map_image = (node.mapping.map.copy() * 255).astype(np.uint8)
    cv2.imwrite(full_png_path, map_image)
        
def main(args=None):    
    rclpy.init(args=args)
    
    try :    
        node = Mapper()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except :
        save_all_map(node, 'map.txt', 'map.png')


if __name__ == '__main__':
    main()