import numpy as np
import cv2
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 9094,
    "Block_SIZE": int(1206),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.30,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "WIDTH": 640, # image width
    "HEIGHT": 480, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.50,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}

# ex 노드 설명
# 로봇에 달려있는 라이다와 카메라 간의 위치 및 자세 정보를 위의 params_lidar, params_cam으로
# 받은 다음, 이를 가지고 좌표 변환 행렬을 만들고, 카메라 이미지에 라이다 포인트들을 projection
# 하는 노드입니다.

# --------------------------- Transformation Utilities ---------------------------

def rotationMtx(yaw, pitch, roll):
    
    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                    
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
    
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                    
    R = np.matmul(R_z, np.matmul(R_x, R_y))

    return R

def translationMtx(x, y, z):

    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M



def transformMTX_lidar2cam(params_lidar, params_cam):
    # LiDAR 좌표계를 카메라 좌표계로 변환하기 위한 4x4 변환 행렬 생성
    """
    1. 라이다와 카메라의 위치와 자세를 받아
    2. 라이다 → 카메라로 변환되는 RT 행렬 생성
    """

    lidar_yaw   = math.radians(params_lidar["YAW"])
    lidar_pitch = math.radians(params_lidar["PITCH"])
    lidar_roll  = math.radians(params_lidar["ROLL"])

    cam_yaw   = math.radians(params_cam["YAW"])
    cam_pitch = math.radians(params_cam["PITCH"])
    cam_roll  = math.radians(params_cam["ROLL"])
    
    lidar_pos = np.array([params_lidar["X"], params_lidar["Y"], params_lidar["Z"]])
    cam_pos   = np.array([params_cam["X"], params_cam["Y"], params_cam["Z"]])

    dx, dy, dz = cam_pos - lidar_pos
    T = translationMtx(dx, dy, dz)
    R = rotationMtx(np.radians(-90),np.radians(-180),np.radians(-90))
    RT = T@R
    return np.linalg.inv(RT)

def project2img_mtx(params_cam):
    # 3D 카메라의 좌표계의 점을 2D 이미지 좌표계로 정사영

    width = params_cam["WIDTH"]
    height = params_cam["HEIGHT"]
    fov_y_deg = params_cam["FOV"]  # 🔥 수직 FOV로 해석해야 함!
    fov_y_rad = math.radians(fov_y_deg)

    # 🔥 수직 FOV 기준 focal length 계산
    fc_y = (height / 2) / math.tan(fov_y_rad / 2)
    fc_x = fc_y  # 🔥 square pixel 가정

    # 로직 2. 이미지 중심 좌표 계산
    cx = width / 2
    cy = height / 2

    # 로직 3. 2x3 프로젝션 행렬 구성
    R_f = np.array([
        [fc_x,   0,   cx],
        [0,     fc_y, cy]
    ])

    return R_f

def draw_pts_img(img, xi, yi):
    point_np = img.copy()
    for ctr in zip(xi, yi):
        ctr_int = (int(ctr[0]), int(ctr[1]))
        point_np = cv2.circle(point_np, ctr_int, 2, (255,0,0),-1)
    return point_np

# --------------------------- Main Transformation Class ---------------------------
class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)
        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p):
        xyz_h = np.hstack((xyz_p, np.ones((xyz_p.shape[0], 1))))
        return (xyz_h @ self.RT.T)[:, :3]

    def project_pts2img(self, xyz_c, crop=True):
        x, y, z = xyz_c[:, 0], xyz_c[:, 1], xyz_c[:, 2]
        mask = z > 0
        xn, yn = x[mask] / z[mask], y[mask] / z[mask]
        pts = np.stack([xn, yn, np.ones_like(xn)], axis=0)
        xy_i = (self.proj_mtx @ pts).T
        return self.crop_pts(xy_i) if crop else xy_i

    def crop_pts(self, pts):
        x, y = pts[:, 0], pts[:, 1]
        mask = (x >= 0) & (x < self.width) & (y >= 0) & (y < self.height)
        return pts[mask]
    
# --------------------------- ROS2 Node ---------------------------
class SensorCalib(Node):
    def __init__(self):
        super().__init__(node_name='ex_calib')
        self.subs_scan = self.create_subscription(LaserScan,'/scan',self.scan_callback, 10)
        self.subs_img = self.create_subscription(CompressedImage,'/image_jpeg/compressed',self.img_callback,10)
        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.xyz, self.R, self.intens = None, None, None
        self.img = None

    def img_callback(self, msg):
        # /image_jpeg/compressed 에서 수신된 이미지를 cv2 포맷을 저장 (self.img)

        # 압축 이미지 데이터를 numpy 배열로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)

        # JPEG 디코딩 → OpenCV 이미지 (BGR 채널)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
        # /scan 토픽에서 2D 거리 + 각도 데이터를 받아 3D 포인트 (x, y, z)로 변환
        self.R = np.array(msg.ranges)
        angles = np.arange(len(self.R)) * msg.angle_increment + msg.angle_min
        x, y = self.R * np.cos(angles), self.R * np.sin(angles)
        self.xyz = np.stack([x, y, np.zeros_like(x)], axis=1)
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min

    def timer_callback(self):
        if self.xyz is None or self.img is None:
            print("Waiting for messages...")
            return

        # ----- 정면 부분만 crop (90~270)-----
        angles = np.arange(len(self.R)) * self.angle_increment + self.angle_min
        angle_deg = np.degrees(angles)
        mask = (angle_deg >= 90) & (angle_deg <= 270)
        xyz_crop = self.xyz[mask]

        # ----- LiDAR → Camera 좌표계 변환 -----
        xyz_c = self.l2c_trans.transform_lidar2cam(xyz_crop)

        # ----- Camera 3D → 2D 이미지 좌표 투영 -----
        pts_2d = self.l2c_trans.project_pts2img(xyz_c)

        # ----- 이미지 위에 포인트 시각화 -----
        img_show = draw_pts_img(self.img.copy(), pts_2d[:, 0], pts_2d[:, 1])
        cv2.imshow("Lidar2Cam", img_show)
        cv2.waitKey(1)

# --------------------------- Main Entry ---------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SensorCalib()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()