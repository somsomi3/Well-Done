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
    "localPort": 2368,
    "Block_SIZE": int(1206),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.4+0.1,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0., # meter
    "Y": 0,
    "Z":  0.8,
    "YAW": 0, # deg
    "PITCH": 0.0,
    "ROLL": 0
}

# ex 노드 설명
# 로봇에 달려있는 라이다와 카메라 간의 위치 및 자세 정보를 위의 params_lidar, params_cam으로
# 받은 다음, 이를 가지고 좌표 변환 행렬을 만들고, 카메라 이미지에 라이다 포인트들을 projection
# 하는 노드입니다.
# 2d 공간만 표현되는 카메라는 3d 위치정보를 포함하지 않기 때문에,
# 라이다의 포인트들을 프레임에 정사영시켜, 카메라 내 객체들의 위치 정보를 추정하도록 만들 수
# 있습니다.

# 노드 로직 순서
# 1. 노드에 필요한 라이다와 카메라 topic의 subscriber 생성
# 2. Params를 받아서 라이다 포인트를 카메라 이미지에 projection 하는 transformation class 정의하기
# 3. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장.
# 4. 라이다 콜백함수에서 2d scan data(거리와 각도)를 가지고 x,y 좌표계로 변환
# 5. 라이다 x,y 좌표 데이터 중 정면 부분만 crop
# 6. transformation class 의 transform_lidar2cam로 라이다 포인트를 카메라 3d좌표로 변환
# 7. transformation class 의 project_pts2img 로 라이다 포인트를 2d 픽셀 좌표상으로 정사영
# 8. draw_pts_img()로 카메라 이미지에 라이다 포인트를 draw 하고 show


# 좌표변환을 하는데 필요한 rotation, translation 행렬을 아래와 같이 완성시켜 놓았습니다. 
# 이를 활용하여 라이다 scan 포인트들을 이미지 프레임 상으로 변환시켜주는 클래스인 
# LIDAR2CAMTransform 를 완성시키십시오.

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
                    
    R = np.matmul(R_x, np.matmul(R_y, R_z))

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
    """
    transformMTX_lidar2cam 내 좌표 변환행렬 로직 순서
    1. params에서 라이다와 카메라 센서들의 자세, 위치 정보를 뽑기.
    2. 라이다에서 카메라 위치까지 변환하는 translation 행렬을 정의
    3. 카메라의 자세로 맞춰주는 rotation 행렬을 정의.
    4. 위의 두 행렬을 가지고 최종 라이다-카메라 변환 행렬을 정의.
    """

    # 로직 1. 위치와 자세 추출 (deg → rad 변환 포함)

    lidar_yaw   = math.radians(params_lidar["YAW"])
    lidar_pitch = math.radians(params_lidar["PITCH"])
    lidar_roll  = math.radians(params_lidar["ROLL"])

    cam_yaw   = math.radians(params_cam["YAW"])
    cam_pitch = math.radians(params_cam["PITCH"])
    cam_roll  = math.radians(params_cam["ROLL"])
    
    lidar_pos = np.array([params_lidar["X"], params_lidar["Y"], params_lidar["Z"]])
    cam_pos   = np.array([params_cam["X"], params_cam["Y"], params_cam["Z"]])

    # 로직 2. 라이다에서 카메라까지의 상대 이동 벡터
    delta_pos = cam_pos - lidar_pos
    Tmtx = translationMtx(delta_pos[0], delta_pos[1], delta_pos[2])

    # 로직 3. 카메라 좌표계로 맞춰주는 회전 행렬
    Rmtx = rotationMtx(cam_yaw, cam_pitch, cam_roll)

    # 로직 4. 최종 RT 행렬 (회전 후 이동)
    RT = np.matmul(Rmtx, Tmtx)

    return RT
    """
    테스트

    params_lidar = {
        "X": 0, # meter
        "Y": 0,
        "Z": 0.6,
        "YAW": 0, # deg
        "PITCH": 0,
        "ROLL": 0
    }


    params_cam = {
        "WIDTH": 640, # image width
        "HEIGHT": 480, # image height
        "FOV": 90, # Field of view
        "X": 0., # meter
        "Y": 0,
        "Z":  1.0,
        "YAW": 0, # deg
        "PITCH": 0.0,
        "ROLL": 0
    }

    이면

    R_T = 
    [[ 6.12323400e-17 -1.00000000e+00  0.00000000e+00  0.00000000e+00]
    [ 6.12323400e-17  3.74939946e-33 -1.00000000e+00  4.00000000e-01]
    [ 1.00000000e+00  6.12323400e-17  6.12323400e-17 -2.44929360e-17]
    [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

    """


def project2img_mtx(params_cam):
    # 3D 카메라의 좌표계의 점을 2D 이미지 좌표계로 정사영
    """
    1. 카메라 FOV와 해상도를 사용해 focal length 계산
    2. 중심 좌표(cx, cy) 계산
    3. projection 행렬 반환 (2x3)
    """

    # 로직 1. FOV를 통해 focal length 계산 (라디안으로 변환)
    width = params_cam["WIDTH"]
    height = params_cam["HEIGHT"]
    fov_deg = params_cam["FOV"]
    fov_rad = math.radians(fov_deg)

    fc_x = (width / 2) / math.tan(fov_rad / 2)
    fc_y = fc_x  # 보통 square pixel 가정


    # 로직 2. 이미지 중심 좌표 계산
    cx = width / 2
    cy = height / 2

    # 로직 3. 2x3 프로젝션 행렬 구성
    R_f = np.array([
        [fc_x,   0,   cx],
        [0,     fc_y, cy]
    ])

    return R_f
    
    """
    테스트

    params_cam = {
        "WIDTH": 320, # image width
        "HEIGHT": 240, # image height
        "FOV": 60, # Field of view
        "X": 0., # meter
        "Y": 0,
        "Z":  1.0,
        "YAW": 0, # deg
        "PITCH": 0.0,
        "ROLL": 0
    }

    이면

    R_f = 
    [[207.84609691   0.         160.        ]
    [  0.         207.84609691 120.        ]]
    """

    return np.zeros((2,3))


def draw_pts_img(img, xi, yi):

    point_np = img

    #Left Lane
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)

    return point_np


class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):

        """

        LIDAR2CAMTransform 정의 및 기능 로직 순서
        1. Params를 입력으로 받아서 필요한 파라메터들과 RT 행렬, projection 행렬 등을 정의. 
        2. 클래스 내 self.RT로 라이다 포인트들을 카메라 좌표계로 변환.
        3. RT로 좌표 변환된 포인트들의 normalizing plane 상의 위치를 계산. 
        4. normalizing plane 상의 라이다 포인트들에 proj_mtx를 곱해 픽셀 좌표값 계산.
        5. 이미지 프레임 밖을 벗어나는 포인트들을 crop.
        """
        
        # 로직 1. Params에서 필요한 파라메터들과 RT 행렬, projection 행렬 등을 정의
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p):
        """
        LiDAR 좌표계의 포인트들을 카메라 좌표계로 변환
        """
        # Step 1. (N, 3) → (N, 4) homogeneous 좌표로 확장
        ones = np.ones((xyz_p.shape[0], 1))
        xyz_homogeneous = np.concatenate([xyz_p, ones], axis=1)  # (N, 4)
        
        # Step 2. RT 행렬 곱셈: (N x 4) @ (4 x 4).T → (N x 4)
        xyz_cam_hom = xyz_homogeneous @ self.RT.T  # (N, 4)

        # Step 3. 상위 3개 값만 추출하여 (N, 3) 형태로 반환
        xyz_c = xyz_cam_hom[:, :3]

        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):
        """
        카메라 좌표계의 3D 포인트들을 2D 이미지 픽셀 좌표로 정사영
        """
        x = xyz_c[:, 0]
        y = xyz_c[:, 1]
        z = xyz_c[:, 2]

        # 로직 3. 정규화된 평면으로 투영 (카메라 프레임 기준)
        # 이미지 평면에 맺히는 위치 (Normalized Plane)
        xn = x / z
        yn = y / z

        # (3, N) 형태로 만들어서 projection 행렬과 곱함
        ones = np.ones_like(xn)
        normalized_pts = np.stack([xn, yn, ones], axis=0)  # shape: (3, N)
        
        # 로직 4. projection matrix 곱 (2x3 @ 3xN → 2xN → (N, 2).T)
        xy_i = self.proj_mtx @ normalized_pts  # shape: (2, N)
        xy_i = xy_i.T  # shape: (N, 2)

        # 로직 5. 이미지 프레임 밖 좌표 제거 (옵션)
        if crop:
            xy_i = self.crop_pts(xy_i)

        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi

class SensorCalib(Node):

    def __init__(self):
        super().__init__(node_name='ex_calib')

        # 로직 1. 노드에 필요한 라이다와 카메라 topic의 subscriber 생성

        self.subs_scan = self.create_subscription(LaserScan,'/scan',self.scan_callback, 10)
        self.subs_img = self.create_subscription(CompressedImage,'/image_jpeg/compressed',self.img_callback,10)

        # 로직 2. Params를 받아서 라이다 포인트를 카메라 이미지에 projection 하는
        # transformation class 정의하기

        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.xyz, self.R, self.intens = None, None, None
        self.img = None

    def img_callback(self, msg):
        # /image_jpeg/compressed 에서 수신된 이미지를 cv2 포맷을 저장 (self.img)
        """
        로직 3. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        """
        # 압축 이미지 데이터를 numpy 배열로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)

        # JPEG 디코딩 → OpenCV 이미지 (BGR 채널)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
        # /scan 토픽에서 2D 거리 + 각도 데이터를 받아 3D 포인트 (x, y, z)로 변환

        # 1. 거리 데이터 및 각도 생성
        self.R = np.array(msg.ranges)
        angles = np.arange(len(self.R)) * msg.angle_increment + msg.angle_min

        # 2. 라이다 기준 좌표로 변환
        x = self.R * np.cos(angles)
        y = self.R * np.sin(angles)
        z = np.zeros_like(x)

        # reshape([-1,1])은 1열로 만들기 위한 것 (행은 자동)
        self.xyz = np.concatenate([
            x.reshape([-1, 1]),
            y.reshape([-1, 1]),
            z.reshape([-1, 1])
        ], axis=1)

    def timer_callback(self):
        if self.xyz is not None and self.img is not None :

            # ----- 로직 5: 정면 부분만 crop -----
            angle_rad = np.arange(len(self.R)) * msg.angle_increment + msg.angle_min
            angle_deg = np.degrees(angle_rad)
            front_mask = np.logical_and(angle_deg >= -45, angle_deg <= 45)
            xyz_p = self.xyz[front_mask]

            # ----- 로직 6: LiDAR → Camera 좌표계 변환 -----
            xyz_c = self.l2c_trans.transform_lidar2cam(xyz_p)

            # ----- 로직 7: Camera 3D → 2D 이미지 좌표 투영 -----
            xy_i = self.l2c_trans.project_pts2img(xyz_c)

            # ----- 로직 8: 이미지 위에 포인트 시각화 -----
            img_l2c = draw_pts_img(self.img, xy_i[:, 0], xy_i[:, 1])
            cv2.imshow("Lidar2Cam", img_l2c)
            cv2.waitKey(1)
        else:
            print("waiting for msg")
            pass


def main(args=None):

    rclpy.init(args=args)

    calibrator = SensorCalib()

    rclpy.spin(calibrator)


if __name__ == '__main__':

    main()