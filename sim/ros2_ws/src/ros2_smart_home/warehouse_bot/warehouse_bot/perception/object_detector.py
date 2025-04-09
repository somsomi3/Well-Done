#!/ C:\Python37\python.exe
import numpy as np
import cv2
import rclpy
import os
from rclpy.node import Node
import time
from sensor_msgs.msg import CompressedImage, LaserScan
from ssafy_msgs.msg import BBox
from ssafy_msgs.msg import ClassCounts
from ultralytics import YOLO  # YOLOv8 모델 임포트
from geometry_msgs.msg import Point
from .ex_calib import *  # 라이다-카메라 캘리브레이션 유지

from geometry_msgs.msg import Point


global img_bgr
img_bgr = None #콜백 되기전.

global xyz
xyz = None  # 초기값 설정 콜백함수에서 사용하기 전


# 로직 6. object detection 클래스 생성
# 로직 7. node 및 image subscriber 생성
# 로직 8. lidar2img 좌표 변환 클래스 정의
# 로직 9. ros 통신을 통한 이미지 수신
# 로직 10. object detection model inference
# 로직 11. 라이다-카메라 좌표 변환 및 정사영
# 로직 12. bounding box 결과 좌표 뽑기
# 로직 13. 인식된 물체의 위치 추정
# 로직 14. 시각화




params_lidar = {
    "Range": 90,  # min & max range of lidar azimuths
    "CHANNEL": int(1),  # verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 9094,
    "Block_SIZE": int(1206),
    "X": 0,  # meter
    "Y": 0,
    # "Z": 0.6,
    "Z": 0.10,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0,
}


params_cam = {
    "WIDTH": 320,  # image width
    "HEIGHT": 240,  # image height
    "FOV": 60,  # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0,  # meter
    "Y": 0,
    # "Z": 1,
    "Z": 0.19,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0,
}


class detection_net_class():
        def __init__(self, model_path):
            self.model = YOLO(model_path)
            self.class_names = self.model.names  # 클래스 이름 저장
        
        def inference(self, image_np):
            t_start = time.time()
            
            # YOLOv8 추론 실행
            results = self.model(image_np, conf=0.8)
            
            # 추론 결과 처리
            image_process = results[0].plot()  # 시각화된 이미지
            
            # 결과 추출
            boxes = results[0].boxes.xyxyn.cpu().numpy()  # 정규화된 좌표 (0~1)
            scores = results[0].boxes.conf.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(np.int32)
            
            # 점수가 0.6 이상인 객체만 선택
            idx_detect = np.where(scores > 0.8)[0]
            boxes_detect = boxes[idx_detect]
            classes_pick = classes[idx_detect]
            
            infer_time = time.time() - t_start
            
            return image_process, infer_time, boxes_detect, scores[idx_detect], classes_pick


# visualize_images 함수 수정:
def visualize_images(image_out, t_cost, boxes, classes, scores, class_names):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(image_out, "YOLOv8", (30, 50), font, 1, (0, 255, 0), 2, 0)
    cv2.putText(image_out, "{:.4f}s".format(t_cost), (30, 150), font, 1, (0, 255, 0), 2, 0)
    
    # 바운딩 박스에 클래스 이름 추가
    for i in range(len(boxes)):
        if scores[i] >= 0.8: 
            x1, y1, x2, y2 = boxes[i]
            class_id = classes[i]
            class_name = class_names[class_id]
            score = scores[i]
            label = f"{class_name}: {score:.2f}"
            cv2.putText(image_out, label, (int(x1), int(y1)-10), font, 0.5, (0, 255, 0), 1)
    
    winname = "YOLOv8 Object Detection"
    cv2.imshow(winname, cv2.resize(image_out, (2 * image_out.shape[1], 2 * image_out.shape[0])))
    cv2.waitKey(1)




def img_callback(msg):

    global img_bgr
    
    np_arr = np.frombuffer(msg.data, np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


def scan_callback(msg):

    global xyz

    R = np.array(msg.ranges)

    x = R * np.cos(np.linspace(0, 2 * np.pi, 360))
    y = R * np.sin(np.linspace(0, 2 * np.pi, 360))
    z = np.zeros_like(x)

    xyz = np.concatenate(
        [x.reshape([-1, 1]), y.reshape([-1, 1]), z.reshape([-1, 1])], axis=1
    )



def main(args=None):

    # 로직 2. pretrained file and label map load
    ## 우선 스켈레톤 코드는 구글이 이미 학습시켜서 model zoo에 올린, mobilenet v1을 backbone으로 하는
    ## single shot detector 모델의 pretrained 파라메터인
    ## 'ssd_mobilenet_v1_coco_2018_01_28' 폴더 내 frozen_inference_graph.pb를 받도록 했습니다.
    ## 현재 sub3/sub3 디렉토리 안에 model_weights 폴더를 두고, 거기에 model 폴더인
    ## 'ssd_mobilenet_v1_coco_11_06_2017'와 data 폴더 내 mscoco_label_map.pbtxt를
    ## 넣어둬야 합니다    
    
    # pkg_path =os.getcwd()
    # back_folder='\ros2_ws\src\ros2_smart_home\sub3'

    # CWD_PATH = os.path.join(pkg_path, back_folder,'sub3')
    CWD_PATH = os.path.dirname(os.path.abspath(__file__))

    MODEL_PATH = os.path.join(CWD_PATH, 'warehouse_objects', 'weights','best.pt')

    # 모델 로드 및 초기화
    ssd_net = detection_net_class(MODEL_PATH)

    global g_node

    rclpy.init(args=args)

    g_node = rclpy.create_node("tf_detector")

    
    # subscription_img
    subscription_img = g_node.create_subscription(CompressedImage, '/image_jpeg/compressed', img_callback, 3)

    # subscription_scan
    subscription_scan = g_node.create_subscription(LaserScan, '/scan', scan_callback, 3)

    # 위치 데이터 퍼블리셔 생성
    position_publisher = g_node.create_publisher(Point, '/detected_object_position', 10)


    
    
    # 로직 8. lidar2img 좌표 변환 클래스 정의
    # sub2의 좌표 변환 클래스를 가져와서 정의.

    l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

    iter_step = 0

    while rclpy.ok():

        time.sleep(0.05)

        # 로직 9. ros 통신을 통한 이미지 수신
        for _ in range(2):

            rclpy.spin_once(g_node)
        if img_bgr is None:
            print("Waiting for image...")
            continue

        # 로직 10. object detection model inference
        image_process, infer_time, boxes_detect, scores, classes_pick = (
            ssd_net.inference(img_bgr)
        )

        # 로직 11. 라이다-카메라 좌표 변환 및 정사영
        # sub2 에서 ex_calib 에 했던 대로 라이다 포인트들을
        # 이미지 프레임 안에 정사영시킵니다.

        # 라이더 데이타 기다리는중
        if xyz is None:
            print("Waiting for lidar data...")
            continue
        

        # 라이다 데이터 필터링
        # xyz_p = xyz[np.where(xyz[:, 0] >= 0)]
        xyz_p = xyz
        if xyz_p.shape[0] == 0:
            print("No valid lidar points after filtering.")
            continue

        # 라이다 -> 카메라 좌표 변환 및 정사영
        xyz_c = l2c_trans.transform_lidar2cam(xyz_p)

        # z > 0 필터링을 먼저 적용
        valid_z =( xyz_c[:, 2] > 0)
        xyz_c_valid_z = xyz_c[valid_z]


        xy_i = l2c_trans.project_pts2img(xyz_c_valid_z, crop=False)
        
        # 이미지 범위 내에 있는 포인트만 필터링
        valid_img = (xy_i[:, 0] >= 0) & (xy_i[:, 0] < params_cam["WIDTH"]) & \
                (xy_i[:, 1] >= 0) & (xy_i[:, 1] < params_cam["HEIGHT"])
                
        xy_i_valid = xy_i[valid_img]
        xyz_c_valid = xyz_c_valid_z[valid_img]


        if xy_i_valid.shape[0] != xyz_c_valid.shape[0]:
            print("Array dimensions do not match. Skipping concatenation.")
            continue


        try:
            xyii = np.concatenate([xy_i_valid, xyz_c_valid], axis=1)
        except ValueError as e:
            print(f"Concatenation failed: {e}")
            continue
        

        # print(f"xyz shape: {xyz.shape}")
        # print(f"xyz_p shape after filtering: {xyz_p.shape}")
        # print(f"xyz_c shape: {xyz_c.shape}")
        # print(f"xyz_c_valid_z shape: {xyz_c_valid_z.shape}")
        # print(f"xy_i shape: {xy_i.shape}")
        # print(f"xy_i_valid shape: {xy_i_valid.shape}")
        # print(f"xyz_c_valid shape: {xyz_c_valid.shape}")

        # 로직 12. bounding box 결과 좌표 뽑기
        ## boxes_detect 안에 들어가 있는 bounding box 결과들을
        ## 좌상단 x,y와 너비 높이인 w,h 구하고, 
        ## 본래 이미지 비율에 맞춰서 integer로 만들어
        ## numpy array로 변환

        if len(boxes_detect) != 0:
            ih = img_bgr.shape[0]
            iw = img_bgr.shape[1]
            
            # YOLOv8 바운딩 박스 형식 변환
            # boxes_detect는 [x1, y1, x2, y2] 형식 (정규화된 좌표)
            x = boxes_detect[:, 0] * iw  # 좌상단 x
            y = boxes_detect[:, 1] * ih  # 좌상단 y
            w = (boxes_detect[:, 2] - boxes_detect[:, 0]) * iw  # 너비
            h = (boxes_detect[:, 3] - boxes_detect[:, 1]) * ih  # 높이
            
            bbox = np.vstack([
                x.astype(np.int32).tolist(),
                y.astype(np.int32).tolist(),
                w.astype(np.int32).tolist(),
                h.astype(np.int32).tolist()
            ]).T



            
        

            # 로직 13. 인식된 물체의 위치 추정
            ## bbox가 구해졌으면, bbox 안에 들어가는 라이다 포인트 들을 구하고
            ## 그걸로 물체의 거리를 추정할 수 있습니다.

            ostate_list = []
            for i in range(bbox.shape[0]):
                x = int(bbox[i, 0])
                y = int(bbox[i, 1])
                w = int(bbox[i, 2])
                h = int(bbox[i, 3])


                margin = 20  # 바운더리 박스 확장 마진

                # 바운더리 박스 중심 구하기
                cx = x + w // 2
                cy = y + h // 2
                radius = max(w, h) // 2+margin

                distances = np.sqrt((xyii[:, 0] - cx)**2 + (xyii[:, 1] - cy)**2)
                xyv = xyii[distances < radius]

                # # 바운더리 박스 안에 들어가는 라이다 포인트들 구하기
                
                # xyv = xyii[(xyii[:, 0] >= x - margin) & (xyii[:, 0] <= x + w + margin) &
                #         (xyii[:, 1] >= y - margin) & (xyii[:, 1] <= y + h + margin)]
                
                print(f"BBox coordinates: x={x}, y={y}, w={w}, h={h}")
                print(f"Filtered lidar points (xyv): {xyv.shape}")

                if len(xyv) > 0:
                    ostate = np.mean(xyv[:, :3], axis=0)  # Calculate mean position of points
                    if not np.isnan(ostate).any():  # NaN 값 체크
                        ostate_list.append(ostate)

                        # 위치 데이터 퍼블리시
                        msg = Point()
                        msg.x = float(ostate[0])
                        msg.y = float(ostate[1])
                        msg.z = float(ostate[2])
                        position_publisher.publish(msg)
                    else:
                        print("NaN detected in ostate. Skipping.")
                else:
                    print("No lidar points found in bounding box.")
            # Draw lidar points on the image for visualization
            # image_process = draw_pts_img(image_process, xy_i[:, 0].astype(np.int32), xy_i[:, 1].astype(np.int32))
            image_process = draw_pts_img(image_process, xy_i_valid[:, 0].astype(np.int32), xy_i_valid[:, 1].astype(np.int32))
            print(f"ostate_list: {ostate_list}")
            

            
                
            
        
        visualize_images(image_process, infer_time, boxes_detect, classes_pick, scores, ssd_net.class_names)

    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()