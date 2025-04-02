#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

# IMGParser 노드는 터틀봇의 카메라로부터 받은 이미지를 처리하는 역할을 함
# - ROS2의 CompressedImage 메시지를 수신하여 OpenCV에서 처리 가능한 형식으로 변환
# - 원본 BGR 이미지를 출력
# - 그레이스케일 변환 후 출력
# - 이미지 크기를 조정하여 출력


class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name="image_convertor")

        # 카메라 이미지 데이터를 구독하는 ROS2 Subscriber 생성
        # '/image_jpeg/compressed' 토픽에서 CompressedImage 메시지를 수신하여 처리
        self.subscription = self.create_subscription(
            CompressedImage, "/image_jpeg/compressed", self.img_callback, 10
        )

    def img_callback(self, msg):
        """
        카메라에서 받은 압축된 이미지 데이터를 디코딩하여 OpenCV에서 사용 가능한 형식으로 변환 후 출력.
        1. bytes 데이터를 uint8 배열로 변환
        2. OpenCV를 이용해 BGR 형식의 이미지로 디코딩
        3. BGR 이미지를 그레이스케일로 변환
        4. 그레이스케일 이미지를 원하는 해상도로 리사이징
        5. 변환된 이미지를 화면에 출력
        """

        # 압축된 이미지 데이터를 uint8 배열로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)

        # OpenCV를 이용하여 BGR 이미지로 디코딩
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # BGR 이미지를 그레이스케일로 변환
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        # 이미지를 원하는 크기로 리사이징 (INTER_AREA 보간법 사용)
        img_resize = cv2.resize(img_gray, (1920, 1080), interpolation=cv2.INTER_AREA)

        # 변환된 이미지 출력
        cv2.imshow("img_bgr", img_bgr)  # 원본 BGR 이미지
        # cv2.imshow("img_gray", img_gray) # 변환된 그레이스케일 이미지
        # cv2.imshow("resize and gray", img_resize) # 리사이징된 이미지

        # OpenCV 창을 유지 (키 입력 대기)
        cv2.waitKey(1)


def main(args=None):
    """
    ROS2 노드를 초기화하고 실행하는 메인 함수.
    - rclpy.init(): ROS2 노드 초기화
    - IMGParser 노드 인스턴스 생성
    - rclpy.spin(): 노드가 종료될 때까지 실행 유지
    """
    rclpy.init(args=args)

    # IMGParser 노드 실행
    image_parser = IMGParser()

    # ROS2 노드를 지속적으로 실행 (종료될 때까지)
    rclpy.spin(image_parser)


if __name__ == "__main__":
    main()
