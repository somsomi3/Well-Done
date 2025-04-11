#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
### 브릿지 코드 개요 및 실행 방법

브릿지에서는 토픽의 메시지를 가져와서 스프링쪽으로 보내주거나,
스프링에서 받아와진 json 데이터를 토픽으로 보내주는 역할을 주로 합니다.

현재 이 코드는 스프링에서 받아온 json 데이터를 토픽으로 보내주는 역할만 구현되어 있으며,
나중에 토픽 명세서가 정리되면 거기에 맞춰 코드를 정리해야 합니다.

코드를 실행하기 전, rqt 실행과 마찬가지로 터미널에 ros2를 미리 소싱을 해줘야 합니다

# ROS2 실행을 위한 환경 변수 설정
call C:\dev\ros2_eloquent\setup.bat

# 워크스페이스에서 설치된 패키지를 사용하도록 설정
call C:\Users\SSAFY\Desktop\temp\S12P21E102\sim\ros2_ws\install\local_setup.bat
(자신의 경로에 맞게 변경경)

이 소싱과정을 거쳐줘야, rclpy를 import 할 수 있습니다.
(rclpy는 ros2에서 파이썬을 사용하기 위한 라이브러리입니다, 추가로 pip 설치할 필요 없이, 소싱만 해주면 됩니다. vscode의 오류는 무시하셔도 됩니다)

'''

'''
### ROS2 토픽 구조 알아내는 방법

1. ros2 topic list
토픽의 리스트를 불러옵니다
ex)
ros2 topic list
/custom_object_info
/envir_status
/hand_control

2. ros2 topic info /토픽이름 
토픽의 데이터 타입을 알려줍니다
ex)
ros2 topic info /turtlebot_status
Type: ssafy_msgs/msg/TurtlebotStatus
Publisher count: 1
Subscriber count: 0

3. ros2 interface show 메시지타입
토픽내부의 데이터 타입들을 알려줍니다
ex)
ros2 interface show ssafy_msgs/msg/TurtlebotStatus
geometry_msgs/Twist twist
uint8 power_supply_status
float32 battery_percentage
bool can_use_hand
bool can_put
bool can_lift
'''


import rclpy
from rclpy.node import Node
import requests
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
import os

# 필요한 메시지 타입을 가져오기 위한 환경 설정
# SSAFY 메시지 패키지 경로를 Python 경로에 추가
# 경로는 자신의 컴퓨터 마다 다르니 따로 설정해 둘 것
sys.path.append(
    "C:/Users/SSAFY/Desktop/ros/mobility-smarthome-skeleton/ros2_smart_home/install/ssafy_msgs/Lib/site-packages"
)

# 메시지 타입 가져오기
try:
    from ssafy_msgs.msg import EnviromentStatus, TurtlebotStatus
    # 마찬가지로 메시지 타입 임포트 불가 오류는 무시해도 됩니다
except ImportError:
    print("ssafy_msgs 패키지를 찾을 수 없습니다. 경로를 확인해주세요.")

    # 패키지를 가져올 수 없을 경우 대체 클래스 정의 (더미)
    class EnviromentStatus:
        def __init__(self):
            self.month = 0
            self.day = 0
            self.hour = 0
            self.minute = 0
            self.temperature = 0
            self.weather = ""
    
    class TurtlebotStatus:
        def __init__(self):
            self.twist = None  # 본래는 geometry_msgs/Twist 타입
            self.power_supply_status = 0
            self.battery_percentage = 0.0
            self.can_use_hand = False
            self.can_put = False
            self.can_lift = False


class RobotBridgeNode(Node):
    def __init__(self):
        super().__init__("robot_bridge_node")

        # Spring 서버 URL
        # 최종 땐 EC2쪽의 서버 링크로 바꿔줘야 합니다
        self.spring_server_url = "http://localhost:8080"

        # JWT 토큰 획득
        self.jwt_token = self.get_jwt_token()

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # /envir_status 토픽 구독 설정
        self.envir_subscription = self.create_subscription(
            EnviromentStatus, "/envir_status", self.envir_status_callback, qos_profile
        )

        # /turtlebot_status 토픽 구독 설정
        self.turtlebot_subscription = self.create_subscription(
            TurtlebotStatus, "/turtlebot_status", self.turtlebot_status_callback, qos_profile
        )

        self.get_logger().info("Robot bridge node initialized")

    def get_jwt_token(self):
        """Spring 서버에서 JWT 토큰 획득"""
        try:
            # 로그인 요청
            login_data = {
                "username": "브릿지용_계정", 
                "password": "으로_변경할_것것", 
            }

            # 로그인 api 변경시 이 부분도 수정해줘야 함
            response = requests.post(
                f"{self.spring_server_url}/auth/login", json=login_data
            )

            if response.status_code == 200:
                response_data = response.json()
                token = response_data.get("accessToken")

                if token:
                    self.get_logger().info("JWT token acquisition successful")
                    return token
                else:
                    self.get_logger().error("Could not find token in response")
                    return None
            else:
                self.get_logger().error(
                    f"JWT token acquisition failed: {response.status_code}"
                )
                return None
        except Exception as e:
            self.get_logger().error(f"Error while getting JWT token: {str(e)}")
            return None

    def envir_status_callback(self, msg):
        """환경 상태 토픽에서 데이터를 받아 Spring 서버로 전송"""
        try:
            # EnviromentStatus 메시지에서 데이터 추출
            data = {
                "month": msg.month,
                "day": msg.day,
                "hour": msg.hour,
                "minute": msg.minute,
                "temperature": msg.temperature,
                "weather": msg.weather,
            }

            self.get_logger().info(f"Environment data received: {data}")

            # JWT 토큰을 헤더에 추가
            headers = {"Content-Type": "application/json"}
            if self.jwt_token:
                headers["Authorization"] = f"Bearer {self.jwt_token}"

            # Spring 서버로 POST 요청
            response = requests.post(
                f"{self.spring_server_url}/api/envir-status", json=data, headers=headers
            )

            if response.status_code == 200:
                self.get_logger().info(
                    "Environment data successfully sent to Spring server"
                )
            else:
                self.get_logger().error(
                    f"Spring server error: {response.status_code}, {response.text}"
                )

        except Exception as e:
            self.get_logger().error(
                f"Exception during environment data processing: {str(e)}"
            )
    
    def turtlebot_status_callback(self, msg):
        """터틀봇 상태 토픽에서 데이터를 받아 Spring 서버로 전송"""
        try:
            # TurtlebotStatus 메시지에서 데이터 추출
            # Twist 타입은 중첩된 구조이므로 개별 필드로 평탄화
            data = {
                "battery_percentage": msg.battery_percentage,
                "power_supply_status": msg.power_supply_status,
                "can_use_hand": msg.can_use_hand,
                "can_put": msg.can_put,
                "can_lift": msg.can_lift,
            }
            
            # Twist 데이터 추가 (있는 경우)
            if hasattr(msg, 'twist') and msg.twist is not None:
                # linear 속도
                if hasattr(msg.twist, 'linear'):
                    data['linear_x'] = getattr(msg.twist.linear, 'x', 0.0)
                    data['linear_y'] = getattr(msg.twist.linear, 'y', 0.0)
                    data['linear_z'] = getattr(msg.twist.linear, 'z', 0.0)
                
                # angular 속도
                if hasattr(msg.twist, 'angular'):
                    data['angular_x'] = getattr(msg.twist.angular, 'x', 0.0)
                    data['angular_y'] = getattr(msg.twist.angular, 'y', 0.0)
                    data['angular_z'] = getattr(msg.twist.angular, 'z', 0.0)

            self.get_logger().info(f"Turtlebot status data received: {data}")

            # JWT 토큰을 헤더에 추가
            headers = {"Content-Type": "application/json"}
            if self.jwt_token:
                headers["Authorization"] = f"Bearer {self.jwt_token}"

            # Spring 서버로 POST 요청
            response = requests.post(
                f"{self.spring_server_url}/api/turtlebot-status", json=data, headers=headers
            )

            if response.status_code == 200:
                self.get_logger().info(
                    "Turtlebot status data successfully sent to Spring server"
                )
            else:
                self.get_logger().error(
                    f"Spring server error: {response.status_code}, {response.text}"
                )

        except Exception as e:
            self.get_logger().error(
                f"Exception during turtlebot status data processing: {str(e)}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()