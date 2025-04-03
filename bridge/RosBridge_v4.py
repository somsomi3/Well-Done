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
call C:\\Users\SSAFY\Desktop\temp\S12P21E102\sim\ros2_ws\install\local_setup.bat
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

from flask import Flask, request, jsonify
import threading
import rclpy
from rclpy.node import Node
import time
import requests
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
import os
import math  # 스캔 데이터 처리에 필요

# 필요한 메시지 타입 가져오기
# 우선 기본 메시지 타입 가져오기 (중요: Header를 먼저 임포트)
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData  # 메시지 타입 추가

app = Flask(__name__)

# 필요한 메시지 타입을 가져오기 위한 환경 설정
# SSAFY 메시지 패키지 경로를 Python 경로에 추가
sys.path.append(
    "C:/Users/SSAFY/Desktop/ros/S12P21E102/sim/ros2_ws/install/ssafy_bridge/Lib/site-packages"
)

# 메시지 타입 가져오기
try:
    from ssafy_msgs.msg import EnviromentStatus, TurtlebotStatus, ScanWithPose
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
            
    class ScanWithPose:
        def __init__(self):
            self.header = None  # std_msgs/Header 타입
            self.ranges = []    # float32[]
            self.intensities = []  # float32[]
            self.angle_min = 0.0
            self.angle_max = 0.0
            self.angle_increment = 0.0
            self.range_min = 0.0
            self.range_max = 0.0
            self.scan_time = 0.0
            self.time_increment = 0.0
            self.pose_x = 0.0
            self.pose_y = 0.0
            self.pose_theta = 0.0


class RobotBridgeNode(Node):
    def __init__(self):
        super().__init__("robot_bridge_node")
        # 시간 인터벌 설정
        self.last_envir_send_time = 0.0
        self.last_turtlebot_send_time = 0.0
        self.last_global_path_send_time = 0.0  # 글로벌 경로 데이터 전송 시간 추가
        self.last_local_path_send_time = 0.0   # 로컬 경로 데이터 전송 시간 추가
        self.last_odom_send_time = 0.0         # 오도메트리 데이터 전송 시간 추가
        self.last_scan_send_time = 0.0         # 스캔 데이터 전송 시간 추가
        self.last_map_send_time = 0.0          # 맵 데이터 전송 시간 추가
        self.send_interval = 1.0

        # 명령 큐 시스템 추가
        self.command_queue = []
        self.queue_lock = threading.Lock()

        # 명령 처리 타이머 추가 (10Hz로 처리)
        self.command_timer = self.create_timer(0.1, self.process_commands)

        # Spring 서버 URL
        self.spring_server_url = "http://localhost:8080"

        # JWT 토큰 획득
        self.jwt_token = self.get_jwt_token()

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # /envir_status 토픽 구독 설정
        self.envir_subscription = self.create_subscription(
            EnviromentStatus, "/envir_status", self.envir_status_callback, qos_profile
        )

        # /turtlebot_status 토픽 구독 설정
        self.turtlebot_subscription = self.create_subscription(
            TurtlebotStatus, "/turtlebot_status", self.turtlebot_status_callback, qos_profile
        )

        # /global_path 토픽 구독 설정
        self.global_path_subscription = self.create_subscription(
            Path, "/global_path", self.global_path_callback, qos_profile
        )
        
        # /local_path 토픽 구독 설정
        self.local_path_subscription = self.create_subscription(
            Path, "/local_path", self.local_path_callback, qos_profile
        )
        
        # /odom_true 토픽 구독 설정
        self.odom_subscription = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, qos_profile
        )
        
        # /scan_with_pose 토픽 구독 설정
        self.scan_subscription = self.create_subscription(
            ScanWithPose, "/scan_with_pose", self.scan_callback, qos_profile
        )
        
        # /map 토픽 구독 설정
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, qos_profile
        )

        # /cmd_vel 토픽 발행 설정
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', qos_profile
        )

        # /global_path 토픽 발행 설정 추가
        self.global_path_publisher = self.create_publisher(
            Path, '/global_path', qos_profile
        )
        
        # /local_path 토픽 발행 설정 추가
        self.local_path_publisher = self.create_publisher(
            Path, '/local_path', qos_profile
        )
        
        # /odom_true 토픽 발행 설정 추가 (필요한 경우)
        self.odom_publisher = self.create_publisher(
            Odometry, '/odom_true', qos_profile
        )
        
        # /scan_with_pose 토픽 발행 설정 추가 (필요한 경우)
        self.scan_publisher = self.create_publisher(
            ScanWithPose, '/scan_with_pose', qos_profile
        )
        
        # /map 토픽 발행 설정 추가 (필요한 경우)
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map', qos_profile
        )

        self.get_logger().info("Robot bridge node initialized with global path, local path, odometry, scan, and map support")

    def get_jwt_token(self):
        """Spring 서버에서 JWT 토큰 획득"""
        try:
            # 로그인 요청
            login_data = {
                "username": "admin", 
                "password": "7156ase@", 
            }

            # 로그인 api 변경시 이 부분도 수정해줘야 함
            response = requests.post(
                f"{self.spring_server_url}/api/auth/login", json=login_data
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
        
    def process_commands(self):
        """명령 큐에서 명령을 가져와 실행"""
        if not self.command_queue:
            return
        
        with self.queue_lock:
            if self.command_queue:
                command = self.command_queue.pop(0)
                
                # 명령 유형에 따라 처리
                if command['type'] == 'move':
                    self.execute_move_command(command)
                elif command['type'] == 'grab':
                    self.execute_grab_command(command)
                elif command['type'] == 'global_path':  # 글로벌 경로 명령 처리
                    self.execute_global_path_command(command)
                elif command['type'] == 'local_path':  # 로컬 경로 명령 처리
                    self.execute_local_path_command(command)
                elif command['type'] == 'odom':  # 오도메트리 명령 처리
                    self.execute_odom_command(command)
                elif command['type'] == 'scan':  # 스캔 명령 처리
                    self.execute_scan_command(command)
                elif command['type'] == 'map':  # 맵 명령 처리
                    self.execute_map_command(command)

    def envir_status_callback(self, msg):
        """환경 상태 토픽에서 데이터를 받아 Spring 서버로 전송"""
        current_time = time.time()
        
        # Check if enough time has passed since the last send
        if current_time - self.last_envir_send_time >= self.send_interval:
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
                    f"{self.spring_server_url}/api/robot/envir-status", json=data, headers=headers
                )

                if response.status_code == 200:
                    self.get_logger().info(
                        "Environment data successfully sent to Spring server"
                    )
                else:
                    self.get_logger().error(
                        f"Spring server error: {response.status_code}, {response.text}"
                    )
                
                self.last_envir_send_time = current_time

            except Exception as e:
                self.get_logger().error(
                    f"Exception during environment data processing: {str(e)}"
                )
    
    def turtlebot_status_callback(self, msg):
        """터틀봇 상태 토픽에서 데이터를 받아 Spring 서버로 전송"""
        current_time = time.time()

        if current_time - self.last_turtlebot_send_time >= self.send_interval:
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
                        # data['linear_y'] = getattr(msg.twist.linear, 'y', 0.0)
                        # data['linear_z'] = getattr(msg.twist.linear, 'z', 0.0)
                    
                    # angular 속도
                    if hasattr(msg.twist, 'angular'):
                        # data['angular_x'] = getattr(msg.twist.angular, 'x', 0.0)
                        # data['angular_y'] = getattr(msg.twist.angular, 'y', 0.0)
                        data['angular_z'] = getattr(msg.twist.angular, 'z', 0.0)

                self.get_logger().info(f"Turtlebot status data received: {data}")

                # JWT 토큰을 헤더에 추가
                headers = {"Content-Type": "application/json"}
                if self.jwt_token:
                    headers["Authorization"] = f"Bearer {self.jwt_token}"

                # Spring 서버로 POST 요청
                response = requests.post(
                    f"{self.spring_server_url}/api/robot/turtlebot-status", json=data, headers=headers
                )

                if response.status_code == 200:
                    self.get_logger().info(
                        "Turtlebot status data successfully sent to Spring server"
                    )
                else:
                    self.get_logger().error(
                        f"Spring server error: {response.status_code}, {response.text}"
                    )
                    
                self.last_turtlebot_send_time = current_time

            except Exception as e:
                self.get_logger().error(
                    f"Exception during turtlebot status data processing: {str(e)}"
                )

    def global_path_callback(self, msg):
        """글로벌 경로 토픽에서 데이터를 받아 Spring 서버로 전송"""
        current_time = time.time()
        
        if current_time - self.last_global_path_send_time >= self.send_interval:
            try:
                # Path 메시지에서 데이터 추출
                path_data = {
                    # "header": {
                    #     "frame_id": msg.header.frame_id,
                    #     "stamp": {
                    #         "sec": msg.header.stamp.sec,
                    #         "nanosec": msg.header.stamp.nanosec
                    #     }
                    # },
                    "poses": []
                }
                
                # PoseStamped 배열 처리
                for pose in msg.poses:
                    pose_data = {
                        # "header": {
                        #     "frame_id": pose.header.frame_id,
                        #     "stamp": {
                        #         "sec": pose.header.stamp.sec,
                        #         "nanosec": pose.header.stamp.nanosec
                        #     }
                        # },
                        "pose": {
                            "position": {
                                "x": pose.pose.position.x,
                                "y": pose.pose.position.y,
                                "z": pose.pose.position.z
                            },
                            # "orientation": {
                            #     "x": pose.pose.orientation.x,
                            #     "y": pose.pose.orientation.y,
                            #     "z": pose.pose.orientation.z,
                            #     "w": pose.pose.orientation.w
                            # }
                        }
                    }
                    path_data["poses"].append(pose_data)
                
                # 경로 길이 정보 추가
                path_data["path_length"] = len(msg.poses)
                
                self.get_logger().info(f"Global path data received with {len(msg.poses)} poses")
                
                # JWT 토큰을 헤더에 추가
                headers = {"Content-Type": "application/json"}
                if self.jwt_token:
                    headers["Authorization"] = f"Bearer {self.jwt_token}"
                
                # Spring 서버로 POST 요청
                response = requests.post(
                    f"{self.spring_server_url}/api/robot/global-path", json=path_data, headers=headers
                )
                
                if response.status_code == 200:
                    self.get_logger().info(
                        "Global path data successfully sent to Spring server"
                    )
                else:
                    self.get_logger().error(
                        f"Spring server error: {response.status_code}, {response.text}"
                    )
                
                self.last_global_path_send_time = current_time
            
            except Exception as e:
                self.get_logger().error(
                    f"Exception during global path data processing: {str(e)}"
                )
                
    def local_path_callback(self, msg):
        """로컬 경로 토픽에서 데이터를 받아 Spring 서버로 전송"""
        current_time = time.time()
        
        if current_time - self.last_local_path_send_time >= self.send_interval:
            try:
                # Path 메시지에서 데이터 추출
                path_data = {
                    # "header": {
                    #     "frame_id": msg.header.frame_id,
                    #     "stamp": {
                    #         "sec": msg.header.stamp.sec,
                    #         "nanosec": msg.header.stamp.nanosec
                    #     }
                    # },
                    "poses": []
                }
                
                # PoseStamped 배열 처리
                for pose in msg.poses:
                    pose_data = {
                        # "header": {
                        #     "frame_id": pose.header.frame_id,
                        #     "stamp": {
                        #         "sec": pose.header.stamp.sec,
                        #         "nanosec": pose.header.stamp.nanosec
                        #     }
                        # },
                        "pose": {
                            "position": {
                                "x": pose.pose.position.x,
                                "y": pose.pose.position.y,
                                "z": pose.pose.position.z
                            },
                            # "orientation": {
                            #     "x": pose.pose.orientation.x,
                            #     "y": pose.pose.orientation.y,
                            #     "z": pose.pose.orientation.z,
                            #     "w": pose.pose.orientation.w
                            # }
                        }
                    }
                    path_data["poses"].append(pose_data)
                
                # 경로 길이 정보 추가
                path_data["path_length"] = len(msg.poses)
                
                self.get_logger().info(f"Local path data received with {len(msg.poses)} poses")
                
                # JWT 토큰을 헤더에 추가
                headers = {"Content-Type": "application/json"}
                if self.jwt_token:
                    headers["Authorization"] = f"Bearer {self.jwt_token}"
                
                # Spring 서버로 POST 요청
                response = requests.post(
                    f"{self.spring_server_url}/api/robot/local-path", json=path_data, headers=headers
                )
                
                if response.status_code == 200:
                    self.get_logger().info(
                        "Local path data successfully sent to Spring server"
                    )
                else:
                    self.get_logger().error(
                        f"Spring server error: {response.status_code}, {response.text}"
                    )
                
                self.last_local_path_send_time = current_time
            
            except Exception as e:
                self.get_logger().error(
                    f"Exception during local path data processing: {str(e)}"
                )
                
    def odom_callback(self, msg):
        """오도메트리 토픽에서 데이터를 받아 Spring 서버로 전송"""
        current_time = time.time()
        
        if current_time - self.last_odom_send_time >= self.send_interval:
            try:
                # Odometry 메시지에서 데이터 추출
                odom_data = {
                    # "header": {
                    #     "frame_id": msg.header.frame_id,
                    #     "stamp": {
                    #         "sec": msg.header.stamp.sec,
                    #         "nanosec": msg.header.stamp.nanosec
                    #     }
                    # },
                    # "child_frame_id": msg.child_frame_id,
                    "pose": {
                        "pose": {
                            "position": {
                                "x": msg.pose.pose.position.x,
                                "y": msg.pose.pose.position.y,
                                "z": msg.pose.pose.position.z
                            },
                            # "orientation": {
                            #     "x": msg.pose.pose.orientation.x,
                            #     "y": msg.pose.pose.orientation.y,
                            #     "z": msg.pose.pose.orientation.z,
                            #     "w": msg.pose.pose.orientation.w
                            # }
                        }
                    },
                    "twist": {
                        "twist": {
                            "linear": {
                                "x": msg.twist.twist.linear.x,
                                # "y": msg.twist.twist.linear.y,
                                # "z": msg.twist.twist.linear.z
                            },
                            "angular": {
                                # "x": msg.twist.twist.angular.x,
                                # "y": msg.twist.twist.angular.y,
                                "z": msg.twist.twist.angular.z
                            }
                        }
                    }
                }
                
                # 공분산 행렬 추가 (필요한 경우)
                # 공분산 행렬은 6x6 double 배열이지만, 직렬화를 위해 1차원 배열로 전송
                odom_data["pose"]["covariance"] = list(msg.pose.covariance)
                odom_data["twist"]["covariance"] = list(msg.twist.covariance)
                
                self.get_logger().info(f"Odometry data received from frame {msg.header.frame_id}")
                
                # JWT 토큰을 헤더에 추가
                headers = {"Content-Type": "application/json"}
                if self.jwt_token:
                    headers["Authorization"] = f"Bearer {self.jwt_token}"
                
                # Spring 서버로 POST 요청
                response = requests.post(
                    f"{self.spring_server_url}/api/robot/odometry", json=odom_data, headers=headers
                )
                
                if response.status_code == 200:
                    self.get_logger().info(
                        "Odometry data successfully sent to Spring server"
                    )
                else:
                    self.get_logger().error(
                        f"Spring server error: {response.status_code}, {response.text}"
                    )
                
                self.last_odom_send_time = current_time
            
            except Exception as e:
                self.get_logger().error(
                    f"Exception during odometry data processing: {str(e)}"
                )
                
    def map_callback(self, msg):
        """맵 토픽에서 데이터를 받아 Spring 서버로 전송"""
        current_time = time.time()
        
        if current_time - self.last_map_send_time >= self.send_interval:
            try:
                # 맵 메타데이터 추출
                map_data = {
                    # "header": {
                    #     "frame_id": msg.header.frame_id,
                    #     "stamp": {
                    #         "sec": msg.header.stamp.sec,
                    #         "nanosec": msg.header.stamp.nanosec
                    #     }
                    # },
                    "info": {
                        "width": msg.info.width,
                        "height": msg.info.height,
                        "resolution": msg.info.resolution,
                        "origin": {
                            "position": {
                                "x": msg.info.origin.position.x,
                                "y": msg.info.origin.position.y,
                                "z": msg.info.origin.position.z
                            },
                            "orientation": {
                                "x": msg.info.origin.orientation.x,
                                "y": msg.info.origin.orientation.y
                                # "z": msg.info.origin.orientation.z,
                                # "w": msg.info.origin.orientation.w
                            }
                        }
                    }
                }
                
                # 맵 데이터는 크기가 매우 클 수 있으므로 최적화 필요
                # 공간 절약을 위해 비어 있는 공간(-1)은 건너뛰고 장애물만 전송
                # compressed_data = []
                # for i, value in enumerate(msg.data):
                #     if value > 0:  # 0보다 크면 점유 확률이 있는 셀
                #         # 1차원 인덱스를 2차원 좌표로 변환
                #         x = i % msg.info.width
                #         y = i // msg.info.width
                #         compressed_data.append({
                #             "x": x,
                #             "y": y,
                #             "value": int(value)
                #         })
                
                # map_data["occupied_cells"] = compressed_data
                # map_data["cells_count"] = len(compressed_data)
                # map_data["total_cells"] = len(msg.data)
                
                # # 맵의 요약 통계도 포함
                # free_cells = sum(1 for val in msg.data if val == 0)
                # unknown_cells = sum(1 for val in msg.data if val == -1)
                # occupied_cells = sum(1 for val in msg.data if val > 0)
                
                # map_data["stats"] = {
                #     "free": free_cells,
                #     "unknown": unknown_cells,
                #     "occupied": occupied_cells,
                #     "occupancy_percent": round(occupied_cells / (free_cells + occupied_cells + 0.0001) * 100, 2)
                # }
                
                self.get_logger().info(f"Map data received: {msg.info.width}x{msg.info.height} with {len(compressed_data)} occupied cells")
                
                # JWT 토큰을 헤더에 추가
                headers = {"Content-Type": "application/json"}
                if self.jwt_token:
                    headers["Authorization"] = f"Bearer {self.jwt_token}"
                
                # Spring 서버로 POST 요청
                response = requests.post(
                    f"{self.spring_server_url}/api/robot/map", json=map_data, headers=headers
                )
                
                if response.status_code == 200:
                    self.get_logger().info(
                        "Map data successfully sent to Spring server"
                    )
                else:
                    self.get_logger().error(
                        f"Spring server error: {response.status_code}, {response.text}"
                    )
                
                self.last_map_send_time = current_time
            
            except Exception as e:
                self.get_logger().error(
                    f"Exception during map data processing: {str(e)}"
                )
    
    def scan_callback(self, msg):
        """스캔 데이터 토픽에서 데이터를 받아 Spring 서버로 전송"""
        current_time = time.time()
        
        if current_time - self.last_scan_send_time >= self.send_interval:
            try:
                # ScanWithPose 메시지에서 데이터 추출
                scan_data = {
                    # "header": {
                    #     "frame_id": msg.header.frame_id,
                    #     "stamp": {
                    #         "sec": msg.header.stamp.sec,
                    #         "nanosec": msg.header.stamp.nanosec
                    #     }
                    # },
                    # 범위 데이터 - 너무 크면 데이터 양 감소
                    # "ranges_sample": list(msg.ranges[:20]) if len(msg.ranges) > 20 else list(msg.ranges),
                    # "ranges_length": len(msg.ranges),
                    
                    # 강도 데이터 - 너무 크면 데이터 양 감소
                    # "intensities_sample": list(msg.intensities[:20]) if len(msg.intensities) > 20 else list(msg.intensities),
                    # "intensities_length": len(msg.intensities),
                    
                    # 스캔 파라미터
                    # "angle_min": msg.angle_min,
                    # "angle_max": msg.angle_max,
                    # "angle_increment": msg.angle_increment,
                    # "range_min": msg.range_min,
                    # "range_max": msg.range_max,
                    # "scan_time": msg.scan_time,
                    # "time_increment": msg.time_increment,
                    
                    # 포즈 정보
                    "pose": {
                        "x": msg.pose_x,
                        "y": msg.pose_y,
                        "theta": msg.pose_theta
                    }
                }
                
                # 장애물 검출 로직 - 간단한 예시
                obstacles = []
                if hasattr(msg, 'ranges') and len(msg.ranges) > 0:
                    angle = msg.angle_min
                    for i, r in enumerate(msg.ranges):
                        if r < msg.range_max and r > msg.range_min:  # 유효한 범위 내에 있는 경우
                            # 장애물 위치 계산 (로봇 중심 기준)
                            obstacle_x = msg.pose_x + r * math.cos(angle)
                            obstacle_y = msg.pose_y + r * math.sin(angle)
                            obstacles.append({
                                "x": obstacle_x,
                                "y": obstacle_y,
                                "distance": r
                            })
                        angle += msg.angle_increment
                        
                        # 너무 많은 장애물 정보를 보내지 않도록 제한
                        if len(obstacles) >= 10:
                            break
                
                scan_data["obstacles"] = obstacles
                
                self.get_logger().info(f"Scan data received with {len(msg.ranges)} range points")
                
                # JWT 토큰을 헤더에 추가
                headers = {"Content-Type": "application/json"}
                if self.jwt_token:
                    headers["Authorization"] = f"Bearer {self.jwt_token}"
                
                # Spring 서버로 POST 요청
                response = requests.post(
                    f"{self.spring_server_url}/api/robot/scan", json=scan_data, headers=headers
                )
                
                if response.status_code == 200:
                    self.get_logger().info(
                        "Scan data successfully sent to Spring server"
                    )
                else:
                    self.get_logger().error(
                        f"Spring server error: {response.status_code}, {response.text}"
                    )
                
                self.last_scan_send_time = current_time
            
            except Exception as e:
                self.get_logger().error(
                    f"Exception during scan data processing: {str(e)}"
                )

    def execute_move_command(self, command):
        """이동 명령 실행"""
        twist = Twist()
        twist.linear.x = command.get('linear_x', 0.0)
        twist.angular.z = command.get('angular_z', 0.0)
        
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Move command executed: {command}")

    def execute_grab_command(self, command):
        """잡기/놓기 명령 실행"""
        # 여기에 잡기/놓기 명령 실행 코드 구현
        self.get_logger().info(f"Grab command executed: {command}")

    def execute_global_path_command(self, command):
        """글로벌 경로 명령 실행"""
        try:
            # 새 Path 메시지 생성
            path_msg = Path()
            
            # Header 설정
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = command.get('frame_id', 'map')
            path_msg.header = header
            
            # Poses 배열 설정
            poses = command.get('poses', [])
            for pose_data in poses:
                pose_stamped = PoseStamped()
                
                # PoseStamped의 Header 설정
                pose_header = Header()
                pose_header.stamp = self.get_clock().now().to_msg()
                pose_header.frame_id = pose_data.get('frame_id', 'map')
                pose_stamped.header = pose_header
                
                # Position 설정
                pose_stamped.pose.position.x = pose_data.get('position', {}).get('x', 0.0)
                pose_stamped.pose.position.y = pose_data.get('position', {}).get('y', 0.0)
                pose_stamped.pose.position.z = pose_data.get('position', {}).get('z', 0.0)
                
                # Orientation 설정
                pose_stamped.pose.orientation.x = pose_data.get('orientation', {}).get('x', 0.0)
                pose_stamped.pose.orientation.y = pose_data.get('orientation', {}).get('y', 0.0)
                pose_stamped.pose.orientation.z = pose_data.get('orientation', {}).get('z', 0.0)
                pose_stamped.pose.orientation.w = pose_data.get('orientation', {}).get('w', 1.0)
                
                path_msg.poses.append(pose_stamped)
            
            # Path 메시지 발행
            self.global_path_publisher.publish(path_msg)
            self.get_logger().info(f"Global path command executed with {len(poses)} poses")
            
        except Exception as e:
            self.get_logger().error(f"Error executing global path command: {str(e)}")
            
    def execute_local_path_command(self, command):
        """로컬 경로 명령 실행"""
        try:
            # 새 Path 메시지 생성
            path_msg = Path()
            
            # Header 설정
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = command.get('frame_id', 'map')
            path_msg.header = header
            
            # Poses 배열 설정
            poses = command.get('poses', [])
            for pose_data in poses:
                pose_stamped = PoseStamped()
                
                # PoseStamped의 Header 설정
                pose_header = Header()
                pose_header.stamp = self.get_clock().now().to_msg()
                pose_header.frame_id = pose_data.get('frame_id', 'map')
                pose_stamped.header = pose_header
                
                # Position 설정
                pose_stamped.pose.position.x = pose_data.get('position', {}).get('x', 0.0)
                pose_stamped.pose.position.y = pose_data.get('position', {}).get('y', 0.0)
                pose_stamped.pose.position.z = pose_data.get('position', {}).get('z', 0.0)
                
                # Orientation 설정
                pose_stamped.pose.orientation.x = pose_data.get('orientation', {}).get('x', 0.0)
                pose_stamped.pose.orientation.y = pose_data.get('orientation', {}).get('y', 0.0)
                pose_stamped.pose.orientation.z = pose_data.get('orientation', {}).get('z', 0.0)
                pose_stamped.pose.orientation.w = pose_data.get('orientation', {}).get('w', 1.0)
                
                path_msg.poses.append(pose_stamped)
            
            # Path 메시지 발행
            self.local_path_publisher.publish(path_msg)
            self.get_logger().info(f"Local path command executed with {len(poses)} poses")
            
        except Exception as e:
            self.get_logger().error(f"Error executing local path command: {str(e)}")
            
    def execute_odom_command(self, command):
        """오도메트리 명령 실행 (주로 테스트 또는 시뮬레이션 목적)"""
        try:
            # 새 Odometry 메시지 생성
            odom_msg = Odometry()
            
            # Header 설정
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = command.get('frame_id', 'odom')
            odom_msg.header = header
            
            # child_frame_id 설정
            odom_msg.child_frame_id = command.get('child_frame_id', 'base_footprint')
            
            # Pose 설정
            pose_data = command.get('pose', {})
            if pose_data:
                # Position
                position = pose_data.get('position', {})
                odom_msg.pose.pose.position.x = position.get('x', 0.0)
                odom_msg.pose.pose.position.y = position.get('y', 0.0)
                odom_msg.pose.pose.position.z = position.get('z', 0.0)
                
                # Orientation (쿼터니언)
                orientation = pose_data.get('orientation', {})
                odom_msg.pose.pose.orientation.x = orientation.get('x', 0.0)
                odom_msg.pose.pose.orientation.y = orientation.get('y', 0.0)
                odom_msg.pose.pose.orientation.z = orientation.get('z', 0.0)
                odom_msg.pose.pose.orientation.w = orientation.get('w', 1.0)
                
                # 공분산 (있는 경우)
                covariance = pose_data.get('covariance', [0.0] * 36)
                for i, val in enumerate(covariance[:36]):  # 최대 36개 요소 (6x6 행렬)
                    odom_msg.pose.covariance[i] = val
            
            # Twist 설정
            twist_data = command.get('twist', {})
            if twist_data:
                # Linear velocity
                linear = twist_data.get('linear', {})
                odom_msg.twist.twist.linear.x = linear.get('x', 0.0)
                odom_msg.twist.twist.linear.y = linear.get('y', 0.0)
                odom_msg.twist.twist.linear.z = linear.get('z', 0.0)
                
                # Angular velocity
                angular = twist_data.get('angular', {})
                odom_msg.twist.twist.angular.x = angular.get('x', 0.0)
                odom_msg.twist.twist.angular.y = angular.get('y', 0.0)
                odom_msg.twist.twist.angular.z = angular.get('z', 0.0)
                
                # 공분산 (있는 경우)
                covariance = twist_data.get('covariance', [0.0] * 36)
                for i, val in enumerate(covariance[:36]):  # 최대 36개 요소 (6x6 행렬)
                    odom_msg.twist.covariance[i] = val
            
            # Odometry 메시지 발행
            self.odom_publisher.publish(odom_msg)
            self.get_logger().info(f"Odometry command executed from {odom_msg.header.frame_id} to {odom_msg.child_frame_id}")
            
        except Exception as e:
            self.get_logger().error(f"Error executing odometry command: {str(e)}")
            
    def execute_map_command(self, command):
        """맵 명령 실행 (주로 테스트 또는 시뮬레이션 목적)"""
        try:
            # 새 OccupancyGrid 메시지 생성
            map_msg = OccupancyGrid()
            
            # Header 설정
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = command.get('frame_id', 'map')
            map_msg.header = header
            
            # Map 메타데이터 설정
            map_info = MapMetaData()
            map_info.width = command.get('width', 100)
            map_info.height = command.get('height', 100)
            map_info.resolution = command.get('resolution', 0.05)  # 5cm 해상도
            
            # 원점 위치 설정
            origin = Pose()
            origin_pos = command.get('origin_position', {})
            origin_ori = command.get('origin_orientation', {})
            
            origin.position.x = origin_pos.get('x', 0.0)
            origin.position.y = origin_pos.get('y', 0.0)
            origin.position.z = origin_pos.get('z', 0.0)
            
            origin.orientation.x = origin_ori.get('x', 0.0)
            origin.orientation.y = origin_ori.get('y', 0.0)
            origin.orientation.z = origin_ori.get('z', 0.0)
            origin.orientation.w = origin_ori.get('w', 1.0)
            
            map_info.origin = origin
            map_msg.info = map_info
            
            # 맵 데이터 설정
            # 일반적으로 맵은 전체 크기가 클 수 있으므로 기본적으로 모든 셀을 '알 수 없음'(-1)으로 설정
            data_size = map_info.width * map_info.height
            map_data = [-1] * data_size  # 모든 셀을 -1(알 수 없음)로 초기화
            
            # 명령에서 제공된 특정 점유 셀 설정
            occupied_cells = command.get('occupied_cells', [])
            for cell in occupied_cells:
                x = cell.get('x', 0)
                y = cell.get('y', 0)
                value = cell.get('value', 100)
                
                # x, y 좌표를 1차원 인덱스로 변환
                if 0 <= x < map_info.width and 0 <= y < map_info.height:
                    index = y * map_info.width + x
                    map_data[index] = value
            
            # 빈 공간 설정 (자유 셀)
            free_cells = command.get('free_cells', [])
            for cell in free_cells:
                x = cell.get('x', 0)
                y = cell.get('y', 0)
                
                if 0 <= x < map_info.width and 0 <= y < map_info.height:
                    index = y * map_info.width + x
                    map_data[index] = 0  # 0은 비어있는 공간
            
            # 맵이 너무 큰 경우 샘플 맵 생성
            if not occupied_cells and not free_cells:
                # 간단한 예시 맵 생성 (10x10 방 형태)
                room_size = min(map_info.width, map_info.height) // 5
                offset_x = map_info.width // 2 - room_size // 2
                offset_y = map_info.height // 2 - room_size // 2
                
                # 방의 경계를 점유 셀로 설정
                for i in range(room_size):
                    # 위쪽 벽
                    top_wall_idx = (offset_y) * map_info.width + (offset_x + i)
                    if 0 <= top_wall_idx < data_size:
                        map_data[top_wall_idx] = 100
                    
                    # 아래쪽 벽
                    bottom_wall_idx = (offset_y + room_size - 1) * map_info.width + (offset_x + i)
                    if 0 <= bottom_wall_idx < data_size:
                        map_data[bottom_wall_idx] = 100
                    
                    # 왼쪽 벽
                    left_wall_idx = (offset_y + i) * map_info.width + offset_x
                    if 0 <= left_wall_idx < data_size:
                        map_data[left_wall_idx] = 100
                    
                    # 오른쪽 벽
                    right_wall_idx = (offset_y + i) * map_info.width + (offset_x + room_size - 1)
                    if 0 <= right_wall_idx < data_size:
                        map_data[right_wall_idx] = 100
                
                # 방 내부를 자유 공간으로 설정
                for y in range(offset_y + 1, offset_y + room_size - 1):
                    for x in range(offset_x + 1, offset_x + room_size - 1):
                        idx = y * map_info.width + x
                        if 0 <= idx < data_size:
                            map_data[idx] = 0
                
                # 방 가운데에 장애물 추가
                center_x = offset_x + room_size // 2
                center_y = offset_y + room_size // 2
                center_idx = center_y * map_info.width + center_x
                if 0 <= center_idx < data_size:
                    map_data[center_idx] = 100
            
            # 맵 데이터 설정
            map_msg.data = map_data
            
            # OccupancyGrid 메시지 발행
            self.map_publisher.publish(map_msg)
            self.get_logger().info(f"Map command executed, created {map_info.width}x{map_info.height} map")
            
        except Exception as e:
            self.get_logger().error(f"Error executing map command: {str(e)}")
            
    def execute_scan_command(self, command):
        """스캔 명령 실행 (주로 테스트 또는 시뮬레이션 목적)"""
        try:
            # 새 ScanWithPose 메시지 생성
            scan_msg = ScanWithPose()
            
            # Header 설정
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = command.get('frame_id', 'laser')
            scan_msg.header = header
            
            # 레이저 스캔 파라미터 설정
            scan_msg.angle_min = command.get('angle_min', -3.14159)
            scan_msg.angle_max = command.get('angle_max', 3.14159)
            scan_msg.angle_increment = command.get('angle_increment', 0.01745)
            scan_msg.range_min = command.get('range_min', 0.0)
            scan_msg.range_max = command.get('range_max', 10.0)
            scan_msg.scan_time = command.get('scan_time', 0.1)
            scan_msg.time_increment = command.get('time_increment', 0.01)
            
            # 포즈 설정
            pose = command.get('pose', {})
            scan_msg.pose_x = pose.get('x', 0.0)
            scan_msg.pose_y = pose.get('y', 0.0)
            scan_msg.pose_theta = pose.get('theta', 0.0)
            
            # 레인지 데이터 설정
            ranges = command.get('ranges', [])
            intensities = command.get('intensities', [])
            
            # 범위와 강도 데이터가 너무 적은 경우 샘플 데이터 생성
            if not ranges:
                # 샘플 레인지 데이터 생성 (간단한 예시)
                num_rays = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
                ranges = [5.0] * num_rays  # 기본값으로 5미터 거리 설정
                
                # 약간의 랜덤 변동 추가
                import random
                for i in range(num_rays):
                    if random.random() < 0.3:  # 30% 확률로 장애물 시뮬레이션
                        ranges[i] = random.uniform(0.5, 4.0)
            
            # 강도 데이터가 없는 경우
            if not intensities and ranges:
                intensities = [100.0] * len(ranges)  # 기본 강도 값
            
            scan_msg.ranges = ranges
            scan_msg.intensities = intensities
            
            # ScanWithPose 메시지 발행
            self.scan_publisher.publish(scan_msg)
            self.get_logger().info(f"Scan command executed with {len(ranges)} range points")
            
        except Exception as e:
            self.get_logger().error(f"Error executing scan command: {str(e)}")

# Flask 라우트 설정
@app.route('/command', methods=['POST'])
def receive_command():
    """Spring 서버에서 명령 수신"""
    command_data = request.json
    
    command_type = command_data.get('command')
    if not command_type:
        return jsonify({'status': 'error', 'message': '명령 유형이 누락되었습니다'}), 400
    
    # 명령 유형에 따라 처리
    if command_type == 'move':
        # 이동 명령 검증 및 처리
        linear_x = command_data.get('linear_x', 0.0)
        angular_z = command_data.get('angular_z', 0.0)
        
        # 명령 큐에 추가
        with node.queue_lock:
            node.command_queue.append({
                'type': 'move',
                'linear_x': linear_x,
                'angular_z': angular_z
            })
        
        return jsonify({'status': 'success', 'message': '이동 명령이 큐에 추가되었습니다'})
    
    elif command_type == 'grab':
        # 잡기/놓기 명령 처리
        action = command_data.get('action', 'grab')
        
        with node.queue_lock:
            node.command_queue.append({
                'type': 'grab',
                'action': action
            })
        
        return jsonify({'status': 'success', 'message': f'물체 {action} 명령이 큐에 추가되었습니다'})
    
    elif command_type == 'global_path':
        # 글로벌 경로 명령 검증 및 처리
        frame_id = command_data.get('frame_id', 'map')
        poses = command_data.get('poses', [])
        
        if not poses:
            return jsonify({'status': 'error', 'message': '경로 포즈 데이터가 누락되었습니다'}), 400
        
        # 명령 큐에 추가
        with node.queue_lock:
            node.command_queue.append({
                'type': 'global_path',
                'frame_id': frame_id,
                'poses': poses
            })
        
        return jsonify({'status': 'success', 'message': f'{len(poses)}개 포즈로 구성된 글로벌 경로 명령이 큐에 추가되었습니다'})
        
    elif command_type == 'local_path':
        # 로컬 경로 명령 검증 및 처리
        frame_id = command_data.get('frame_id', 'map')
        poses = command_data.get('poses', [])
        
        if not poses:
            return jsonify({'status': 'error', 'message': '경로 포즈 데이터가 누락되었습니다'}), 400
        
        # 명령 큐에 추가
        with node.queue_lock:
            node.command_queue.append({
                'type': 'local_path',
                'frame_id': frame_id,
                'poses': poses
            })
        
        return jsonify({'status': 'success', 'message': f'{len(poses)}개 포즈로 구성된 로컬 경로 명령이 큐에 추가되었습니다'})
    
    elif command_type == 'odom':
        # 오도메트리 명령 검증 및 처리
        frame_id = command_data.get('frame_id', 'odom')
        child_frame_id = command_data.get('child_frame_id', 'base_footprint')
        pose = command_data.get('pose', {})
        twist = command_data.get('twist', {})
        
        # 명령 큐에 추가
        with node.queue_lock:
            node.command_queue.append({
                'type': 'odom',
                'frame_id': frame_id,
                'child_frame_id': child_frame_id,
                'pose': pose,
                'twist': twist
            })
        
        return jsonify({'status': 'success', 'message': '오도메트리 명령이 큐에 추가되었습니다'})
    
    elif command_type == 'scan':
        # 스캔 명령 검증 및 처리
        frame_id = command_data.get('frame_id', 'laser')
        ranges = command_data.get('ranges', [])
        intensities = command_data.get('intensities', [])
        pose = command_data.get('pose', {})
        
        # 명령 큐에 추가
        with node.queue_lock:
            node.command_queue.append({
                'type': 'scan',
                'frame_id': frame_id,
                'ranges': ranges,
                'intensities': intensities,
                'pose': pose,
                'angle_min': command_data.get('angle_min', -3.14159),
                'angle_max': command_data.get('angle_max', 3.14159),
                'angle_increment': command_data.get('angle_increment', 0.01745),
                'range_min': command_data.get('range_min', 0.0),
                'range_max': command_data.get('range_max', 10.0),
                'scan_time': command_data.get('scan_time', 0.1),
                'time_increment': command_data.get('time_increment', 0.01)
            })
        
        return jsonify({'status': 'success', 'message': '스캔 명령이 큐에 추가되었습니다'})
        
    elif command_type == 'map':
        # 맵 명령 검증 및 처리
        frame_id = command_data.get('frame_id', 'map')
        width = command_data.get('width', 100)
        height = command_data.get('height', 100)
        resolution = command_data.get('resolution', 0.05)
        origin_position = command_data.get('origin_position', {})
        origin_orientation = command_data.get('origin_orientation', {})
        occupied_cells = command_data.get('occupied_cells', [])
        free_cells = command_data.get('free_cells', [])
        
        # 명령 큐에 추가
        with node.queue_lock:
            node.command_queue.append({
                'type': 'map',
                'frame_id': frame_id,
                'width': width,
                'height': height,
                'resolution': resolution,
                'origin_position': origin_position,
                'origin_orientation': origin_orientation,
                'occupied_cells': occupied_cells,
                'free_cells': free_cells
            })
        
        return jsonify({'status': 'success', 'message': f'{width}x{height} 크기의 맵 명령이 큐에 추가되었습니다'})
    
    else:
        return jsonify({'status': 'error', 'message': f'알 수 없는 명령 유형: {command_type}'}), 400

# 경로 조회 API 추가
@app.route('/global-path', methods=['GET'])
def get_global_path():
    """현재 글로벌 경로 조회"""
    # TODO: 저장된 최신 경로 데이터 반환 로직 구현 필요
    return jsonify({'status': 'success', 'message': '글로벌 경로 조회 기능은 아직 구현되지 않았습니다'})

@app.route('/local-path', methods=['GET'])
def get_local_path():
    """현재 로컬 경로 조회"""
    # TODO: 저장된 최신 경로 데이터 반환 로직 구현 필요
    return jsonify({'status': 'success', 'message': '로컬 경로 조회 기능은 아직 구현되지 않았습니다'})

@app.route('/odometry', methods=['GET'])
def get_odometry():
    """현재 오도메트리 데이터 조회"""
    # TODO: 저장된 최신 오도메트리 데이터 반환 로직 구현 필요
    return jsonify({'status': 'success', 'message': '오도메트리 조회 기능은 아직 구현되지 않았습니다'})

@app.route('/scan', methods=['GET'])
def get_scan():
    """현재 스캔 데이터 조회"""
    # TODO: 저장된 최신 스캔 데이터 반환 로직 구현 필요
    return jsonify({'status': 'success', 'message': '스캔 데이터 조회 기능은 아직 구현되지 않았습니다'})

@app.route('/map', methods=['GET'])
def get_map():
    """현재 맵 데이터 조회"""
    # TODO: 저장된 최신 맵 데이터 반환 로직 구현 필요
    return jsonify({'status': 'success', 'message': '맵 데이터 조회 기능은 아직 구현되지 않았습니다'})


def main(args=None):
    rclpy.init(args=args)
    global node
    node = RobotBridgeNode()
    
    # Flask 서버를 별도 스레드에서 실행
    flask_thread = threading.Thread(target=run_flask_server)
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def run_flask_server():
    """Flask 서버 실행"""
    app.run(host='0.0.0.0', port=5000)

if __name__ == "__main__":
    main()