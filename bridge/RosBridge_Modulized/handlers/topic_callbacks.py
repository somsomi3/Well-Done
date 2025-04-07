# topic_callbacks.py
import time
import requests
import json
import math  # 스캔 데이터 처리에 필요
import base64  # 이미지 데이터 인코딩에 필요

def register_all_callbacks(node):
    """모든 콜백 메서드를 노드에 등록"""
    node.envir_status_callback = lambda msg: envir_status_callback(node, msg)
    node.turtlebot_status_callback = lambda msg: turtlebot_status_callback(node, msg)
    node.global_path_callback = lambda msg: global_path_callback(node, msg)
    node.local_path_callback = lambda msg: local_path_callback(node, msg)
    node.odom_callback = lambda msg: odom_callback(node, msg)
    node.scan_callback = lambda msg: scan_callback(node, msg)
    node.map_callback = lambda msg: map_callback(node, msg)
    node.mapping_done_callback = lambda msg: mapping_done_callback(node, msg)
    node.map_status_callback = lambda msg: map_status_callback(node, msg)
    node.obstacle_alert_callback = lambda msg: obstacle_alert_callback(node, msg)
    node.goal_status_callback = lambda msg: goal_status_callback(node, msg)
    node.pick_done_callback = lambda msg: pick_done_callback(node, msg)
    node.place_done_callback = lambda msg: place_done_callback(node, msg)
    node.image_jpeg_compressed_callback = lambda msg: image_jpeg_compressed_callback(node, msg)

def envir_status_callback(node, msg):
    """환경 상태 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    
    # 마지막 전송 이후 충분한 시간이 지났는지 확인
    if current_time - node.last_send_times['envir'] >= node.send_interval:
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

            node.get_logger().info(f"Environment data received: {data}")

            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }

                    # Spring 서버로 POST 요청 (타임아웃 설정)
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/envir-status", 
                        json=data, 
                        headers=headers,
                        timeout=2.0
                    )

                    if response.status_code == 200:
                        node.get_logger().info(
                            "Environment data successfully sent to Spring server"
                        )
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send environment data to Spring server: {str(e)}"
                    )
            else:
                node.get_logger().debug("JWT token not available, skipping data transmission")
            
            # 마지막 전송 시간 업데이트
            node.last_send_times['envir'] = current_time

        except Exception as e:
            node.get_logger().error(
                f"Exception during environment data processing: {str(e)}"
            )

def turtlebot_status_callback(node, msg):
    """터틀봇 상태 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()

    if current_time - node.last_send_times['turtlebot'] >= node.send_interval:
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

            node.get_logger().info(f"Turtlebot status data received: {data}")

            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }

                    # Spring 서버로 POST 요청 (타임아웃 설정)
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/turtlebot-status", 
                        json=data, 
                        headers=headers,
                        timeout=2.0
                    )

                    if response.status_code == 200:
                        node.get_logger().info(
                            "Turtlebot status data successfully sent to Spring server"
                        )
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send turtlebot status data to Spring server: {str(e)}"
                    )
            
            # 마지막 전송 시간 업데이트
            node.last_send_times['turtlebot'] = current_time

        except Exception as e:
            node.get_logger().error(
                f"Exception during turtlebot status data processing: {str(e)}"
            )

def global_path_callback(node, msg):
    """글로벌 경로 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    
    if current_time - node.last_send_times['global_path'] >= node.send_interval:
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
            
            node.get_logger().info(f"Global path data received with {len(msg.poses)} poses")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청 (타임아웃 설정)
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/global-path", 
                        json=path_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info(
                            "Global path data successfully sent to Spring server"
                        )
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send global path data to Spring server: {str(e)}"
                    )
            
            # 마지막 전송 시간 업데이트
            node.last_send_times['global_path'] = current_time
        
        except Exception as e:
            node.get_logger().error(
                f"Exception during global path data processing: {str(e)}"
            )
            
def local_path_callback(node, msg):
    """로컬 경로 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    
    if current_time - node.last_send_times['local_path'] >= node.send_interval:
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
            
            node.get_logger().info(f"Local path data received with {len(msg.poses)} poses")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청 (타임아웃 설정)
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/local-path", 
                        json=path_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info(
                            "Local path data successfully sent to Spring server"
                        )
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send local path data to Spring server: {str(e)}"
                    )
            
            # 마지막 전송 시간 업데이트
            node.last_send_times['local_path'] = current_time
        
        except Exception as e:
            node.get_logger().error(
                f"Exception during local path data processing: {str(e)}"
            )
            
def odom_callback(node, msg):
    """오도메트리 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    
    if current_time - node.last_send_times['odom'] >= node.send_interval:
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
            
            node.get_logger().info(f"Odometry data received from frame {msg.header.frame_id}")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청 (타임아웃 설정)
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/odometry", 
                        json=odom_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info(
                            "Odometry data successfully sent to Spring server"
                        )
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send odometry data to Spring server: {str(e)}"
                    )
            
            # 마지막 전송 시간 업데이트
            node.last_send_times['odom'] = current_time
        
        except Exception as e:
            node.get_logger().error(
                f"Exception during odometry data processing: {str(e)}"
            )
            
def map_callback(node, msg):
    """맵 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    map_interval = 3.0
    
    if current_time - node.last_send_times['map'] >= map_interval:
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
            
            # 맵 데이터는 크기가 매우 클 수 있으므로 주석 처리
            # 필요시 아래 코드 주석 해제 가능
            """
            # 공간 절약을 위해 비어 있는 공간(-1)은 건너뛰고 장애물만 전송
            compressed_data = []
            for i, value in enumerate(msg.data):
                if value > 0:  # 0보다 크면 점유 확률이 있는 셀
                    # 1차원 인덱스를 2차원 좌표로 변환
                    x = i % msg.info.width
                    y = i // msg.info.width
                    compressed_data.append({
                        "x": x,
                        "y": y,
                        "value": int(value)
                    })
            
            map_data["occupied_cells"] = compressed_data
            map_data["cells_count"] = len(compressed_data)
            map_data["total_cells"] = len(msg.data)
            
            # 맵의 요약 통계도 포함
            free_cells = sum(1 for val in msg.data if val == 0)
            unknown_cells = sum(1 for val in msg.data if val == -1)
            occupied_cells = sum(1 for val in msg.data if val > 0)
            
            map_data["stats"] = {
                "free": free_cells,
                "unknown": unknown_cells,
                "occupied": occupied_cells,
                "occupancy_percent": round(occupied_cells / (free_cells + occupied_cells + 0.0001) * 100, 2)
            }
            """
            
            node.get_logger().info(f"Map data received: {msg.info.width}x{msg.info.height}")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청 (타임아웃 설정)
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/map", 
                        json=map_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info(
                            "Map data successfully sent to Spring server"
                        )
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send map data to Spring server: {str(e)}"
                    )
            
            # 마지막 전송 시간 업데이트
            node.last_send_times['map'] = current_time
        
        except Exception as e:
            node.get_logger().error(
                f"Exception during map data processing: {str(e)}"
            )

def scan_callback(node, msg):
    """스캔 데이터 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    
    if current_time - node.last_send_times['scan'] >= node.send_interval:
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
            
            node.get_logger().info(f"Scan data received with {len(msg.ranges)} range points")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청 (타임아웃 설정)
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/scan", 
                        json=scan_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info(
                            "Scan data successfully sent to Spring server"
                        )
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send scan data to Spring server: {str(e)}"
                    )
            
            # 마지막 전송 시간 업데이트
            node.last_send_times['scan'] = current_time
        
        except Exception as e:
            node.get_logger().error(
                f"Exception during scan data processing: {str(e)}"
            )

def mapping_done_callback(node, msg):
    """매핑 완료 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    if current_time - node.last_send_times.get('mapping_done', 0) >= node.send_interval:
        try:
            # 매핑 데이터 준비
            mapping_data = {
                "success": True,
                "map": {
                    "info": {
                        "width": msg.map.info.width,
                        "height": msg.map.info.height,
                        "resolution": msg.map.info.resolution,
                        "origin": {
                            "position": {
                                "x": msg.map.info.origin.position.x,
                                "y": msg.map.info.origin.position.y,
                                "z": msg.map.info.origin.position.z
                            },
                            "orientation": {
                                "x": msg.map.info.origin.orientation.x,
                                "y": msg.map.info.origin.orientation.y,
                                "z": msg.map.info.origin.orientation.z,
                                "w": msg.map.info.origin.orientation.w
                            }
                        }
                    },
                    "data": list(msg.map.data)  # 바이트 배열을 리스트로 변환
                },
                "map_inflated": {
                    # inflated 맵 데이터가 있다면 여기에 추가
                }
            }
            
            node.get_logger().info("Mapping completed data received")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청 (타임아웃 설정)
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/mapping-done", 
                        json=mapping_data, 
                        headers=headers,
                        timeout=5.0  # 맵 데이터가 클 수 있으므로 타임아웃을 좀 더 길게
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info(
                            "Mapping completed data successfully sent to Spring server"
                        )
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send mapping completed data to Spring server: {str(e)}"
                    )
        
        except Exception as e:
            node.get_logger().error(
                f"Exception during mapping completed data processing: {str(e)}"
            )

def map_status_callback(node, msg):
    """맵 상태 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    
    if current_time - node.last_send_times.get('map_status', 0) >= node.send_interval:
        try:
            # 맵 상태 데이터 준비
            map_status_data = {
                "coverage": msg.coverage,
                "map_change_rate": msg.map_change_rate,
                "frontier_count": msg.frontier_count
            }
            
            node.get_logger().info(f"Map status data received: coverage={msg.coverage}%, change_rate={msg.map_change_rate}, frontiers={msg.frontier_count}")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청 (타임아웃 설정)
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/map-status", 
                        json=map_status_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info(
                            "Map status data successfully sent to Spring server"
                        )
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send map status data to Spring server: {str(e)}"
                    )
        
        except Exception as e:
            node.get_logger().error(
                f"Exception during map status data processing: {str(e)}"
            )

def obstacle_alert_callback(node, msg):
    """장애물 감지 알림 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    if current_time - node.last_send_times.get('obstacle_alert', 0) >= node.send_interval:
        try:
            # 장애물 감지 데이터 준비
            obstacle_data = {
                "detected": msg.detected,
                "distance": msg.distance
            }
            
            node.get_logger().info(f"Obstacle alert: detected={msg.detected}, distance={msg.distance}")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/obstacle-alert", 
                        json=obstacle_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info("Obstacle alert data successfully sent to Spring server")
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send obstacle alert data to Spring server: {str(e)}"
                    )
        
        except Exception as e:
            node.get_logger().error(f"Exception during obstacle alert processing: {str(e)}")

def goal_status_callback(node, msg):
    """목표 도달 상태 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    
    if current_time - node.last_send_times.get('goal_status', 0) >= node.send_interval:
        try:
            # 목표 도달 데이터 준비
            goal_data = {
                "reached": msg.reached,
                "time_taken_sec": msg.time_taken_sec
            }
            
            node.get_logger().info(f"Goal status: reached={msg.reached}, time_taken={msg.time_taken_sec}s")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/goal-status", 
                        json=goal_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info("Goal status data successfully sent to Spring server")
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send goal status data to Spring server: {str(e)}"
                    )
        
        except Exception as e:
            node.get_logger().error(f"Exception during goal status processing: {str(e)}")

def pick_done_callback(node, msg):
    """물건 집기 완료 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    
    if current_time - node.last_send_times.get('pick_done', 0) >= node.send_interval:
        try:
            # 물건 집기 완료 데이터 준비
            pick_data = {
                "success": msg.success,
                "product_id": msg.product_id,
                "timestamp": msg.timestamp
            }
            
            if msg.success:
                node.get_logger().info(f"물건 집기 성공: product_id={msg.product_id}, timestamp={msg.timestamp}")
            else:
                node.get_logger().info(f"물건 집기 실패: product_id={msg.product_id}, timestamp={msg.timestamp}")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/pick-done", 
                        json=pick_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info("Pick done data successfully sent to Spring server")
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send pick done data to Spring server: {str(e)}"
                    )
        
        except Exception as e:
            node.get_logger().error(f"Exception during pick done processing: {str(e)}")

def place_done_callback(node, msg):
    """전시 완료 알림 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    
    if current_time - node.last_send_times.get('place_done', 0) >= node.send_interval:
        try:
            # 전시 완료 데이터 준비
            place_data = {
                "success": msg.success,
                "display_spot": msg.display_spot,
                "product_id": msg.product_id
            }
            
            if msg.success:
                node.get_logger().info(f"전시 완료: product_id={msg.product_id}, display_spot={msg.display_spot}")
            else:
                node.get_logger().info(f"전시 실패: product_id={msg.product_id}, display_spot={msg.display_spot}")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    # JWT 토큰을 헤더에 추가
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    # Spring 서버로 POST 요청
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/place-done", 
                        json=place_data, 
                        headers=headers,
                        timeout=2.0
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info("Place done data successfully sent to Spring server")
                    else:
                        node.get_logger().error(
                            f"Spring server error: {response.status_code}, {response.text}"
                        )
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(
                        f"Failed to send place done data to Spring server: {str(e)}"
                    )
        
        except Exception as e:
            node.get_logger().error(f"Exception during place done processing: {str(e)}")

def image_jpeg_compressed_callback(node, msg):
    """압축된 JPEG 이미지 토픽에서 데이터를 받아 Spring 서버로 전송"""
    current_time = time.time()
    map_interval = 3.0
    
    if current_time - node.last_send_times.get('image_jpeg', 0) >= map_interval:
        try:
            # CompressedImage 메시지에서 데이터 추출
            image_data = {
                "header": {
                    "frame_id": msg.header.frame_id,
                    "stamp": {
                        "sec": msg.header.stamp.sec,
                        "nanosec": msg.header.stamp.nanosec
                    }
                },
                "format": msg.format,
                "data": base64.b64encode(msg.data).decode('utf-8')  # 바이트 배열을 base64로 인코딩
            }
            
            node.get_logger().info(f"Compressed image data received: {len(msg.data)} bytes")
            
            # JWT 토큰이 있는 경우에만 Spring 서버로 전송 시도
            if node.jwt_token:
                try:
                    headers = {
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {node.jwt_token}"
                    }
                
                    response = requests.post(
                        f"{node.spring_server_url}/api/robot/image-jpeg-compressed", 
                        json=image_data, 
                        headers=headers,
                        timeout=5.0  # 이미지 데이터가 클 수 있으므로 타임아웃을 좀 더 길게 설정
                    )
                
                    if response.status_code == 200:
                        node.get_logger().info("Compressed image data successfully sent to Spring server")
                    else:
                        node.get_logger().error(f"Spring server error: {response.status_code}, {response.text}")
                except requests.exceptions.RequestException as e:
                    node.get_logger().warning(f"Failed to send compressed image data to Spring server: {str(e)}")
            
            # 마지막 전송 시간 업데이트
            node.last_send_times['image_jpeg'] = current_time
        
        except Exception as e:
            node.get_logger().error(f"Exception during compressed image data processing: {str(e)}")