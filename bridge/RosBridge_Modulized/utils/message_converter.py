# message_converter.py
import math

def path_to_json(path_msg):
    """Path 메시지를 JSON 구조로 변환"""
    path_data = {
        "poses": []
    }
    
    for pose in path_msg.poses:
        pose_data = {
            "pose": {
                "position": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": pose.pose.position.z
                }
            }
        }
        path_data["poses"].append(pose_data)
    
    path_data["path_length"] = len(path_msg.poses)
    return path_data

def odometry_to_json(odom_msg):
    """Odometry 메시지를 JSON 구조로 변환"""
    odom_data = {
        "pose": {
            "pose": {
                "position": {
                    "x": odom_msg.pose.pose.position.x,
                    "y": odom_msg.pose.pose.position.y,
                    "z": odom_msg.pose.pose.position.z
                }
            }
        },
        "twist": {
            "twist": {
                "linear": {
                    "x": odom_msg.twist.twist.linear.x
                },
                "angular": {
                    "z": odom_msg.twist.twist.angular.z
                }
            }
        }
    }
    
    # 공분산 행렬 추가
    odom_data["pose"]["covariance"] = list(odom_msg.pose.covariance)
    odom_data["twist"]["covariance"] = list(odom_msg.twist.covariance)
    
    return odom_data

def scan_to_json(scan_msg, include_obstacles=True, max_obstacles=10):
    """ScanWithPose 메시지를 JSON 구조로 변환"""
    scan_data = {
        "pose": {
            "x": scan_msg.pose_x,
            "y": scan_msg.pose_y,
            "theta": scan_msg.pose_theta
        }
    }
    
    # 장애물 정보 추가 (옵션)
    if include_obstacles and hasattr(scan_msg, 'ranges') and len(scan_msg.ranges) > 0:
        obstacles = []
        angle = scan_msg.angle_min
        
        for i, r in enumerate(scan_msg.ranges):
            if r < scan_msg.range_max and r > scan_msg.range_min:
                # 장애물 위치 계산 (로봇 중심 기준)
                obstacle_x = scan_msg.pose_x + r * math.cos(angle)
                obstacle_y = scan_msg.pose_y + r * math.sin(angle)
                obstacles.append({
                    "x": obstacle_x,
                    "y": obstacle_y,
                    "distance": r
                })
                
                # 지정된 개수만큼만 장애물 정보 추가
                if len(obstacles) >= max_obstacles:
                    break
                    
            angle += scan_msg.angle_increment
            
        scan_data["obstacles"] = obstacles
    
    return scan_data

def map_to_json(map_msg, include_cells=False, max_cells=1000):
    """OccupancyGrid 메시지를 JSON 구조로 변환"""
    map_data = {
        "info": {
            "width": map_msg.info.width,
            "height": map_msg.info.height,
            "resolution": map_msg.info.resolution,
            "origin": {
                "position": {
                    "x": map_msg.info.origin.position.x,
                    "y": map_msg.info.origin.position.y,
                    "z": map_msg.info.origin.position.z
                },
                "orientation": {
                    "x": map_msg.info.origin.orientation.x,
                    "y": map_msg.info.origin.orientation.y
                }
            }
        }
    }
    
    # 셀 정보 추가 (옵션)
    if include_cells:
        compressed_data = []
        cell_count = 0
        
        for i, value in enumerate(map_msg.data):
            if value > 0:  # 점유된 셀만 추가
                # 1차원 인덱스를 2차원 좌표로 변환
                x = i % map_msg.info.width
                y = i // map_msg.info.width
                compressed_data.append({
                    "x": x,
                    "y": y,
                    "value": int(value)
                })
                
                cell_count += 1
                if cell_count >= max_cells:
                    break
        
        map_data["occupied_cells"] = compressed_data
        map_data["cells_count"] = len(compressed_data)
        map_data["total_cells"] = len(map_msg.data)
        
        # 맵 통계 정보 추가
        free_cells = sum(1 for val in map_msg.data if val == 0)
        unknown_cells = sum(1 for val in map_msg.data if val == -1)
        occupied_cells = sum(1 for val in map_msg.data if val > 0)
        
        map_data["stats"] = {
            "free": free_cells,
            "unknown": unknown_cells,
            "occupied": occupied_cells,
            "occupancy_percent": round(occupied_cells / (free_cells + occupied_cells + 0.0001) * 100, 2)
        }
    
    return map_data

def envir_status_to_json(envir_msg):
    """EnviromentStatus 메시지를 JSON 구조로 변환"""
    return {
        "month": envir_msg.month,
        "day": envir_msg.day,
        "hour": envir_msg.hour,
        "minute": envir_msg.minute,
        "temperature": envir_msg.temperature,
        "weather": envir_msg.weather
    }

def turtlebot_status_to_json(turtlebot_msg):
    """TurtlebotStatus 메시지를 JSON 구조로 변환"""
    data = {
        "battery_percentage": turtlebot_msg.battery_percentage,
        "power_supply_status": turtlebot_msg.power_supply_status,
        "can_use_hand": turtlebot_msg.can_use_hand,
        "can_put": turtlebot_msg.can_put,
        "can_lift": turtlebot_msg.can_lift
    }
    
    # Twist 데이터 추가 (있는 경우)
    if hasattr(turtlebot_msg, 'twist') and turtlebot_msg.twist is not None:
        # Linear 속도
        if hasattr(turtlebot_msg.twist, 'linear'):
            data['linear_x'] = getattr(turtlebot_msg.twist.linear, 'x', 0.0)
        
        # Angular 속도
        if hasattr(turtlebot_msg.twist, 'angular'):
            data['angular_z'] = getattr(turtlebot_msg.twist.angular, 'z', 0.0)
    
    return data