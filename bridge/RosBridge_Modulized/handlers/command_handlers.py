# command_handlers.py
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header, Bool

# SSAFY 메시지 타입 임포트 시도
try:
    from ssafy_msgs.msg import ScanWithPose
except ImportError:
    # 없으면 상위 모듈에서 가져옴
    from ..message_types import ScanWithPose

import random  # 스캔 명령에서 랜덤 데이터 생성에 필요

def register_all_command_handlers(node):
    """모든 명령 핸들러 메서드를 노드에 등록"""
    node.execute_move_command = lambda cmd: execute_move_command(node, cmd)
    node.execute_grab_command = lambda cmd: execute_grab_command(node, cmd)
    node.execute_global_path_command = lambda cmd: execute_global_path_command(node, cmd)
    node.execute_local_path_command = lambda cmd: execute_local_path_command(node, cmd)
    node.execute_odom_command = lambda cmd: execute_odom_command(node, cmd)
    node.execute_scan_command = lambda cmd: execute_scan_command(node, cmd)
    node.execute_map_command = lambda cmd: execute_map_command(node, cmd)
    node.execute_start_auto_map_command = lambda cmd: execute_start_auto_map_command(node, cmd)
    node.execute_stop_auto_map_command = lambda cmd: execute_stop_auto_map_command(node, cmd)
    node.execute_goal_pose_command = lambda cmd: execute_goal_pose_command(node, cmd)
    node.execute_pick_place_command = lambda cmd: execute_pick_place_command(node, cmd)

def execute_move_command(node, command):
    """이동 명령 실행"""
    try:
        # 명령 타입 및 내용 로깅
        node.get_logger().info(f"Move command received: {command}")
        
        twist = Twist()
        
        # 명령이 딕셔너리인지 확인
        if isinstance(command, dict):
            twist.linear.x = command.get('linear_x', 0.0)
            twist.angular.z = command.get('angular_z', 0.0)
        else:
            # 딕셔너리가 아닌 경우 명시적으로 딕셔너리로 변환 시도
            try:
                # 제너레이터나 이터러블인 경우 리스트로 변환 후 접근
                command_dict = dict(command)
                twist.linear.x = command_dict.get('linear_x', 0.0)
                twist.angular.z = command_dict.get('angular_z', 0.0)
            except (TypeError, ValueError):
                # 변환 실패 시 기본값 사용
                node.get_logger().warning(f"Command cannot be converted to dictionary: {type(command)}")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        
        # 메시지 발행
        node.cmd_vel_publisher.publish(twist)
        node.get_logger().info(f"Move command executed: linear_x={twist.linear.x}, angular_z={twist.angular.z}")
    except Exception as e:
        node.get_logger().error(f"Error executing move command: {str(e)}")
        # 스택 트레이스 출력 (더 자세한 오류 정보)
        import traceback
        node.get_logger().error(traceback.format_exc())

def execute_grab_command(node, command):
    """잡기/놓기 명령 실행"""
    try:
        # 여기에 잡기/놓기 명령 실행 코드 구현
        action = command.get('action', 'grab')
        target = command.get('target', 'unknown')
        node.get_logger().info(f"Grab command executed: action={action}, target={target}")
    except Exception as e:
        node.get_logger().error(f"Error executing grab command: {str(e)}")

def execute_global_path_command(node, command):
    """글로벌 경로 명령 실행"""
    try:
        # 새 Path 메시지 생성
        path_msg = Path()
        
        # Header 설정
        header = Header()
        header.stamp = node.get_clock().now().to_msg()
        header.frame_id = command.get('frame_id', 'map')
        path_msg.header = header
        
        # Poses 배열 설정
        poses = command.get('poses', [])
        for pose_data in poses:
            pose_stamped = PoseStamped()
            
            # PoseStamped의 Header 설정
            pose_header = Header()
            pose_header.stamp = node.get_clock().now().to_msg()
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
        node.global_path_publisher.publish(path_msg)
        node.get_logger().info(f"Global path command executed with {len(poses)} poses")
        
    except KeyError:
        node.get_logger().error("global_path publisher not found in publishers dictionary")
    except Exception as e:
        node.get_logger().error(f"Error executing global path command: {str(e)}")
        
def execute_local_path_command(node, command):
    """로컬 경로 명령 실행"""
    try:
        # 새 Path 메시지 생성
        path_msg = Path()
        
        # Header 설정
        header = Header()
        header.stamp = node.get_clock().now().to_msg()
        header.frame_id = command.get('frame_id', 'map')
        path_msg.header = header
        
        # Poses 배열 설정
        poses = command.get('poses', [])
        for pose_data in poses:
            pose_stamped = PoseStamped()
            
            # PoseStamped의 Header 설정
            pose_header = Header()
            pose_header.stamp = node.get_clock().now().to_msg()
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
        node.local_path_publisher.publish(path_msg)
        node.get_logger().info(f"Local path command executed with {len(poses)} poses")
        
    except KeyError:
        node.get_logger().error("local_path publisher not found in publishers dictionary")
    except Exception as e:
        node.get_logger().error(f"Error executing local path command: {str(e)}")
        
def execute_odom_command(node, command):
    """오도메트리 명령 실행 (주로 테스트 또는 시뮬레이션 목적)"""
    try:
        # 새 Odometry 메시지 생성
        odom_msg = Odometry()
        
        # Header 설정
        header = Header()
        header.stamp = node.get_clock().now().to_msg()
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
        node.odom_publisher.publish(odom_msg)
        node.get_logger().info(f"Odometry command executed from {odom_msg.header.frame_id} to {odom_msg.child_frame_id}")
        
    except KeyError:
        node.get_logger().error("odom publisher not found in publishers dictionary")
    except Exception as e:
        node.get_logger().error(f"Error executing odometry command: {str(e)}")
        
def execute_map_command(node, command):
    """맵 명령 실행 (주로 테스트 또는 시뮬레이션 목적)"""
    try:
        # 새 OccupancyGrid 메시지 생성
        map_msg = OccupancyGrid()
        
        # Header 설정
        header = Header()
        header.stamp = node.get_clock().now().to_msg()
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
        node.map_publisher.publish(map_msg)
        node.get_logger().info(f"Map command executed, created {map_info.width}x{map_info.height} map")
        
    except KeyError:
        node.get_logger().error("map publisher not found in publishers dictionary")
    except Exception as e:
        node.get_logger().error(f"Error executing map command: {str(e)}")
        
def execute_scan_command(node, command):
    """스캔 명령 실행 (주로 테스트 또는 시뮬레이션 목적)"""
    try:
        # 새 ScanWithPose 메시지 생성
        scan_msg = ScanWithPose()
        
        # Header 설정
        header = Header()
        header.stamp = node.get_clock().now().to_msg()
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
            for i in range(num_rays):
                if random.random() < 0.3:  # 30% 확률로 장애물 시뮬레이션
                    ranges[i] = random.uniform(0.5, 4.0)
        
        # 강도 데이터가 없는 경우
        if not intensities and ranges:
            intensities = [100.0] * len(ranges)  # 기본 강도 값
        
        scan_msg.ranges = ranges
        scan_msg.intensities = intensities
        
        # ScanWithPose 메시지 발행
        node.scan_publisher.publish(scan_msg)
        node.get_logger().info(f"Scan command executed with {len(ranges)} range points")
        
    except KeyError:
        node.get_logger().error("scan publisher not found in publishers dictionary")
    except Exception as e:
        node.get_logger().error(f"Error executing scan command: {str(e)}")

def execute_start_auto_map_command(node, command):
    """자동 매핑 시작 명령 실행"""
    try:
        # Bool 메시지 생성
        from std_msgs.msg import Bool
        bool_msg = Bool()
        bool_msg.data = command.get('data', True)  # 기본값은 True
        
        # 발행자가 딕셔너리로 존재하는지 확인
        if hasattr(node, 'publishers') and 'start_auto_map' in node.publishers:
            node.start_auto_map_publisher.publish(bool_msg)
        # 개별 발행자로 존재하는지 확인
        elif hasattr(node, 'start_auto_map_publisher'):
            node.start_auto_map_publisher.publish(bool_msg)
        else:
            node.get_logger().error("Cannot find start_auto_map publisher")
            return
            
        node.get_logger().info(f"Auto mapping command executed: data={bool_msg.data}")
    except Exception as e:
        node.get_logger().error(f"Error executing auto mapping command: {str(e)}")
        import traceback
        node.get_logger().error(traceback.format_exc())

def execute_stop_auto_map_command(node, command):
    """자동 매핑 끄기 명령 실행"""
    try:
        # Bool 메시지 생성
        from std_msgs.msg import Bool
        bool_msg = Bool()
        bool_msg.data = command.get('data', True)  # 기본값은 True
        
        # 발행자가 딕셔너리로 존재하는지 확인
        if hasattr(node, 'publishers') and 'stop_auto_map' in node.publishers:
            node.stop_auto_map_publisher.publish(bool_msg)
        # 개별 발행자로 존재하는지 확인
        elif hasattr(node, 'stop_auto_map_publisher'):
            node.stop_auto_map_publisher.publish(bool_msg)
        else:
            node.get_logger().error("Cannot find stop_auto_map publisher")
            return
            
        node.get_logger().info(f"Auto mapping stop command executed: data={bool_msg.data}")
    except Exception as e:
        node.get_logger().error(f"Error executing auto mapping stop command: {str(e)}")
        import traceback
        node.get_logger().error(traceback.format_exc())

def execute_goal_pose_command(node, command):
    """목적지 설정 명령 실행"""
    try:
        # 명령 데이터 추출
        position_data = command.get('position', {})
        position_x = position_data.get('x', 0.0)
        position_y = position_data.get('y', 0.0)
        orientation = command.get('orientation', 0.0)
        
        node.get_logger().info(f"Goal pose command received: x={position_x}, y={position_y}, orientation={orientation}")
        
        # PoseStamped 메시지 생성
        from geometry_msgs.msg import PoseStamped
        pose_msg = PoseStamped()
        
        # Header 설정
        pose_msg.header.stamp = node.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"  # 맵 프레임 기준
        
        # Position 설정
        pose_msg.pose.position.x = position_x
        pose_msg.pose.position.y = position_y
        pose_msg.pose.position.z = 0.0  # 일반적으로 2D 네비게이션에서는 z=0
        
        # Orientation 설정 (Quaternion으로 변환)
        import math
        yaw = orientation  # 라디안 단위로 가정
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 발행자가 있는지 확인
        if hasattr(node, 'goal_pose_publisher'):
            node.goal_pose_publisher.publish(pose_msg)
            node.get_logger().info(f"Goal pose command executed: x={position_x}, y={position_y}, orientation={orientation}")
        else:
            node.get_logger().error("Cannot find goal_pose publisher")
    except Exception as e:
        node.get_logger().error(f"Error executing goal pose command: {str(e)}")
        import traceback
        node.get_logger().error(traceback.format_exc())

def execute_pick_place_command(node, cmd):
    """pick_place_command 실행 - 세타값(각도)을 쿼터니언으로 변환"""
    if not hasattr(node, 'pick_place_command_publisher'):
        node.get_logger().error("pick_place_command publisher not available")
        return {"result": False, "message": "pick_place_command publisher not available"}

    try:
        # 백엔드에서 전달받은 데이터 추출
        from_pos = cmd.get('from_pos', {})
        to_pos = cmd.get('to_pos', {})
        product_id = cmd.get('product_id', '')
        display_spot = cmd.get('display_spot', 0)
        
        # 새 PickPlaceCommand 메시지 생성
        msg = PickPlaceCommand()
        
        # 메시지 Header 설정
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # 또는 적절한 프레임 ID
        
        # from_pos 설정 (세타값을 쿼터니언으로 변환)
        import math
        
        # from_pos의 위치 정보 설정
        msg.from_pos.position.x = from_pos.get('position', {}).get('x', 0.0)
        msg.from_pos.position.y = from_pos.get('position', {}).get('y', 0.0)
        msg.from_pos.position.z = from_pos.get('position', {}).get('z', 0.0)
        
        # from_pos의 방향 정보 설정 (세타 -> 쿼터니언)
        from_theta = from_pos.get('theta', 0.0)  # 세타 값(라디안)
        msg.from_pos.orientation.x = 0.0
        msg.from_pos.orientation.y = 0.0
        msg.from_pos.orientation.z = math.sin(from_theta / 2.0)
        msg.from_pos.orientation.w = math.cos(from_theta / 2.0)
        
        # to_pos 설정 (세타값을 쿼터니언으로 변환)
        msg.to_pos.position.x = to_pos.get('position', {}).get('x', 0.0)
        msg.to_pos.position.y = to_pos.get('position', {}).get('y', 0.0)
        msg.to_pos.position.z = to_pos.get('position', {}).get('z', 0.0)
        
        # to_pos의 방향 정보 설정 (세타 -> 쿼터니언)
        to_theta = to_pos.get('theta', 0.0)  # 세타 값(라디안)
        msg.to_pos.orientation.x = 0.0
        msg.to_pos.orientation.y = 0.0
        msg.to_pos.orientation.z = math.sin(to_theta / 2.0)
        msg.to_pos.orientation.w = math.cos(to_theta / 2.0)
        
        # 제품 ID와 진열대 번호 설정
        msg.product_id = product_id
        msg.display_spot = display_spot
        
        # 디버깅 정보 로깅
        node.get_logger().debug(f"From position: x={msg.from_pos.position.x}, y={msg.from_pos.position.y}")
        node.get_logger().debug(f"From theta: {from_theta} rad, converted to quaternion: z={msg.from_pos.orientation.z}, w={msg.from_pos.orientation.w}")
        node.get_logger().debug(f"To position: x={msg.to_pos.position.x}, y={msg.to_pos.position.y}")
        node.get_logger().debug(f"To theta: {to_theta} rad, converted to quaternion: z={msg.to_pos.orientation.z}, w={msg.to_pos.orientation.w}")
        
        # 메시지 발행
        node.pick_place_command_publisher.publish(msg)
        node.get_logger().info(f"Published pick_place_command for product {product_id} to display spot {display_spot}")
        
        return {"result": True, "message": "Pick and place command published successfully"}
    except Exception as e:
        node.get_logger().error(f"Error executing pick_place_command: {str(e)}")
        import traceback
        node.get_logger().error(traceback.format_exc())
        return {"result": False, "message": f"Error: {str(e)}"}
