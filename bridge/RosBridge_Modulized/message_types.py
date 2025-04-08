# message_types.py
import sys
# 기본 ROS 메시지 타입 (점 없이 임포트)
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Point
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import CompressedImage

# SSAFY 메시지 패키지 경로 추가
sys.path.append(
    "C:/Users/SSAFY/Desktop/ros/S12P21E102/sim/ros2_ws/install/ssafy_bridge/Lib/site-packages"
)
# SSAFY 메시지 타입 임포트 시도
try:
    # from ssafy_msgs.msg import EnviromentStatus, TurtlebotStatus, ScanWithPose, MappingDone, MapStatus, ObstacleAlert, GoalStatus, PickDone, PlaceDone, PickPlaceCommand
    from ssafy_msgs.msg import EnviromentStatus, TurtlebotStatus, ScanWithPose, MappingDone, PickPlaceCommand
    CUSTOM_IMPORTS_AVAILABLE = True
except ImportError:
    # 패키지를 가져올 수 없을 경우 대체 클래스 정의
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
            self.twist = None
            self.power_supply_status = 0
            self.battery_percentage = 0.0
            self.can_use_hand = False
            self.can_put = False
            self.can_lift = False
            
    class ScanWithPose:
        def __init__(self):
            self.header = None
            self.ranges = []
            self.intensities = []
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
    
    # 매핑 완료 결과 메시지 클래스 추가
    class MappingDone:
        def __init__(self):
            self.success = False
            self.map = None  # OccupancyGrid 타입
            self.map_inflated = None  # OccupancyGrid 타입
    
    class MapStatus:
        def __init__(self):
            self.coverage = 0.0  # 맵 커버리지 (%)
            self.map_change_rate = 0.0  # 최근 맵 변화율
            self.frontier_count = 0  # 탐색 가능한 경계 수
    
    class ObstacleAlert:
        def __init__(self):
            self.detected = False
            self.distance = 0.0
    
    class GoalStatus:
        def __init__(self):
            self.reached = False
            self.time_taken_sec = 0.0
    
    class PickDone:
        def __init__(self):
            self.success = False
            self.product_id = ""
            self.timestamp = ""
            
    class PlaceDone:
        def __init__(self):
            self.success = False
            self.display_spot = 0
            self.product_id = ""
    
    class PickPlaceCommand:
        def __init__(self):
            self.from_pos = Point()
            self.to_pos = Point()
            self.product_id = ""
            self.display_spot = 0
            
    CUSTOM_IMPORTS_AVAILABLE = False

def import_all_message_types():
    """임포트 성공 여부 반환"""
    return CUSTOM_IMPORTS_AVAILABLE

class ImageJpegCompressed(CompressedImage):
    def __init__(self):
        super().__init__()
        self.format = "jpeg"