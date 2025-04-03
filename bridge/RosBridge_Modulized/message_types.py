# message_types.py
import sys

# 기본 ROS 메시지 타입 (점 없이 임포트)
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData

# SSAFY 메시지 패키지 경로 추가
sys.path.append(
    "C:/Users/SSAFY/Desktop/ros/S12P21E102/sim/ros2_ws/install/ssafy_bridge/Lib/site-packages"
)

# SSAFY 메시지 타입 임포트 시도
try:
    from ssafy_msgs.msg import EnviromentStatus, TurtlebotStatus, ScanWithPose
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
    
    CUSTOM_IMPORTS_AVAILABLE = False

def import_all_message_types():
    """임포트 성공 여부 반환"""
    return CUSTOM_IMPORTS_AVAILABLE