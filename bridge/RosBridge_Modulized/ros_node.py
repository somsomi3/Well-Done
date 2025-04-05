# ros node
# 파일 상단에 다음 임포트 구문 추가
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
import time
import requests

# ROS 메시지 타입 직접 임포트
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData

# SSAFY 메시지 타입 임포트 시도
try:
    from ssafy_msgs.msg import EnviromentStatus, TurtlebotStatus, ScanWithPose, MappingDone, MapStatus, ObstacleAlert, GoalStatus, PickDone, PlaceDone
except ImportError:
    # 없으면 message_types.py에서 가져옴
    from .message_types import EnviromentStatus, TurtlebotStatus, ScanWithPose, MappingDone, MapStatus, ObstacleAlert, GoalStatus, PickDone, PlaceDone

# 내부 모듈 임포트
from .handlers.topic_callbacks import register_all_callbacks
from .handlers.command_handlers import register_all_command_handlers
from .utils.auth import get_jwt_token

class RobotBridgeNode(Node):
    def __init__(self):
        super().__init__("robot_bridge_node")
        
        # 시간 간격 추적 설정
        self.last_send_times = {
            'envir': 0.0,
            'turtlebot': 0.0,
            'global_path': 0.0,
            'local_path': 0.0,
            'odom': 0.0,
            'scan': 0.0,
            'map': 0.0
        }
        self.send_interval = 1.0

        # 명령 큐 설정
        self.command_queue = []
        self.queue_lock = threading.Lock()
        self.command_timer = self.create_timer(0.1, self.process_commands)

        # self.publishers = {}

        # Spring 서버 URL 및 인증
        self.spring_server_url = "http://172.26.15.101:8080"
        self.jwt_token = None
        self.token_refresh_timer = None
        self.attempt_jwt_token_acquisition()

        # QoS 프로필 설정
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 콜백 핸들러 먼저 등록 (중요!)
        register_all_callbacks(self)
        
        # 명령 핸들러 등록
        register_all_command_handlers(self)
        
        # 콜백이 등록된 후에 발행자와 구독자 초기화
        self.setup_publishers()
        self.setup_subscribers()
        
        self.get_logger().info("Robot bridge node initialized")

    def attempt_jwt_token_acquisition(self):
        """스프링 서버 연결 및 JWT 토큰 획득 시도"""
        self.jwt_token = get_jwt_token(self)
        
        if self.jwt_token is None:
            self.get_logger().warn("스프링 서버에 연결할 수 없습니다. 토큰 없이 계속합니다.")
            self.get_logger().info("30초 후 토큰 획득을 다시 시도합니다.")
            
            # 기존 타이머가 있으면 취소
            if self.token_refresh_timer:
                self.token_refresh_timer.cancel()
            
            # 30초 후 다시 시도하는 타이머 설정
            self.token_refresh_timer = self.create_timer(30.0, self.attempt_jwt_token_acquisition)
        else:
            self.get_logger().info("스프링 서버에 연결되었습니다.")
            
            # 토큰을 얻었으므로 재시도 타이머 취소
            if self.token_refresh_timer:
                self.token_refresh_timer.cancel()
                self.token_refresh_timer = None

    def setup_publishers(self):
        """모든 발행자 설정"""
        # 개별 발행자 속성으로 설정
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', self.qos_profile)
        self.global_path_publisher = self.create_publisher(Path, '/global_path', self.qos_profile)
        self.local_path_publisher = self.create_publisher(Path, '/local_path', self.qos_profile)
        self.odom_publisher = self.create_publisher(Odometry, '/odom_true', self.qos_profile)
        self.scan_publisher = self.create_publisher(ScanWithPose, '/scan_with_pose', self.qos_profile)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', self.qos_profile)
        self.start_auto_map_publisher = self.create_publisher(Bool, '/start_auto_map', self.qos_profile)
        self.stop_auto_map_publisher = self.create_publisher(Bool, '/stop_auto_map', self.qos_profile)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', self.qos_profile)
        
        # 딕셔너리 업데이트
        # self.publishers.update({
        #     'cmd_vel': self.cmd_vel_publisher,
        #     'global_path': self.global_path_publisher,
        #     'local_path': self.local_path_publisher,
        #     'odom': self.odom_publisher,
        #     'scan': self.scan_publisher,
        #     'map': self.map_publisher,
        #     'start_auto_map': self.start_auto_map_publisher,
        #     'stop_auto_map': self.stop_auto_map_publisher
        # })

        self.get_logger().info("모든 발행자가 성공적으로 설정되었습니다.")
    def setup_subscribers(self):
        """Set up all subscribers"""
        self.subscribers = {
            # 환경 상태 및 터틀봇 상태 구독
            'envir_status': self.create_subscription(
                EnviromentStatus, "/envir_status", self.envir_status_callback, self.qos_profile
            ),
            'turtlebot_status': self.create_subscription(
                TurtlebotStatus, "/turtlebot_status", self.turtlebot_status_callback, self.qos_profile
            ),
            
            # 경로 관련 토픽 구독
            'global_path': self.create_subscription(
                Path, "/global_path", self.global_path_callback, self.qos_profile
            ),
            'local_path': self.create_subscription(
                Path, "/local_path", self.local_path_callback, self.qos_profile
            ),
            
            # 오도메트리, 스캔, 맵 구독
            'odom': self.create_subscription(
                Odometry, "/odom_true", self.odom_callback, self.qos_profile
            ),
            'scan': self.create_subscription(
                ScanWithPose, "/scan_with_pose", self.scan_callback, self.qos_profile
            ),
            'map': self.create_subscription(
                OccupancyGrid, "/map", self.map_callback, self.qos_profile
            ),
            'mapping_done': self.create_subscription(
                MappingDone, "/mapping_done", self.mapping_done_callback, self.qos_profile
            ),
            'map_status': self.create_subscription(
                MapStatus, "/map_status", self.map_status_callback, self.qos_profile
            ),
            'obstacle_alert': self.create_subscription(
                ObstacleAlert, "/obstacle_alert", self.obstacle_alert_callback, self.qos_profile
            ),
            'goal_status': self.create_subscription(
                GoalStatus, "/goal_status", self.goal_status_callback, self.qos_profile
            ),
            'pick_done': self.create_subscription(
                PickDone, "/pick_done", self.pick_done_callback, self.qos_profile
            ),
            'place_done': self.create_subscription(
                PlaceDone, "/place_done", self.place_done_callback, self.qos_profile
            )
        }

    def process_commands(self):
        """Process commands from the queue"""
        if not self.command_queue:
            return
        
        with self.queue_lock:
            if self.command_queue:
                command = self.command_queue.pop(0)
                
                # Call the appropriate handler method based on command type
                handler_method = f"execute_{command['type']}_command"
                if hasattr(self, handler_method):
                    getattr(self, handler_method)(command)
                else:
                    self.get_logger().error(f"Unknown command type: {command['type']}")