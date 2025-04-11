# 주요 클래스와 함수를 공개 인터페이스로 노출
from .ros_node import RobotBridgeNode
from .flask_server import run_flask_server, set_node_reference

# 버전 정보 등의 메타데이터
__version__ = '0.4.0'