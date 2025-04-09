import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, String
from enum import Enum


class Mode(Enum):
    NONE = 0
    MAPPING = 1
    PICK_AND_PLACE = 2
    STOP = 99


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__("mode_manager_node")
        self.current_mode = Mode.NONE
        self.stopped = False

        # Subscriber
        self.create_subscription(Int32, "/mode_select", self.mode_select_callback, 10)
        self.create_subscription(Bool, "/stop_all", self.stop_all_callback, 10)

        # Publisher
        self.mode_pub = self.create_publisher(String, "/current_mode", 10)

        # 타이머로 현재 모드 주기적 브로드캐스트
        self.create_timer(0.5, self.publish_current_mode)

    def mode_select_callback(self, msg):
        if self.stopped:
            self.get_logger().warn("정지 상태이므로 모드 변경 무시됨.")
            return

        try:
            selected_mode = Mode(msg.data)
        except ValueError:
            self.get_logger().warn(f"잘못된 모드 값: {msg.data}")
            return

        self.current_mode = selected_mode
        self.get_logger().info(f"모드 변경됨: {self.current_mode.name}")

    def stop_all_callback(self, msg):
        if msg.data:
            self.stopped = True
            self.current_mode = Mode.STOP
            self.get_logger().warn("전체 기능 강제 정지됨.")
        else:
            self.stopped = False
            self.current_mode = Mode.NONE
            self.get_logger().info("정지 상태 해제됨.")

    def publish_current_mode(self):
        msg = String()
        msg.data = self.current_mode.name
        self.mode_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
