import rclpy
from rclpy.node import Node
from enum import Enum
from squaternion import Quaternion
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
from ssafy_msgs.msg import PickPlaceCommand, HandControl, TurtlebotStatus
from ssafy_msgs.msg import StatusStamped


class PickAndPlaceFSM(Enum):
    IDLE = 0
    GO_TO_PICK = 1
    ALIGN_OBJECT = 2
    PICK_OBJECT = 3
    GO_TO_PLACE = 4
    CHECK_RACK = 5
    ERROR_ALERT = 6
    ALIGN_RACK = 7
    PLACE_OBJECT = 8
    FINISHED = 9
    WAIT_ALIGNMENT_DONE = 10


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_and_place_node")
        self.state = PickAndPlaceFSM.IDLE

        # 퍼블리셔 & 서브스크라이버
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.hand_pub = self.create_publisher(HandControl, "/hand_control", 10)
        self.align_pub = self.create_publisher(PoseStamped, "/target_pose", 10)

        self.goal_result_sub = self.create_subscription(
            StatusStamped, "/goal_reached", self.goal_callback, 10
        )
        self.command_sub = self.create_subscription(
            PickPlaceCommand, "/pick_place_command", self.command_callback, 10
        )
        self.status_sub = self.create_subscription(
            TurtlebotStatus, "/turtlebot_status", self.status_callback, 10
        )
        self.align_done_sub = self.create_subscription(
            Bool, "/alignment_done", self.align_done_callback, 10
        )

        self.timer = self.create_timer(0.5, self.fsm_step)

        # 내부 변수
        self.goal_reached = False
        self.from_pos = None
        self.to_pos = None
        self.product_id = None
        self.display_spot = None
        self.alignment_done = False
        self.state_before_alignment = None
        self.turtlebot_status = TurtlebotStatus()

        self.hand_msg = HandControl()
        self.put_distance = 0.5
        self.put_height = 0.2

        self.placing_preview_done = False
        self.placing_done = False

    def command_callback(self, msg: PickPlaceCommand):
        self.from_pos = msg.from_pos
        self.to_pos = msg.to_pos
        self.product_id = msg.product_id
        self.display_spot = msg.display_spot
        self.goal_reached = False
        self.get_logger().info(
            f"📥 [COMMAND] Pick & Place 명령 수신: from({msg.from_pos.x}, {msg.from_pos.y}) → to({msg.to_pos.x}, {msg.to_pos.y})"
        )
        self.publish_goal_pose(self.from_pos)
        self.state = PickAndPlaceFSM.GO_TO_PICK

    def publish_goal_pose(self, target_pos: Point):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = target_pos
        pose.pose.orientation.w = 1.0
        self.goal_pub.publish(pose)
        self.get_logger().info(
            f"🗺️ [GOAL] 목표 위치 퍼블리시: ({target_pos.x:.2f}, {target_pos.y:.2f})"
        )

    def publish_target_pose(self, pos: Point, yaw: float = 0.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = pos

        q = Quaternion.from_euler(0, 0, yaw)
        pose.pose.orientation.x = q.x
        pose.pose.orientation.y = q.y
        pose.pose.orientation.z = q.z
        pose.pose.orientation.w = q.w

        self.align_pub.publish(pose)

        self.get_logger().info(
            f"🎯 [ALIGN] 정밀 정렬용 목표 퍼블리시: ({pos.x:.2f}, {pos.y:.2f})"
        )

    def goal_callback(self, msg):
        self.goal_reached = msg.status
        self.get_logger().info(f"✅ [GOAL] goal_reached 수신: {msg.status}")

    def status_callback(self, msg):
        self.turtlebot_status = msg

    def align_done_callback(self, msg):
        if msg.data:
            self.get_logger().info("✅ [ALIGNMENT] 정밀 정렬 완료 수신")
            self.alignment_done = True

    def fsm_step(self):
        self.get_logger().info(f"🔄 [FSM] 현재 상태: {self.state.name}")

        if self.state == PickAndPlaceFSM.GO_TO_PICK:
            if self.goal_reached:
                self.get_logger().info("🛬 [GO_TO_PICK] 픽업 위치 도착 → 정렬 전환")
                self.goal_reached = False
                self.alignment_done = False
                self.state_before_alignment = PickAndPlaceFSM.ALIGN_OBJECT
                self.publish_target_pose(self.from_pos)
                self.state = PickAndPlaceFSM.WAIT_ALIGNMENT_DONE

        elif self.state == PickAndPlaceFSM.GO_TO_PLACE:
            if self.goal_reached:
                self.get_logger().info("🛬 [GO_TO_PLACE] 전시 위치 도착 → 정렬 전환")
                self.goal_reached = False
                self.alignment_done = False
                self.state_before_alignment = PickAndPlaceFSM.ALIGN_RACK
                self.publish_target_pose(self.to_pos)
                self.state = PickAndPlaceFSM.WAIT_ALIGNMENT_DONE

        elif self.state == PickAndPlaceFSM.WAIT_ALIGNMENT_DONE:
            if self.alignment_done:
                self.get_logger().info(
                    "🧭 [WAIT_ALIGNMENT_DONE] 정렬 완료 → 다음 단계 전환"
                )
                self.alignment_done = False
                self.state = self.state_before_alignment

        elif self.state == PickAndPlaceFSM.ALIGN_OBJECT:
            self.get_logger().info("🔧 [ALIGN_OBJECT] 물체 정렬 수행 중 (추후 구현)")
            self.state = PickAndPlaceFSM.PICK_OBJECT

        elif self.state == PickAndPlaceFSM.PICK_OBJECT:
            if self.turtlebot_status.can_lift:
                self.get_logger().info("🤖 [PICK_OBJECT] 물체 집기 시도 중...")
                self.hand_msg.control_mode = 2
                self.hand_pub.publish(self.hand_msg)
            elif self.turtlebot_status.can_use_hand:
                self.get_logger().info("✅ [PICK_OBJECT] 집기 완료 → 이동 시작")
                self.hand_msg.put_distance = self.put_distance
                self.hand_msg.put_height = self.put_height
                self.state = PickAndPlaceFSM.GO_TO_PLACE
                self.publish_goal_pose(self.to_pos)

        elif self.state == PickAndPlaceFSM.ALIGN_RACK:
            self.get_logger().info("🔧 [ALIGN_RACK] 랙 정렬 수행 중 (추후 구현)")
            self.state = PickAndPlaceFSM.CHECK_RACK

        elif self.state == PickAndPlaceFSM.CHECK_RACK:
            self.get_logger().info("📦 [CHECK_RACK] 랙 상태 확인 중 (추후 구현)")
            self.state = PickAndPlaceFSM.PLACE_OBJECT

        elif self.state == PickAndPlaceFSM.PLACE_OBJECT:
            if not self.placing_preview_done:
                if not self.turtlebot_status.can_put:
                    self.get_logger().info("📸 [PLACE_OBJECT] 프리뷰 실행 중...")
                    self.hand_msg.control_mode = 1
                    self.hand_pub.publish(self.hand_msg)
                else:
                    self.get_logger().info("✅ [PLACE_OBJECT] 프리뷰 완료")
                    self.placing_preview_done = True
            elif not self.placing_done:
                if self.turtlebot_status.can_put:
                    self.get_logger().info("📤 [PLACE_OBJECT] 물체 내려놓기 중...")
                    self.hand_msg.control_mode = 3
                    self.hand_pub.publish(self.hand_msg)
                else:
                    self.get_logger().info("✅ [PLACE_OBJECT] 내려놓기 완료 → 종료")
                    self.placing_done = True
                    self.state = PickAndPlaceFSM.FINISHED

        elif self.state == PickAndPlaceFSM.FINISHED:
            self.get_logger().info(
                f"🏁 [FINISHED] 작업 완료. 전시 위치: ({self.to_pos.x:.2f}, {self.to_pos.y:.2f}) → IDLE 복귀"
            )
            self.placing_preview_done = False
            self.placing_done = False
            self.state = PickAndPlaceFSM.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
