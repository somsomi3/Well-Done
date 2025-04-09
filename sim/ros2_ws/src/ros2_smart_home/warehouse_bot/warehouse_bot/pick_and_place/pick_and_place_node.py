import rclpy
from rclpy.node import Node
from enum import Enum
from squaternion import Quaternion
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid
from ssafy_msgs.msg import PickPlaceCommand, HandControl, TurtlebotStatus, StatusStamped, PlaceDone, PickDone


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
        self.sub_goal_failed = self.create_subscription(
            StatusStamped, "/goal_failed", self.goal_failed_callback, 1
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
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.sub_map_inflated = self.create_subscription(
            OccupancyGrid, "/map_inflated", self.map_inflated_callback, 10
        )
        self.create_subscription(String, "/current_mode", self.mode_callback, 10)
        self.create_subscription(Bool, "/stop_all", self.stop_all_callback, 10)

        self.place_done_pub = self.create_publisher(PlaceDone, "/place_done", 10)
        self.pick_done_pub = self.create_publisher(PickDone, "/pick_done", 10)

        self.timer = self.create_timer(0.5, self.fsm_step)

        # 내부 변수
        self.goal_reached = False
        self.goal_failed = False
        self.from_pos = None
        self.to_pos = None
        self.product_id = None
        self.display_spot = None
        self.alignment_done = False
        self.state_before_alignment = None
        self.turtlebot_status = TurtlebotStatus()
        self.is_active = True
        self.stopped = False

        self.hand_msg = HandControl()
        self.put_distance = 0.62
        self.put_height = 0.2

        self.placing_preview_done = False
        self.placing_done = False

        self.latest_map = None
        self.latest_map_inflated = None
        self.from_id = ""
        self.to_id = ""

    def map_callback(self, msg):
        self.latest_map = msg

    def map_inflated_callback(self, msg):
        self.latest_map_inflated = msg

    def mode_callback(self, msg):
        self.is_active = msg.data == "PICK_AND_PLACE"

    def stop_all_callback(self, msg):
        self.stopped = msg.data
        if self.stopped:
            self.get_logger().warn("🛑 시스템 전체 정지 신호 수신됨. FSM 중단.")

    def command_callback(self, msg: PickPlaceCommand):
        self.from_pos = msg.from_pos
        self.to_pos = msg.to_pos
        self.product_id = msg.product_id
        self.from_id = msg.from_id
        self.to_id = msg.to_id
        self.goal_reached = False
        self.goal_failed = False
        self.get_logger().info(
            f"📥 [COMMAND] Pick & Place 명령 수신: from({msg.from_pos.position.x}, {msg.from_pos.position.y}) → to({msg.to_pos.position.x}, {msg.to_pos.position.y})"
        )
        self.publish_goal_pose(self.from_pos)
        self.state = PickAndPlaceFSM.GO_TO_PICK

    def publish_goal_pose(self, target_pose):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = target_pose
        self.goal_pub.publish(pose)
        self.get_logger().info(
            f"🗺️ [GOAL] 목표 위치 퍼블리시: ({target_pose.position.x:.2f}, {target_pose.position.y:.2f})"
        )

    def publish_target_pose(self, pose_msg):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = pose_msg

        self.align_pub.publish(pose)

        self.get_logger().info(
            f"🎯 [ALIGN] 정밀 정렬용 목표 퍼블리시: ({pose_msg.position.x:.2f}, {pose_msg.position.y:.2f})"
        )

    def goal_callback(self, msg):
        self.goal_reached = msg.status
        self.get_logger().info(f"✅ [GOAL] goal_reached 수신: {msg.status}")

    def goal_failed_callback(self, msg):
        if msg.status:
            self.get_logger().warn(
                "⚠️ [GOAL] 목표 지점 도달 신뢰 실패 → 동일 목표 재시도"
            )
            if self.state == PickAndPlaceFSM.GO_TO_PICK:
                self.publish_goal_pose(self.from_pos)
            elif self.state == PickAndPlaceFSM.GO_TO_PLACE:
                self.publish_goal_pose(self.to_pos)

    def status_callback(self, msg):
        self.turtlebot_status = msg

    def align_done_callback(self, msg):
        if msg.data:
            self.get_logger().info("✅ [ALIGNMENT] 정밀 정렬 완료 수신")
            self.alignment_done = True

    def fsm_step(self):
        if not self.is_active or self.stopped:
            return

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

                msg = PickDone()
                msg.success = True
                msg.product_id = self.product_id
                msg.from_id = self.from_id
                msg.map = self.latest_map if self.latest_map else OccupancyGrid()
                msg.map_inflated = self.latest_map_inflated if self.latest_map_inflated else OccupancyGrid()
                self.pick_done_pub.publish(msg)
                self.get_logger().info("📦 [PICK_DONE] 집기 완료 메시지 발행됨.")
                
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
                f"🏁 [FINISHED] 작업 완료. 전시 위치: ({self.to_pos.position.x:.2f}, {self.to_pos.position.y:.2f}) → IDLE 복귀"
            )
            self.placing_preview_done = False
            self.placing_done = False

            msg = PlaceDone()
            msg.success = True
            msg.product_id = self.product_id
            msg.to_id = self.to_id
            msg.map = self.latest_map if self.latest_map else OccupancyGrid()
            msg.map_inflated = self.latest_map_inflated if self.latest_map_inflated else OccupancyGrid()
            self.place_done_pub.publish(msg)

            self.state = PickAndPlaceFSM.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
