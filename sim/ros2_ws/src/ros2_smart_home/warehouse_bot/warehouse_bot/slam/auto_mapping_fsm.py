import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ssafy_msgs.msg import MappingDone
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from ssafy_msgs.msg import StatusStamped
from squaternion import Quaternion
from warehouse_bot.utils.frontier_utils import (
    find_frontiers,
    grid_to_world,
    is_within_fov,
)
from warehouse_bot.utils.sim_config import MAP_PATH, params_map
from warehouse_bot.utils.logger_utils import print_log
import numpy as np


def get_heading(msg):
    q = msg.pose.pose.orientation
    quat = Quaternion(q.w, q.x, q.y, q.z)
    _, _, heading = quat.to_euler()
    return heading


class AutoMappingFSM(Node):
    def __init__(self):
        super().__init__("auto_mapping_fsm")

        # FSM 상태 정의
        self.state = "WAIT_FOR_COMMAND"

        # 내부 상태
        self.file_tag = "auto_mapping"
        self.map_data = None
        self.prev_map = None
        self.last_change_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.map_info = None
        self.current_pose = None
        self.frontiers = []
        self.prev_goal = None
        self.failed_goals = []

        self.last_goal_reach_time = None

        self.raw_map_msg = None

        # 종료 조건 파라미터
        self.MAP_CHANGE_THRESHOLD = 0.01
        self.MAP_COVERAGE_THRESHOLD = 0.60
        self.MAP_IDLE_DURATION = 5.0  # seconds

        # 퍼블리셔/서브스크라이버
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.done_pub = self.create_publisher(MappingDone, "/mapping_done", 1)
        self.pub_reset = self.create_publisher(Bool, "/reset_mapping", 1)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.sub_start = self.create_subscription(
            Bool, "/start_auto_map", self.start_callback, 1
        )
        self.sub_raw_map = self.create_subscription(
            OccupancyGrid, "/map", self.raw_map_callback, 10
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map_inflated", self.map_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, 10
        )
        self.sub_plan_done = self.create_subscription(
            StatusStamped, "/plan_success", self.plan_success_callback, 1
        )
        self.sub_plan_failed = self.create_subscription(
            StatusStamped, "/plan_failed", self.plan_failed_callback, 1
        )
        self.sub_goal_reached = self.create_subscription(
            StatusStamped, "/goal_reached", self.goal_reached_callback, 1
        )
        self.sub_goal_failed = self.create_subscription(
            StatusStamped, "/goal_failed", self.goal_failed_callback, 1
        )
        self.sub_stop = self.create_subscription(
            Bool, "/stop_auto_map", self.stop_callback, 1
        )

        # 상태 전이 확인용 타이머
        self.running = False
        self.timer = self.create_timer(1.0, self.fsm_step)

        print_log(
            "info",
            self.get_logger(),
            "AutoMapping FSM Node initialized.",
            file_tag=self.file_tag,
        )

    def start_callback(self, msg):
        if msg.data and self.state == "WAIT_FOR_COMMAND":
            print_log(
                "info",
                self.get_logger(),
                "▶️ Start command received. Moving to FRONTIER_SEARCH.",
                file_tag=self.file_tag,
            )

            self.pub_reset.publish(Bool(data=True))
            print_log(
                "warn",
                self.get_logger(),
                "🧼 Sent reset_mapping signal to run_mapping.",
                file_tag=self.file_tag,
            )

            self.prev_goal = None
            self.failed_goals = []
            self.prev_map = None
            self.map_data = None
            self.running = False

            self.state = "FRONTIER_SEARCH"

    def stop_callback(self, msg):
        if msg.data:
            print_log(
                "warn",
                self.get_logger(),
                "🛑 Stop command received. Switching to WAIT_FOR_COMMAND.",
                file_tag=self.file_tag,
            )
            self.state = "WAIT_FOR_COMMAND"
            self.running = False
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.cmd_pub.publish(stop_twist)
            self.prev_goal = None

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = get_heading(msg)
        self.current_pose = [x, y, theta]

    def raw_map_callback(self, msg):
        self.raw_map_msg = msg

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if self.prev_map is not None:
            diff = np.abs(self.map_data - self.prev_map)
            change_rate = np.count_nonzero(diff) / diff.size
            # print_log(
            #     "info",
            #     self.get_logger(),
            #     f"[MAP] Change rate: {change_rate:.4f}",
            #     file_tag=self.file_tag,
            # )

            observed = (self.map_data == 0) | (self.map_data > 70)
            coverage = np.count_nonzero(observed) / self.map_data.size
            # print_log(
            #     "info",
            #     self.get_logger(),
            #     f"[MAP] Coverage: {coverage:.2%}",
            #     file_tag=self.file_tag,
            # )

            if change_rate > self.MAP_CHANGE_THRESHOLD:
                self.last_change_time = now

            duration = now - self.last_change_time
            if self.state == "WAIT_FOR_FRONTIER_CHECK":
                if (
                    duration > self.MAP_IDLE_DURATION
                    and coverage > self.MAP_COVERAGE_THRESHOLD
                ):
                    print_log(
                        "info",
                        self.get_logger(),
                        "✅ Mapping complete by coverage/stability. Sending done signal.",
                        file_tag=self.file_tag,
                    )
                    # ✅ 매핑 완료 메시지 구성
                    done_msg = MappingDone()
                    done_msg.header.stamp = self.get_clock().now().to_msg()
                    done_msg.header.frame_id = "map"
                    done_msg.success = True
                    done_msg.map = self.raw_map_msg  # /map
                    done_msg.map_inflated = msg  # /map_inflated (현재 콜백 메시지)

                    self.done_pub.publish(done_msg)

                    self.state = "WAIT_FOR_COMMAND"
                elif duration > self.MAP_IDLE_DURATION:
                    print_log(
                        "info",
                        self.get_logger(),
                        "ℹ️ Coverage 부족. 다음 frontier 탐색으로 복귀.",
                        file_tag=self.file_tag,
                    )
                    self.state = "FRONTIER_SEARCH"

        self.prev_map = self.map_data.copy()

    def plan_success_callback(self, msg):
        if self.state == "WAIT_FOR_COMMAND":
            return
        print_log(
            "info",
            self.get_logger(),
            f"[FSM] plan_success_callback received. State: {self.state}",
            file_tag=self.file_tag,
        )
        if self.state == "WAIT_FOR_PLAN_RESULT" and msg.status:
            print_log(
                "info",
                self.get_logger(),
                "✅ Plan success. Waiting for goal result.",
                file_tag=self.file_tag,
            )
            self.failed_goals = []
            self.state = "WAIT_FOR_GOAL_RESULT"
            print_log(
                "info",
                self.get_logger(),
                f"[FSM] State changed to: {self.state}",
                file_tag=self.file_tag,
            )
            self.last_goal_reach_time = None

    def plan_failed_callback(self, msg):
        if self.state == "WAIT_FOR_COMMAND":
            return
        if self.state == "WAIT_FOR_PLAN_RESULT" and msg.status:
            print_log(
                "warn",
                self.get_logger(),
                "❌ Plan failed. Retrying frontier search.",
                file_tag=self.file_tag,
            )
            if self.prev_goal is not None:
                self.failed_goals.append(self.prev_goal)  # ❗ 실패한 목표 추가
                print_log(
                    "warn",
                    self.get_logger(),
                    f"⚠️ Added to blacklist: {self.prev_goal}",
                    file_tag=self.file_tag,
                )
            self.state = "FRONTIER_SEARCH"

    def goal_reached_callback(self, msg):
        if self.state == "WAIT_FOR_COMMAND":
            return
        print_log(
            "info",
            self.get_logger(),
            f"[FSM] goal_reached_callback received. State: {self.state}",
            file_tag=self.file_tag,
        )
        if self.state == "WAIT_FOR_GOAL_RESULT" and msg.status:
            current_time = (msg.stamp.sec, msg.stamp.nanosec)
            if self.last_goal_reach_time == current_time:
                print_log(
                    "warn",
                    self.get_logger(),
                    "⚠️ Duplicate goal_reached message received. Skipping.",
                    file_tag=self.file_tag,
                )
                return

            self.last_goal_reach_time = current_time

            print_log(
                "info",
                self.get_logger(),
                f"[FSM] Goal reached at {msg.stamp.sec}.{str(msg.stamp.nanosec).zfill(9)}",
                file_tag=self.file_tag,
            )
            self.state = "FRONTIER_SEARCH"

    def goal_failed_callback(self, msg):
        if self.state == "WAIT_FOR_COMMAND":
            return
        if self.state == "WAIT_FOR_GOAL_RESULT" and msg.status:
            print_log(
                "warn",
                self.get_logger(),
                "❌ Goal failed. Retrying frontier search.",
                file_tag=self.file_tag,
            )
            self.state = "FRONTIER_SEARCH"

    def fsm_step(self):
        if self.state == "WAIT_FOR_COMMAND":
            return
        print_log(
            "info",
            self.get_logger(),
            f"[FSM] fsm_step() called. Current state: {self.state}",
            file_tag=self.file_tag,
        )
        # ✅ 중복 실행 방지
        if self.running:
            print_log(
                "warn",
                self.get_logger(),
                "[FSM] Already running. Skipping this tick.",
                file_tag=self.file_tag,
            )
            return
        print_log(
            "info",
            self.get_logger(),
            "[FSM] Acquired execution lock.",
            file_tag=self.file_tag,
        )
        self.running = True

        try:
            if self.state == "FRONTIER_SEARCH":
                if (
                    self.map_data is None
                    or self.current_pose is None
                    or self.map_info is None
                ):
                    print_log(
                        "warn",
                        self.get_logger(),
                        "[FSM] Missing map or pose. Skipping.",
                        file_tag=self.file_tag,
                    )
                    return

                # 프론티어 탐색 후 개수 확인
                self.frontiers = find_frontiers(self.map_data)
                print_log(
                    "info",
                    self.get_logger(),
                    f"[DEBUG] Found {len(self.frontiers)} frontiers.",
                    file_tag=self.file_tag,
                )

                if not self.frontiers:
                    print_log(
                        "info",
                        self.get_logger(),
                        "✅ No frontiers left. Mapping complete.",
                        file_tag=self.file_tag,
                    )
                    done_msg = MappingDone()
                    done_msg.header.stamp = self.get_clock().now().to_msg()
                    done_msg.header.frame_id = "map"
                    done_msg.success = True
                    done_msg.map = self.raw_map_msg
                    done_msg.map_inflated = OccupancyGrid()
                    done_msg.map_inflated.header.frame_id = "map"
                    done_msg.map_inflated.header.stamp = self.get_clock().now().to_msg()
                    done_msg.map_inflated.info = self.map_info
                    done_msg.map_inflated.data = self.map_data.flatten().tolist()

                    self.done_pub.publish(done_msg)
                    self.state = "WAIT_FOR_COMMAND"
                    return

                # 2. grid → world 변환
                frontier_world = [
                    grid_to_world(x, y, self.map_info) for x, y in self.frontiers
                ]
                print_log(
                    "info",
                    self.get_logger(),
                    f"[DEBUG] First 3 world frontiers: {frontier_world[:3]}",
                    file_tag=self.file_tag,
                )

                # 3. FOV 필터링 (360도 허용)
                fov_filtered = [
                    pt
                    for pt in frontier_world
                    if is_within_fov(self.current_pose, pt, fov_deg=360)
                ]
                print_log(
                    "info",
                    self.get_logger(),
                    f"[DEBUG] FOV filtered: {len(fov_filtered)}",
                    file_tag=self.file_tag,
                )

                if not fov_filtered:
                    print_log(
                        "warn",
                        self.get_logger(),
                        "⚠️ No frontiers within FOV.",
                        file_tag=self.file_tag,
                    )
                    return

                # 4. 최소 거리 필터링
                MIN_FRONTIER_DIST = 2.0
                far_enough = [
                    pt
                    for pt in fov_filtered
                    if np.linalg.norm(np.array(self.current_pose[:2]) - np.array(pt))
                    >= MIN_FRONTIER_DIST
                ]
                print_log(
                    "info",
                    self.get_logger(),
                    f"[DEBUG] Far enough: {len(far_enough)} (MIN={MIN_FRONTIER_DIST}m)",
                    file_tag=self.file_tag,
                )

                if not far_enough:
                    print_log(
                        "warn",
                        self.get_logger(),
                        "⚠️ No frontiers far enough. Skipping.",
                        file_tag=self.file_tag,
                    )
                    return

                # 5. 가장 가까운 프론티어 중 이전 goal과 다른 지점 선택
                sorted_candidates = sorted(
                    far_enough,
                    key=lambda pt: np.linalg.norm(
                        np.array(self.current_pose[:2]) - np.array(pt)
                    ),
                )

                target = None
                for candidate in sorted_candidates:
                    if any(
                        np.linalg.norm(np.array(candidate) - np.array(fg)) < 0.5
                        for fg in self.failed_goals
                    ):
                        continue
                    if self.prev_goal is None:
                        target = candidate
                        break
                    dist_to_prev = np.linalg.norm(
                        np.array(candidate) - np.array(self.prev_goal)
                    )
                    if dist_to_prev >= 0.5:  # 너무 가까운 이전 goal은 제외
                        target = candidate
                        break
                if target is None:
                    print_log(
                        "warn",
                        self.get_logger(),
                        "⚠️ All candidates too close to previous goal. Skipping publish.",
                        file_tag=self.file_tag,
                    )
                    return

                # 5. 가장 가까운 프론티어 선택
                dist_to_target = np.linalg.norm(
                    np.array(self.current_pose[:2]) - np.array(target)
                )
                print_log(
                    "info",
                    self.get_logger(),
                    f"[DEBUG] Selected target: ({target[0]:.2f}, {target[1]:.2f}) | Distance to bot: {dist_to_target:.2f}m",
                    file_tag=self.file_tag,
                )

                # # ✅ [6] 이전 goal과 거리 로그
                # if self.prev_goal is not None:
                #     dist_to_prev_goal = np.linalg.norm(
                #         np.array(self.prev_goal) - np.array(target)
                #     )
                #     print_log(
                #         "info",
                #         self.get_logger(),
                #         f"[DEBUG] Distance to prev_goal: {dist_to_prev_goal:.2f}m",
                #         file_tag=self.file_tag,
                #     )
                #     if dist_to_prev_goal < 0.5:
                #         print_log(
                #             "info",
                #             self.get_logger(),
                #             f"⚠️ Too close to previous goal. Skipping publish.\n    - Previous goal: ({self.prev_goal[0]:.2f}, {self.prev_goal[1]:.2f})\n    - Current candidate: ({target[0]:.2f}, {target[1]:.2f})",
                #             file_tag=self.file_tag,
                #         )
                #         return

                # 7. goal_pose 퍼블리시
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = "map"
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.position.x = target[0]
                goal_msg.pose.position.y = target[1]
                goal_msg.pose.orientation.w = 1.0

                print_log(
                    "info",
                    self.get_logger(),
                    f"[FSM] Publishing goal: {target}",
                    file_tag=self.file_tag,
                )
                self.pub_goal.publish(goal_msg)
                print_log(
                    "info",
                    self.get_logger(),
                    f"[FSM] Goal published.",
                    file_tag=self.file_tag,
                )
                self.prev_goal = target
                print_log(
                    "info",
                    self.get_logger(),
                    f"📍 Published goal to frontier ({target[0]:.2f}, {target[1]:.2f})",
                    file_tag=self.file_tag,
                )

                # 다음 상태로 전이
                self.state = "WAIT_FOR_PLAN_RESULT"
                print_log(
                    "info",
                    self.get_logger(),
                    f"[FSM] State changed to WAIT_FOR_PLAN_RESULT (after goal publish)",
                    file_tag=self.file_tag,
                )
                return
        finally:
            self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = AutoMappingFSM()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
