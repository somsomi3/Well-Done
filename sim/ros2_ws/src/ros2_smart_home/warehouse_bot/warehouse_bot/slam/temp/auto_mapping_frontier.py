# ROS 2 노드: 자동 프론티어 탐색 기반 매핑을 수행하는 노드
# 맵의 미개척 영역(frontier)을 탐색하며 goal을 설정하고 로봇을 이동시킴

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from squaternion import Quaternion
import numpy as np

# 유틸 함수들 (프론티어 탐색, FOV 필터링, 각도 정규화 등)
from warehouse_bot.utils.frontier_utils import (
    find_frontiers,
    is_within_fov,
    normalize_angle,
)


# 두 좌표 간 거리 계산 함수
def get_distance(p1, p2):
    return np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))


# 오도메트리 메시지에서 방향(heading) 추출
def get_heading(odom_msg):
    q = odom_msg.pose.pose.orientation
    quat = Quaternion(q.w, q.x, q.y, q.z)
    _, _, heading = quat.to_euler()
    return normalize_angle(heading)


# OccupancyGrid 좌표계를 실제 월드 좌표계로 변환
def grid_to_world(x, y, map_info):
    world_x = map_info.origin.position.x + (x + 0.5) * map_info.resolution
    world_y = map_info.origin.position.y + (y + 0.5) * map_info.resolution
    return world_x, world_y


# 프론티어 기반 자동 매핑 노드 정의
class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # 탐색 조건 파라미터
        self.MAP_CHANGE_THRESHOLD = 0.01  # 맵 변화율 기준
        self.MAP_COVERAGE_THRESHOLD = 0.60  # 커버리지 종료 기준
        self.MAP_IDLE_DURATION = 5.0  # 맵 변화 없을 시 종료 시간 기준
        self.GOAL_REPUBLISH_THRESHOLD = 0.5  # goal이 이전 goal과 너무 가까울 경우 skip

        # 내부 상태
        self.map_data = None
        self.map_info = None
        self.prev_map = None
        self.last_change_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.prev_goal = None
        self.current_pose = None
        self.goal_failed = False  # ❌ 경로 실패 플래그
        self.goal_reached = True  # ✅ goal 도달 상태 추가

        # 퍼블리셔/서브스크라이버 설정
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.done_pub = self.create_publisher(Bool, "/mapping_done", 1)
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map_inflated", self.map_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, 10
        )
        self.sub_goal_failed = self.create_subscription(
            Bool, "/goal_failed", self.goal_failed_callback, 10
        )
        self.sub_goal_reached = self.create_subscription(
            Bool, "/goal_reached", self.goal_reached_callback, 1
        )
        self.sub_plan_failed = self.create_subscription(
            Bool, "/plan_failed", self.plan_failed_callback, 10
        )
        self.sub_plan_success = self.create_subscription(
            Bool, "/plan_success", self.plan_success_callback, 10
        )

        # 타이머 콜백 (주기적 프론티어 탐색)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Frontier-based auto mapping started.")

    def publish_mapping_done(self):
        self.done_pub.publish(Bool(data=True))
        self.get_logger().info("📬 Published mapping done signal.")

    def goal_failed_callback(self, msg):
        if msg.data:
            self.get_logger().warn(
                "[FAIL] Received goal failure signal from path_tracking."
            )
            self.goal_failed = True
            self.goal_reached = True

    def goal_reached_callback(self, msg):
        if msg.data and not self.goal_reached:
            self.get_logger().info("✅ [RESULT] Goal reached signal received.")
            self.goal_reached = True

    def plan_failed_callback(self, msg):
        if msg.data:
            self.get_logger().warn("[FAIL] Received plan failure from a_star.")
            self.goal_failed = True
            self.goal_reached = True

    def plan_success_callback(self, msg):
        if msg.data:
            self.get_logger().info("✅ [PLAN] Received plan success from a_star.")

    # 현재 로봇 위치 저장
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = get_heading(msg)
        self.current_pose = [x, y, theta]

    # 맵 수신 시 변화율, 커버리지, 프론티어 판단
    def map_callback(self, msg):
        new_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        now = self.get_clock().now().seconds_nanoseconds()[0]

        if self.prev_map is not None:
            diff = np.abs(new_map - self.prev_map)
            change_rate = np.count_nonzero(diff) / diff.size
            self.get_logger().info(f"[MAP] Change rate: {change_rate:.4f}")

            # 맵 내 알려진 셀 비율 계산 (0 또는 100)
            observed = (new_map == 0) | (new_map > 70)
            coverage = np.count_nonzero(observed) / new_map.size
            self.get_logger().info(f"[MAP] Coverage: {coverage:.2%}")

            # 프론티어가 더 이상 없으면 종료
            frontiers = find_frontiers(new_map)
            if len(frontiers) == 0:
                self.get_logger().info("✅ [MAP] No frontiers left. Auto stopping.")
                self.publish_mapping_done()
                self.destroy_node()
                return

            # 맵 변화가 있다면 idle 타이머 초기화
            if change_rate >= self.MAP_CHANGE_THRESHOLD:
                self.last_change_time = now

            duration = now - self.last_change_time

            # 일정 시간 변화가 없고, 커버리지가 충분하면 종료
            if (
                duration > self.MAP_IDLE_DURATION
                and coverage > self.MAP_COVERAGE_THRESHOLD
            ):
                self.get_logger().info("✅ [MAP] Mapping complete. Shutting down.")
                self.publish_mapping_done()
                self.destroy_node()
                return

        else:
            self.get_logger().info("[MAP] First map received.")

        self.prev_map = new_map.copy()
        self.map_data = new_map
        self.map_info = msg.info

    # 프론티어 탐색 및 goal_pose 퍼블리시
    def timer_callback(self):
        if self.map_data is None or self.map_info is None or self.current_pose is None:
            self.get_logger().warn("[TIMER] Waiting for map and pose...")
            return
        
        if not self.goal_reached:
            # TODO: goal_reached가 켜졌을 때, 정확한 위치정보를 위해 odom을 한번 받을 때 까지 대기할 필요가 있을까?
            self.get_logger().info("[TIMER] Goal in progress. Skipping frontier exploration.")
            return
        
        # 1. 프론티어 셀 찾기
        frontiers = find_frontiers(self.map_data)
        if not frontiers:
            #TODO: 프론티어가 없다면 매핑이 끝났다는 신호를 보내야 하지 않을까?
            self.get_logger().warn("[TIMER] No frontiers found.")
            return

        # 2. grid → world 좌표 변환
        frontier_world = [grid_to_world(x, y, self.map_info) for x, y in frontiers]

        # 3. FOV 필터링 (현재는 360도 전체 허용)
        fov_filtered = [
            pt
            for pt in frontier_world
            if is_within_fov(self.current_pose, pt, fov_deg=360)
        ]
        if not fov_filtered:
            self.get_logger().warn("[TIMER] No frontiers within FOV.")
            return

        MIN_FRONTIER_DIST = 5  # 🔧 최소 프론티어 거리 제한

        # 4. 가장 가까운 프론티어 선택
        bot_x, bot_y = self.current_pose[0], self.current_pose[1]
        far_enough_frontiers = [
            pt
            for pt in fov_filtered
            if get_distance((bot_x, bot_y), pt) >= MIN_FRONTIER_DIST
        ]

        if not far_enough_frontiers:
            #TODO: 여기도 프론티어가 없으면 매핑을 종료시킬까?
            self.get_logger().warn("[GOAL] No frontiers far enough. Skipping publish.")
            return

        nearest = min(
            far_enough_frontiers, key=lambda pt: get_distance((bot_x, bot_y), pt)
        )

        # 이전 goal과 너무 가까우면 skip
        if not self.goal_failed:
            if (
                self.prev_goal is not None
                and get_distance(self.prev_goal, nearest)
                < self.GOAL_REPUBLISH_THRESHOLD
            ):
                self.get_logger().info(
                    "[GOAL] Goal too close to previous. Skipping publish."
                )
                return

        # goal_pose 메시지 생성 및 퍼블리시
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = nearest[0]
        goal.pose.position.y = nearest[1]
        goal.pose.orientation.w = 1.0

        self.get_logger().info(
            f"[GOAL] Navigating to frontier at ({nearest[0]:.2f}, {nearest[1]:.2f})"
        )
        self.pub_goal.publish(goal)
        self.prev_goal = nearest
        self.goal_failed = False  # ✅ goal 재설정 이후 실패 상태 초기화
        self.goal_reached = False  # ✅ goal 설정 후 대기 상태로 전환


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
