# ROS 2 노드: A* 기반 전역 경로 생성 노드
# 주어진 맵과 목표 위치(goal_pose)를 받아 최단 경로(global_path)를 생성함

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque
import heapq

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool

from warehouse_bot.utils.sim_config import params_map


class AStarPlanner(Node):
    def __init__(self):
        super().__init__("a_star_planner")

        # 맵 파라미터 로드
        self.resolution = params_map["MAP_RESOLUTION"]
        self.map_width = int(params_map["MAP_SIZE"][0] / self.resolution)
        self.map_height = int(params_map["MAP_SIZE"][1] / self.resolution)
        self.map_center = params_map["MAP_CENTER"]
        self.offset_x = self.map_center[0] - params_map["MAP_SIZE"][0] / 2
        self.offset_y = self.map_center[1] - params_map["MAP_SIZE"][1] / 2

        self.map = None
        self.odom = None
        self.goal_cell = None

        # 이동 방향 (8방향)
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.cost = [1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4]

        # Publisher & Subscriber
        self.sub_map = self.create_subscription(
            OccupancyGrid, "/map_inflated", self.map_callback, 1
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom_true", self.odom_callback, 1
        )
        self.sub_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 1
        )
        self.pub_path = self.create_publisher(Path, "/global_path", 1)
        self.plan_success_pub = self.create_publisher(Bool, "/plan_success", 1)
        self.plan_failed_pub = self.create_publisher(Bool, "/plan_failed", 1)

    def map_callback(self, msg):
        self.map = (
            np.array(msg.data, dtype=np.int8)
            .reshape((msg.info.height, msg.info.width))
            .T
        )

    def odom_callback(self, msg):
        self.odom = msg

    def goal_callback(self, msg):
        if msg.header.frame_id != "map":
            self.get_logger().warn("Goal frame must be 'map'")
            return

        if self.map is None or self.odom is None:
            self.get_logger().warn("Waiting for map and odom...")
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.goal_cell = self.world_to_grid(goal_x, goal_y)

        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y
        start_cell = self.world_to_grid(start_x, start_y)

        if not self.valid_cell(self.goal_cell):
            self.get_logger().warn("Invalid goal cell.")
            self.plan_failed_pub.publish(Bool(data=True))
            return

        # ✅ start_cell은 100(장애물)만 아니면 통과
        sx, sy = start_cell
        if (
            not (0 <= sx < self.map_width and 0 <= sy < self.map_height)
            or self.map[sx][sy] == 100
        ):
            self.get_logger().warn("Invalid start cell.")
            self.plan_failed_pub.publish(Bool(data=True))
            return

        path = self.a_star(start_cell, self.goal_cell)
        if not path:
            self.get_logger().warn("Path not found.")
            self.plan_failed_pub.publish(Bool(data=True))
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for cell in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            x, y = self.grid_to_world(cell)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.pub_path.publish(path_msg)
        self.get_logger().info(f"Published global path with {len(path)} points.")
        self.plan_success_pub.publish(Bool(data=True))

    def world_to_grid(self, x, y):
        gx = int((x - self.offset_x) / self.resolution)
        gy = int((y - self.offset_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, cell):
        x = cell[0] * self.resolution + self.offset_x + self.resolution / 2
        y = cell[1] * self.resolution + self.offset_y + self.resolution / 2
        return x, y

    def valid_cell(self, cell):
        x, y = cell
        return (
            0 <= x < self.map_width and 0 <= y < self.map_height and self.map[x][y] < 80
        )

    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))  # (f_score, node)

        came_from = {}
        g_score = np.full((self.map_width, self.map_height), np.inf)
        f_score = np.full((self.map_width, self.map_height), np.inf)

        g_score[start] = 0
        f_score[start] = heuristic(start, goal)

        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            if current in visited:
                continue
            visited.add(current)

            for i in range(8):
                nx, ny = current[0] + self.dx[i], current[1] + self.dy[i]
                neighbor = (nx, ny)

                if not self.valid_cell(neighbor):
                    continue

                tentative_g = g_score[current] + self.cost[i]

                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


def heuristic(a, b):
    # 휴리스틱 함수: 8방향 이동이므로 유클리드 거리 사용
    return np.linalg.norm(np.array(a) - np.array(b))


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
