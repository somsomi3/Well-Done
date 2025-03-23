import rclpy
import numpy as np
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData,Path
from math import pi,cos,sin
from collections import deque

# a_star 노드는  OccupancyGrid map을 받아 grid map 기반 최단경로 탐색 알고리즘을 통해 로봇이 목적지까지 가는 경로를 생성하는 노드입니다.
# 로봇의 위치(/pose), 맵(/map), 목표 위치(/goal_pose)를 받아서 전역경로(/global_path)를 만들어 줍니다. 
# goal_pose는 rviz2에서 2D Goal Pose 버튼을 누르고 위치를 찍으면 메시지가 publish 됩니다. 
# 주의할 점 : odom을 받아서 사용하는데 기존 odom 노드는 시작했을 때 로봇의 초기 위치가 x,y,heading(0,0,0) 입니다. 로봇의 초기위치를 맵 상에서 로봇의 위치와 맞춰줘야 합니다. 
# 따라서 sub2의 odom 노드를 수정해줍니다. turtlebot_status 안에는 정답데이터(절대 위치)가 있는데 그 정보를 사용해서 맵과 로봇의 좌표를 맞춰 줍니다.

class a_star(Node):

    def __init__(self):
        super().__init__('a_Star')

        # Publisher & Subscriber
        self.map_sub = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,1)
        self.goal_sub = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)
        self.a_star_pub= self.create_publisher(Path, 'global_path', 1)
        
        # 메시지 저장
        self.map_msg=OccupancyGrid()
        self.odom_msg=Odometry()

        self.is_map=False
        self.is_odom=False
        self.is_found_path=False
        self.is_grid_update=False


        # 맵 파라미터
        self.goal = [184,224] 
        self.map_size_x=350
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-8-8.75
        self.map_offset_y=-4-8.75
        self.GRIDSIZE=350 

        # 8방향 이동
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

    def grid_update(self):
        self.is_grid_update = True
        map_to_grid = np.array(self.map_msg.data)
        self.grid = np.reshape(map_to_grid, (self.GRIDSIZE, self.GRIDSIZE))

    def pose_to_grid_cell(self,x,y):
        map_point_x = int((x - self.map_offset_x) / self.map_resolution)
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)
        return map_point_x, map_point_y

    def grid_cell_to_pose(self,grid_cell):
        x = grid_cell[0] * self.map_resolution + self.map_offset_x
        y = grid_cell[1] * self.map_resolution + self.map_offset_y
        return [x, y]

    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def map_callback(self,msg):
        self.is_map=True
        self.map_msg=msg

    def goal_callback(self,msg):
        if msg.header.frame_id=='map':
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            self.goal = self.pose_to_grid_cell(goal_x, goal_y)
            # ⚠️ grid가 아직 없으면 먼저 생성
            if not self.is_grid_update:
                self.grid_update()

            # ✅ 이제 접근 가능
            self.get_logger().info(f"goal: {self.goal}")
            self.get_logger().info(f"grid value at goal: {self.grid[self.goal[0]][self.goal[1]]}")

            # print(msg)
            
            if self.is_map and self.is_odom:
                if not self.is_grid_update:
                    self.grid_update()
        
                self.final_path=[]
                x=self.odom_msg.pose.pose.position.x
                y=self.odom_msg.pose.pose.position.y
                start = self.pose_to_grid_cell(x, y)

                self.path = [[0 for _ in range(self.GRIDSIZE)] for _ in range(self.GRIDSIZE)]
                self.cost = np.full((self.GRIDSIZE, self.GRIDSIZE), self.GRIDSIZE * self.GRIDSIZE)

                if self.grid[start[0]][start[1]] == 0  and self.grid[self.goal[0]][self.goal[1]] ==0  and start != self.goal :
                    self.dijkstra(start)

                self.global_path_msg=Path()
                self.global_path_msg.header.frame_id='map'
                for grid_cell in reversed(self.final_path) :
                    tmp_pose=PoseStamped()
                    x, y = self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x = x
                    tmp_pose.pose.position.y = y
                    tmp_pose.pose.orientation.w = 1.0
                    self.global_path_msg.poses.append(tmp_pose)
            
                if len(self.final_path)!=0 :
                    self.a_star_pub.publish(self.global_path_msg)

    def dijkstra(self,start):
        self.get_logger().info("Path planning started")
        Q = deque()
        Q.append(start)
        self.cost[start[0]][start[1]] = 0
        
        while Q:
            current = Q.popleft()

            if current == self.goal:
                break

            for i in range(8):
                nx = current[0] + self.dx[i]
                ny = current[1] + self.dy[i]
                
                if 0 <= nx < self.GRIDSIZE and 0 <= ny < self.GRIDSIZE:
                    if self.grid[nx][ny] >= 30:
                        continue

                    new_cost = self.cost[current[0]][current[1]] + self.dCost[i]
                    if new_cost < self.cost[nx][ny]:
                        self.cost[nx][ny] = new_cost
                        self.path[nx][ny] = current
                        Q.append([nx, ny])

        self.final_path.clear()
        node = self.goal
        while node != start:
            self.final_path.append(node)
            node = self.path[node[0]][node[1]]
        self.final_path.append(start)
        self.get_logger().info("Path planning complete")
        
def main(args=None):
    rclpy.init(args=args)
    node = a_star()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()