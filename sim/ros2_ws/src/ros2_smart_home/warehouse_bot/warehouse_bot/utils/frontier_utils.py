# warehouse_bot/utils/frontier_utils.py

import numpy as np
import math


# 프론티어 셀 탐색 함수
def find_frontiers(occupancy_grid):
    """
    OccupancyGrid 맵 상에서 프론티어 셀(known(0) 주변에 unknown(-1)이 있는 셀)을 찾음.
    """
    height, width = occupancy_grid.shape
    frontiers = set()
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if occupancy_grid[y, x] == 0:
                neighbors = occupancy_grid[y - 1 : y + 2, x - 1 : x + 2].flatten()
                if -1 in neighbors:
                    frontiers.add((x, y))
    return list(frontiers)


# 특정 셀이 봇의 시야(FOV) 내에 있는지 확인
def is_within_fov(bot_pose, target_pos, fov_deg=60):
    bot_x, bot_y, bot_theta = bot_pose
    dx, dy = target_pos[0] - bot_x, target_pos[1] - bot_y
    angle_to_target = math.atan2(dy, dx)
    angle_diff = normalize_angle(angle_to_target - bot_theta)
    return abs(math.degrees(angle_diff)) <= fov_deg / 2


# 각도 보정 함수
def normalize_angle(angle):
    """
    -pi ~ pi 범위로 각도 보정
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def grid_to_world(x, y, map_info):
    world_x = map_info.origin.position.x + (x + 0.5) * map_info.resolution
    world_y = map_info.origin.position.y + (y + 0.5) * map_info.resolution
    return world_x, world_y

def world_to_grid(x, y, map_info):
    res = map_info.resolution
    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y

    gx = int((x - origin_x) / res)
    gy = int((y - origin_y) / res)
    return (gx, gy)