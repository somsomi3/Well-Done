import socket
import numpy as np
import cv2


def xyh2mat2D(vec):
    # euler_rad = vec[2] * np.pi / 180
    euler_rad = vec[2]
    rot = np.array(
        [np.cos(euler_rad), -np.sin(euler_rad), np.sin(euler_rad), np.cos(euler_rad)]
    )
    trans = vec[0:2]

    T = np.identity(3)
    T[0:2, 0:2] = rot.reshape(2, 2)
    T[0:2, 2] = trans.reshape(-1)

    # print('vec', vec)
    # print('T', T)

    return T


def mat2D2xyh(T):
    vec = np.array([0.0, 0.0, 0.0])
    t = T[:2, 2]
    rot = T[:2, :2]
    vec[0] = t[0]
    vec[1] = t[1]
    vec[2] = np.arctan2(rot[1, 0], rot[0, 0]) * 180.0 / np.pi

    return vec


# radian
def limit_angular_range(rad):

    if rad > np.pi:
        rad -= 2 * np.pi

    if rad < -np.pi:
        rad += 2 * np.pi

    if rad > np.pi:
        rad -= 2 * np.pi

    if rad < -np.pi:
        rad += 2 * np.pi

    return rad


def inflate_map(map_data: np.ndarray, radius: int = 2) -> np.ndarray:
    """
    주어진 occupancy 맵에서 장애물 셀 주변을 지정된 반경만큼 팽창시킨 맵을 반환
    - map_data: 2D numpy array, 값은 -1 (미탐색), 0~100 범위의 int
    - radius: 팽창 거리 (셀 단위)
    """
    h, w = map_data.shape
    inflated = map_data.copy()
    obs_y, obs_x = np.where(map_data == 100)

    for y, x in zip(obs_y, obs_x):
        y0 = max(0, y - radius)
        y1 = min(h, y + radius + 1)
        x0 = max(0, x - radius)
        x1 = min(w, x + radius + 1)
        for i in range(y0, y1):
            for j in range(x0, x1):
                if inflated[i, j] != 100:
                    inflated[i, j] = max(
                        inflated[i, j], 80
                    )  # 기존 값보다 작을 때만 80으로

    return inflated
