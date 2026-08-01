"""姿态表示转换工具。

欧拉角顺序: ZYX (yaw → pitch → roll)
旋转矩阵定义: C_b^n 表示从 b 系到 n 系的转换
"""
import math
from typing import Tuple

import numpy as np


def euler2dcm(rpy: np.ndarray) -> np.ndarray:
    """欧拉角 [roll, pitch, yaw] (rad) → 旋转矩阵 C_b^n (ZYX 旋转顺序)。

    旋转顺序: 先绕 Z 轴 (yaw), 再绕 Y 轴 (pitch), 最后绕 X 轴 (roll)
    """
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr               ],
    ], dtype=np.float64)


def dcm2euler(C: np.ndarray) -> np.ndarray:
    """旋转矩阵 C_b^n → 欧拉角 [roll, pitch, yaw] (rad)。"""
    pitch = math.asin(-max(-1.0, min(1.0, C[2, 0])))
    if abs(C[2, 0]) < 1.0 - 1e-9:
        roll = math.atan2(C[2, 1], C[2, 2])
        yaw = math.atan2(C[1, 0], C[0, 0])
    else:
        # 万向锁
        roll = 0.0
        yaw = math.atan2(-C[0, 1], C[1, 1])
    return np.array([roll, pitch, yaw], dtype=np.float64)


def dcm2quat(C: np.ndarray) -> np.ndarray:
    """旋转矩阵 → 四元数 [w, x, y, z]。"""
    trace = C[0, 0] + C[1, 1] + C[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (C[2, 1] - C[1, 2]) / s
        y = (C[0, 2] - C[2, 0]) / s
        z = (C[1, 0] - C[0, 1]) / s
    elif C[0, 0] > C[1, 1] and C[0, 0] > C[2, 2]:
        s = math.sqrt(1.0 + C[0, 0] - C[1, 1] - C[2, 2]) * 2.0
        w = (C[2, 1] - C[1, 2]) / s
        x = 0.25 * s
        y = (C[0, 1] + C[1, 0]) / s
        z = (C[0, 2] + C[2, 0]) / s
    elif C[1, 1] > C[2, 2]:
        s = math.sqrt(1.0 + C[1, 1] - C[0, 0] - C[2, 2]) * 2.0
        w = (C[0, 2] - C[2, 0]) / s
        x = (C[0, 1] + C[1, 0]) / s
        y = 0.25 * s
        z = (C[1, 2] + C[2, 1]) / s
    else:
        s = math.sqrt(1.0 + C[2, 2] - C[0, 0] - C[1, 1]) * 2.0
        w = (C[1, 0] - C[0, 1]) / s
        x = (C[0, 2] + C[2, 0]) / s
        y = (C[1, 2] + C[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def quat2dcm(q: np.ndarray) -> np.ndarray:
    """四元数 [w, x, y, z] → 旋转矩阵。"""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)    ],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)    ],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def att_caln2e(lat: float, lon: float, C_b_n: np.ndarray) -> np.ndarray:
    """n 系姿态矩阵 → E 系姿态矩阵 C_b^e = C_n^e × C_b^n。"""
    from src.core.ins.earth_param import cal_Cn2e
    C_n_e = cal_Cn2e(lat, lon)  # NED→ECEF
    return C_n_e @ C_b_n
