"""地球参数与坐标转换工具。

参考 gnss_ins_lc_nhc navearth.hpp 与 KF-GINS Winearth.hpp。
"""
import math
from typing import Tuple

import numpy as np

# WGS84 椭球参数
EARTH_SEMI_MAJOR = 6378137.0           # 长半轴 a (m)
EARTH_SEMI_MINOR = 6356752.314245      # 短半轴 b (m)
EARTH_ECCENTRICITY_SQ = 1.0 - (EARTH_SEMI_MINOR / EARTH_SEMI_MAJOR) ** 2  # 第一偏心率平方 e²
EARTH_GRAVITY_EQUATOR = 9.7803253359   # 赤道重力 (m/s²)
EARTH_GRAVITY_POLE = 9.8321849378      # 极点重力 (m/s²)
EARTH_GRAVITY_CONST = 3.986004418e14   # 地球引力常数 GM (m³/s²)
EARTH_ROTATION_RATE = 7.2921151467e-5  # 地球自转角速度 ωie (rad/s)
EARTH_FLATTENING = 1.0 / 298.257223563


def ecef2llh(pos_e: np.ndarray) -> Tuple[float, float, float]:
    """ECEF → [lat, lon, h] (WGS84)。

    参考 gnss_ins_lc_nhc WGS84XYZ2BLH。
    """
    x, y, z = pos_e
    lon = math.atan2(y, x)
    p = math.sqrt(x * x + y * y)
    if p < 1e-12:
        lat = math.copysign(math.pi / 2, z)
        h = abs(z) - EARTH_SEMI_MINOR
        return lat, lon, h
    # 迭代求解纬度
    lat = math.atan2(z, p * (1.0 - EARTH_ECCENTRICITY_SQ))
    for _ in range(10):
        sin_lat = math.sin(lat)
        N = EARTH_SEMI_MAJOR / math.sqrt(1.0 - EARTH_ECCENTRICITY_SQ * sin_lat * sin_lat)
        h = p / math.cos(lat) - N
        lat = math.atan2(z, p * (1.0 - EARTH_ECCENTRICITY_SQ * N / (N + h)))
    return lat, lon, h


def llh2ecef(lat: float, lon: float, h: float) -> np.ndarray:
    """[lat, lon, h] → ECEF (WGS84)。"""
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    N = EARTH_SEMI_MAJOR / math.sqrt(1.0 - EARTH_ECCENTRICITY_SQ * sin_lat * sin_lat)
    x = (N + h) * cos_lat * cos_lon
    y = (N + h) * cos_lat * sin_lon
    z = (N * (1.0 - EARTH_ECCENTRICITY_SQ) + h) * sin_lat
    return np.array([x, y, z], dtype=np.float64)


def cal_Ce2n(lat: float, lon: float) -> np.ndarray:
    """构造 ECEF→NED 旋转矩阵 C_e^n。

    参考 gnss_ins_lc_nhc CalCe2n。
    NED: x=North, y=East, z=Down
    """
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    return np.array([
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [-sin_lon,           cos_lon,            0.0    ],
        [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat],
    ], dtype=np.float64)


def cal_Cn2e(lat: float, lon: float) -> np.ndarray:
    """构造 NED→ECEF 旋转矩阵 C_n^e = C_e^n.T。"""
    return cal_Ce2n(lat, lon).T


def gravity_ecef(pos_e: np.ndarray) -> np.ndarray:
    """ECEF 位置处的重力 (含离心力)。

    参考 KF-GINS Winearth.hpp computeGravity。
    返回 ECEF 系下的重力加速度向量 (m/s²)。
    """
    x, y, z = pos_e
    r2 = x * x + y * y + z * z
    r = math.sqrt(r2)
    if r < 1.0:
        return np.zeros(3, dtype=np.float64)
    # 正常重力 (Somigliana 公式简化)
    lat, _, _ = ecef2llh(pos_e)
    sin_lat = math.sin(lat)
    # 重力大小 (WGS84 正常重力公式)
    g = EARTH_GRAVITY_EQUATOR * (1.0 +
        1.931852652458e-3 * sin_lat * sin_lat) / math.sqrt(
        1.0 - 6.694379990141e-3 * sin_lat * sin_lat)
    # ECEF 系重力方向 (指向地心)
    gamma_e = -g * pos_e / r
    # 加上离心力 (地球自转)
    omega2 = EARTH_ROTATION_RATE ** 2
    gamma_e[0] += omega2 * x
    gamma_e[1] += omega2 * y
    return gamma_e
