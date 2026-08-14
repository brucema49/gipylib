"""地球参数与坐标转换工具。

参考 KF-GINS Winearth.hpp。
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
EARTH_J2 = 1.082627e-3                 # 地球第二带谐系数
EARTH_ROTATION_RATE = 7.2921151467e-5  # 地球自转角速度 ωie (rad/s)
EARTH_FLATTENING = 1.0 / 298.257223563


def ecef2llh(pos_e: np.ndarray) -> Tuple[float, float, float]:
    """ECEF → [lat, lon, h] (WGS84)。"""
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
    """ECEF 位置处的重力加速度 (m/s²)。

    与 ignav ``pregrav()`` 一致，对地球中心引力加入 J2 摄动和离心
    加速度。这里不能把 Somigliana 正常重力再沿 ECEF 半径投影后叠加
    离心项，否则会把已包含在正常重力中的旋转效应重复处理。
    """
    pos_e = np.asarray(pos_e, dtype=np.float64)
    r = float(np.linalg.norm(pos_e))
    if r < EARTH_SEMI_MAJOR / 2.0:
        return np.array([0.0, 0.0, 9.81], dtype=np.float64)

    zeta = -EARTH_GRAVITY_CONST / (r ** 3)
    gamma = 1.5 * EARTH_J2 * EARTH_SEMI_MAJOR ** 2 / (r ** 2)
    z_ratio_sq = (pos_e[2] / r) ** 2
    gravity = np.empty(3, dtype=np.float64)
    equatorial_factor = gamma * (1.0 - 5.0 * z_ratio_sq)
    gravity[0] = zeta * (pos_e[0] + equatorial_factor * pos_e[0])
    gravity[1] = zeta * (pos_e[1] + equatorial_factor * pos_e[1])
    polar_factor = gamma * (3.0 - 5.0 * z_ratio_sq)
    gravity[2] = zeta * (pos_e[2] + polar_factor * pos_e[2])
    gravity[0:2] += EARTH_ROTATION_RATE ** 2 * pos_e[0:2]
    return gravity


def georadi(lat: float) -> float:
    """地心半径 (参考 ignav georadi, ins-gnss.cc line 214-218)。

    Args:
        lat: 纬度 (rad)

    Returns:
        地心半径 (m)
    """
    s = math.sin(lat)
    c = math.cos(lat)
    e_sq = EARTH_ECCENTRICITY_SQ
    return EARTH_SEMI_MAJOR / math.sqrt(1.0 - e_sq * s * s) * \
        math.sqrt(c * c + (1.0 - e_sq) ** 2 * s * s)
