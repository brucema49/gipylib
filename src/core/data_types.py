"""GInsStream 核心数据类型定义。"""
from dataclasses import dataclass
from typing import Optional, List

import numpy as np


@dataclass
class ImuMeasurement:
    """IMU 单次测量。"""
    timestamp: float          # GPST 秒（周内秒）
    week: int                 # GPS 周号
    accel: np.ndarray         # [3] m/s² 机体坐标系
    gyro: np.ndarray          # [3] rad/s 机体坐标系


@dataclass
class GnssSolution:
    """外部 GNSS 结果。"""
    timestamp: float          # GPST 秒（周内秒）
    week: int
    position: np.ndarray      # [3] ECEF (m)
    quality: int              # 1=SPP, 2=RTD, 5=LC
    num_sv: int
    sd: np.ndarray            # [3] 位置标准差 (sdx, sdy, sdz)


@dataclass
class AlignedBlock:
    """对齐后的块数据：1 个 GNSS + N 个 IMU。"""
    gnss: GnssSolution
    imu_list: List[ImuMeasurement]


@dataclass
class SensorData:
    """传感器数据统一容器（一次只承载一种类型）。"""
    tag: str                                  # "imu" / "gnss_solution"
    imu: Optional[ImuMeasurement] = None
    gnss_solution: Optional[GnssSolution] = None
