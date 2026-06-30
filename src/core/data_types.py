"""GInsStream 核心数据类型定义。"""
from dataclasses import dataclass
from typing import Optional

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
class AlignedRow:
    """对齐后的一行输出数据。"""
    week: int
    sow: float                # GNSS 历元时间
    imu_count: int            # 窗口内 IMU 数据条数
    imu_first_sow: float
    imu_last_sow: float
    gnss_pos: np.ndarray      # [3]
    gnss_q: int
    gnss_ns: int
    gnss_sd: np.ndarray       # [3]
    imu_avg_accel: np.ndarray # [3] 窗口均值
    imu_avg_gyro: np.ndarray  # [3] 窗口均值


@dataclass
class SensorData:
    """传感器数据统一容器（一次只承载一种类型）。"""
    tag: str                                  # "imu" / "gnss_solution"
    imu: Optional[ImuMeasurement] = None
    gnss_solution: Optional[GnssSolution] = None
