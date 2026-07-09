"""GInsStream 核心数据类型定义。"""
from dataclasses import dataclass
from typing import Optional, List

import numpy as np


@dataclass
class ImuMeasurement:
    """IMU 单次测量。"""
    timestamp: float          # Unix 时间戳（秒，与 rtklib-py gtime_t 一致）
    week: int                 # GPS 周号（由 timestamp 派生，便利字段）
    accel: np.ndarray         # [3] m/s² 机体坐标系
    gyro: np.ndarray          # [3] rad/s 机体坐标系


@dataclass
class GnssSolution:
    """外部 GNSS 结果。"""
    timestamp: float          # Unix 时间戳（秒，与 rtklib-py gtime_t 一致）
    week: int                 # GPS 周号（由 timestamp 派生，便利字段）
    position: np.ndarray      # [3] ECEF (m)
    quality: int              # 1=SPP, 2=RTD, 5=LC
    num_sv: int
    sd: np.ndarray            # [3] 位置标准差 (sdx, sdy, sdz)
    cov: Optional[np.ndarray] = None  # [3,3] ECEF 协方差矩阵（可选，含非对角项）
    velocity: Optional[np.ndarray] = None  # [3] ECEF 速度 (m/s)（多普勒测速，可选）
    vel_sd: Optional[np.ndarray] = None    # [3] 速度标准差 (m/s)


@dataclass
class InsState:
    """INS 初始化状态（E 系 ECEF）。"""
    timestamp: float              # Unix 时间戳 (s)
    pos_e: np.ndarray             # [3] ECEF 位置 (m)
    vel_e: np.ndarray             # [3] ECEF 速度 (m/s)
    C_b_e: np.ndarray             # [3,3] 旋转矩阵 b→e
    q_b_e: np.ndarray             # [4] 四元数 b→e [w,x,y,z]
    att_rpy: np.ndarray           # [3] 欧拉角 [roll,pitch,yaw] (rad)
    gyro_bias: np.ndarray         # [3] 陀螺零偏 (rad/s)
    accel_bias: np.ndarray        # [3] 加计零偏 (m/s²)
    imu_angle: np.ndarray         # [2] IMU 安装角 [pitch,yaw] (rad)
    imu_leverarm: np.ndarray      # [3] IMU 杆臂 b→v (m)
    leverarm: np.ndarray          # [3] GNSS 天线杆臂 (b 系, m)
    time_sync: float = 0.0        # IMU-GNSS 时间对齐误差 (s)


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
