"""GInsStream 核心数据类型定义。"""
from dataclasses import dataclass, field
from typing import Optional, List

import numpy as np
from src.core.time_utils import unix_to_gpst


@dataclass(frozen=True)
class RateImuData:
    """速率式 IMU payload。"""
    gyro: np.ndarray          # [3] rad/s 机体坐标系
    accel: np.ndarray         # [3] m/s² 机体坐标系


@dataclass(frozen=True)
class IncrementImuData:
    """增量式 IMU payload，保留来源文件的精确采样区间。"""
    dtheta: np.ndarray        # [3] rad，采样区间角增量
    dvel: np.ndarray          # [3] m/s，采样区间速度增量
    dt: float                 # s，来源文件相邻样本的时间差
    sow: float                # s，当前样本的 GPS 周内秒

    def __post_init__(self) -> None:
        dtheta = np.asarray(self.dtheta, dtype=np.float64)
        dvel = np.asarray(self.dvel, dtype=np.float64)
        if dtheta.shape != (3,) or dvel.shape != (3,):
            raise ValueError("IMU increments must contain three axes")
        if not np.all(np.isfinite(dtheta)) or not np.all(np.isfinite(dvel)):
            raise ValueError("IMU increments must be finite")
        if not np.isfinite(self.dt) or self.dt <= 0.0:
            raise ValueError("IMU increment dt must be finite and positive")
        if not np.isfinite(self.sow):
            raise ValueError("IMU increment SOW must be finite")
        object.__setattr__(self, "dtheta", dtheta)
        object.__setattr__(self, "dvel", dvel)
        object.__setattr__(self, "dt", float(self.dt))
        object.__setattr__(self, "sow", float(self.sow))


@dataclass(frozen=True, init=False)
class ImuMeasurement:
    """IMU 样本公共时间外壳，payload 明确区分速率和增量。"""
    timestamp: float          # Unix 时间戳（秒，与 rtklib-py gtime_t 一致）
    week: int                 # GPS 周号（由 timestamp 派生，便利字段）
    payload: RateImuData | IncrementImuData

    def __init__(self, timestamp: float, week: int,
                 accel: np.ndarray | None = None,
                 gyro: np.ndarray | None = None,
                 payload: RateImuData | IncrementImuData | None = None):
        # Keep the historical positional (timestamp, week, accel, gyro)
        # constructor for rate samples while making the representation explicit
        # for all new parser and mechanization code.
        if payload is None:
            if accel is None or gyro is None:
                raise TypeError("rate IMU samples require accel and gyro")
            payload = RateImuData(
                gyro=np.asarray(gyro, dtype=np.float64),
                accel=np.asarray(accel, dtype=np.float64),
            )
        if not isinstance(payload, (RateImuData, IncrementImuData)):
            raise TypeError("payload must be RateImuData or IncrementImuData")
        object.__setattr__(self, "timestamp", float(timestamp))
        object.__setattr__(self, "week", int(week))
        object.__setattr__(self, "payload", payload)

    def is_rate(self) -> bool:
        return isinstance(self.payload, RateImuData)

    def is_increment(self) -> bool:
        return isinstance(self.payload, IncrementImuData)

    def rate_view(self, fallback_dt: float | None = None) -> RateImuData:
        """Return rate values for constraints/diagnostics without changing payload."""
        if isinstance(self.payload, RateImuData):
            return self.payload
        return RateImuData(
            gyro=self.payload.dtheta / self.payload.dt,
            accel=self.payload.dvel / self.payload.dt,
        )

    def increment_view(self, propagation_dt: float | None = None) -> IncrementImuData:
        """Return raw increments, deriving them from a rate only with explicit dt."""
        if isinstance(self.payload, IncrementImuData):
            return self.payload
        if propagation_dt is None:
            raise ValueError("rate IMU increment_view requires propagation_dt")
        if not np.isfinite(propagation_dt) or propagation_dt <= 0.0:
            raise ValueError("propagation_dt must be finite and positive")
        _, sow = unix_to_gpst(self.timestamp)
        return IncrementImuData(
            dtheta=self.payload.gyro * propagation_dt,
            dvel=self.payload.accel * propagation_dt,
            dt=float(propagation_dt),
            sow=sow,
        )

    @property
    def gyro(self) -> np.ndarray:
        """Compatibility access for rate samples; increments must use rate_view()."""
        if not isinstance(self.payload, RateImuData):
            raise TypeError("increment IMU has no gyro rate; use rate_view()")
        return self.payload.gyro

    @property
    def accel(self) -> np.ndarray:
        """Compatibility access for rate samples; increments must use rate_view()."""
        if not isinstance(self.payload, RateImuData):
            raise TypeError("increment IMU has no accel rate; use rate_view()")
        return self.payload.accel


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
    source_sow: Optional[float] = None     # 原始 GPS 周内秒（可选，避免 Unix 浮点回算）


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
    # 可选 IMU 比例因子 (无量纲；1.0e-6 = 1 ppm)。默认零以保持
    # 既有 15 状态路径及所有非 KF-GINS 配置的行为不变。
    gyro_scale: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    accel_scale: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))


@dataclass
class AlignedBlock:
    """对齐后的块数据：1 个 GNSS + N 个 IMU。"""
    gnss: GnssSolution
    imu_list: List[ImuMeasurement]


@dataclass
class SensorData:
    """传感器数据统一容器（一次只承载一种类型）。"""
    tag: str                                  # "imu" / "gnss_solution" / "gnss_raw"
    imu: Optional[ImuMeasurement] = None
    gnss_solution: Optional[GnssSolution] = None
    gnss_raw: Optional[tuple] = None          # (obsr, obsb, nav) 原始观测 (TC 模式)
