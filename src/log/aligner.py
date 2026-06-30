"""GNSS 触发的 IMU 匹配器。

每收到一个 GNSS 历元，从 IMU 缓冲中取时间窗口内数据：
    窗口 = [t_gnss - dt_imu/2, t_gnss + dt_imu/2]
"""
from collections import deque
from typing import Optional

import numpy as np

from src.core.data_types import AlignedRow, GnssSolution, ImuMeasurement


class Aligner:
    """GNSS 触发的 IMU 匹配器。"""

    def __init__(self, imu_dt: float):
        self.imu_dt = float(imu_dt)
        self.half_window = self.imu_dt / 2.0
        self.imu_buffer: deque = deque()

    def push_imu(self, imu: ImuMeasurement) -> None:
        """将一条 IMU 数据推入缓冲。"""
        self.imu_buffer.append(imu)

    def match(self, gnss: GnssSolution) -> Optional[AlignedRow]:
        """对一个 GNSS 历元做匹配。

        Returns:
            AlignedRow 或 None（窗口内无 IMU）
        """
        t_lo = gnss.timestamp - self.half_window
        t_hi = gnss.timestamp + self.half_window

        # 1. 弹出窗口左侧过期 IMU
        while self.imu_buffer and self.imu_buffer[0].timestamp < t_lo:
            self.imu_buffer.popleft()

        # 2. 收集窗口内 IMU
        windowed = [imu for imu in self.imu_buffer
                    if t_lo <= imu.timestamp <= t_hi]

        if not windowed:
            return None

        # 3. 统计量
        accels = np.array([imu.accel for imu in windowed])
        gyros = np.array([imu.gyro for imu in windowed])
        avg_accel = accels.mean(axis=0)
        avg_gyro = gyros.mean(axis=0)

        return AlignedRow(
            week=gnss.week,
            sow=gnss.timestamp,
            imu_count=len(windowed),
            imu_first_sow=windowed[0].timestamp,
            imu_last_sow=windowed[-1].timestamp,
            gnss_pos=gnss.position,
            gnss_q=gnss.quality,
            gnss_ns=gnss.num_sv,
            gnss_sd=gnss.sd,
            imu_avg_accel=avg_accel,
            imu_avg_gyro=avg_gyro,
        )
