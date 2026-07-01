"""IMU 积攒 + GNSS 收割的匹配器。

提供两种匹配策略：
- match(): GNSS 触发窗口 [t_gnss - dt_imu/2, t_gnss + dt_imu/2]（旧，将弃用）
- harvest(): IMU 积攒 + GNSS 收割 timestamp <= t_gnss + harvest_margin（新）
"""
from collections import deque
from typing import Optional, List

import numpy as np

from src.core.data_types import AlignedBlock, AlignedRow, GnssSolution, ImuMeasurement


class Aligner:
    """IMU 积攒 + GNSS 收割的匹配器。"""

    def __init__(self, imu_dt: float, harvest_margin: float = 0.01):
        self.imu_dt = float(imu_dt)
        self.harvest_margin = float(harvest_margin)
        self.half_window = self.imu_dt / 2.0
        self.imu_buffer: deque = deque()
        self.align_started: bool = False

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

    def harvest(self, gnss: GnssSolution) -> Optional[AlignedBlock]:
        """对一个 GNSS 历元做收割。

        首次调用时丢弃 timestamp < t_gnss 的首部 IMU。
        收割缓冲中 timestamp <= t_gnss + harvest_margin 的 IMU。

        Returns:
            AlignedBlock 或 None（收割列表为空）
        """
        t_cut = gnss.timestamp + self.harvest_margin
        # 首次收割：丢弃首部 IMU（timestamp < t_gnss）
        if not self.align_started:
            while self.imu_buffer and self.imu_buffer[0].timestamp < gnss.timestamp:
                self.imu_buffer.popleft()
            self.align_started = True
        # 收割 timestamp <= t_cut 的 IMU
        imu_list: List[ImuMeasurement] = []
        while self.imu_buffer and self.imu_buffer[0].timestamp <= t_cut:
            imu_list.append(self.imu_buffer.popleft())
        if not imu_list:
            return None
        return AlignedBlock(gnss=gnss, imu_list=imu_list)
