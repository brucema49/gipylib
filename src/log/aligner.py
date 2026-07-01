"""IMU 积攒 + GNSS 收割的匹配器。

IMU 流式推入缓冲，GNSS 到来时收割 timestamp <= t_gnss + harvest_margin 的 IMU。
首个 GNSS 到来前，timestamp < t_gnss_first 的 IMU 被丢弃（首部不对齐数据不输出）。
"""
from collections import deque
from typing import Optional, List

from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement


class Aligner:
    """IMU 积攒 + GNSS 收割的匹配器。"""

    def __init__(self, imu_dt: float, harvest_margin: float = 0.01):
        self.imu_dt = float(imu_dt)
        self.harvest_margin = float(harvest_margin)
        self.imu_buffer: deque = deque()
        self.align_started: bool = False

    def push_imu(self, imu: ImuMeasurement) -> None:
        """将一条 IMU 数据推入缓冲。"""
        self.imu_buffer.append(imu)

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
