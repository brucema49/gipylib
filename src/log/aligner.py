"""IMU 积攒 + GNSS 收割的匹配器。

IMU 流式推入缓冲，GNSS 到来时收割 [t_gnss, t_gnss + harvest_window) 的 IMU。
每个 GNSS 历元都丢弃 timestamp < t_gnss 的首部 IMU（包括上一块截止到当前 GNSS 之间的 IMU），
确保每块 IMU 时间戳均 >= 对应 GNSS 时间戳，整体时间顺序正确。
"""
from collections import deque
from typing import Optional, List

from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement


class Aligner:
    """IMU 积攒 + GNSS 收割的匹配器。"""

    def __init__(self, imu_dt: float, harvest_window: float = 1.0):
        self.imu_dt = float(imu_dt)
        self.harvest_window = float(harvest_window)
        self.imu_buffer: deque = deque()

    def push_imu(self, imu: ImuMeasurement) -> None:
        """将一条 IMU 数据推入缓冲。"""
        self.imu_buffer.append(imu)

    def harvest(self, gnss: GnssSolution) -> Optional[AlignedBlock]:
        """对一个 GNSS 历元做收割。

        每次调用都丢弃 timestamp < t_gnss 的首部 IMU（含上一块截止到当前 GNSS 之间的部分），
        收割缓冲中 t_gnss <= timestamp < t_gnss + harvest_window 的 IMU。

        Returns:
            AlignedBlock 或 None（收割列表为空）
        """
        t_lo = gnss.timestamp
        t_hi = gnss.timestamp + self.harvest_window
        # 丢弃 timestamp < t_gnss 的首部 IMU
        while self.imu_buffer and self.imu_buffer[0].timestamp < t_lo:
            self.imu_buffer.popleft()
        # 收割 t_gnss <= timestamp < t_gnss + harvest_window 的 IMU
        imu_list: List[ImuMeasurement] = []
        while self.imu_buffer and self.imu_buffer[0].timestamp < t_hi:
            imu_list.append(self.imu_buffer.popleft())
        if not imu_list:
            return None
        return AlignedBlock(gnss=gnss, imu_list=imu_list)
