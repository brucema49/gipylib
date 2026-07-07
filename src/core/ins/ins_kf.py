"""INS 卡尔曼滤波协方差传播 (P1 主滤波)。

参考:
- GINav ins_time_updata.m (每历元传播, 中间值法)
- gnss_ins_lc_nhc navfilter.cc TimeUpdate (P = Φ·P·Φ^T + Q)

本阶段为开环模式: 只传播 P1, 不做量测更新, 不反馈修正 InsState。
P = Φ·(P + 0.5Q)·Φ^T + 0.5Q  (GINav 中间值法)
"""
import logging

import numpy as np

from src.core.data_types import ImuMeasurement
from src.core.ins.transfer_matrix import TransferMatrix

logger = logging.getLogger(__name__)


class InsKf:
    """INS 卡尔曼滤波器 (协方差传播)。

    维护 15x15 P1 主滤波协方差矩阵。
    开环模式: 只传播, 不修正状态。
    """

    def __init__(self, P1: np.ndarray, config: dict):
        self._P1 = P1.copy()
        self._tm = TransferMatrix(config)
        logger.info(
            f"InsKf 初始化: P1 shape={P1.shape}, "
            f"trace={np.trace(P1):.6e}"
        )

    @property
    def P1(self) -> np.ndarray:
        return self._P1

    def propagate(self, imu: ImuMeasurement, ins_core, prev_timestamp: float) -> None:
        """协方差传播: P1 = Φ·(P1 + 0.5Q)·Φ^T + 0.5Q。

        Args:
            imu: 当前 IMU 测量
            ins_core: InsCore 实例 (提供 C_b_e, f_b, w_b_ib)
            prev_timestamp: 上一历元时间戳 (用于计算 dt)
        """
        dt = imu.timestamp - prev_timestamp
        if dt <= 0.0:
            return

        # 从 InsCore 获取当前状态量
        C_b_e = ins_core.state.C_b_e
        f_b = ins_core.f_b
        w_b_ib = ins_core.w_b_ib

        # 构造 F, Φ, Q
        F = self._tm.build_F(C_b_e, f_b, w_b_ib)
        Phi = self._tm.build_Phi(F, dt)
        Q = self._tm.build_Q(dt, C_b_e)

        # GINav 中间值法: P = Φ·(P + 0.5Q)·Φ^T + 0.5Q
        P0 = self._P1 + 0.5 * Q
        self._P1 = Phi @ P0 @ Phi.T + 0.5 * Q

        # 对称化 (数值稳定性)
        self._P1 = 0.5 * (self._P1 + self._P1.T)
