"""松组合导航集成 (最近邻时间对齐 + 主循环)。

参考:
- KF-GINS newImuProcess (imupre/imucur/pending_gnss 结构)
- 项目约定: 最近邻匹配 (替换 KF-GINS 增量切分, 适配速率式 IMU)

主循环 (每个新 IMU 到来时):
  1. 预推进检查: 若 pending GNSS 更近于当前 imucur (即旧 imucur),
     用当前 state (imucur 时刻) 做量测更新 + 反馈, 然后出队。
  2. 推进: imupre←imucur, imucur←imu, time_update。
  3. 推进后检查: 若 pending GNSS 更近于新 imucur, 做量测更新 + 反馈。

GNSS 由 add_gnss 入队, 不直接触发更新 (等下一个 IMU 决定最近邻)。
"""
import collections
import logging
import math
from typing import Optional

import numpy as np

from src.core.data_types import GnssSolution, ImuMeasurement

logger = logging.getLogger(__name__)


class LcIntegration:
    """松组合导航集成 (最近邻时间对齐 + 双滤波 EKF 主循环)。"""

    def __init__(self, estimator, config: dict):
        self.est = estimator
        ins_cfg = config.get("ins", {})
        self.static_speed_threshold = ins_cfg.get("static_speed_threshold", 0.5)
        self.angular_velocity_threshold = ins_cfg.get(
            "angular_velocity_threshold", 30.0 * math.pi / 180.0)
        # 超过此秒数认为 GNSS 已过期, 丢弃 (避免用错位状态做更新)
        self._stale_threshold = 0.5

        self.imupre: Optional[ImuMeasurement] = None
        self.imucur: Optional[ImuMeasurement] = None
        self.pending_gnss: collections.deque = collections.deque()

    def add_imu(self, imu: ImuMeasurement) -> None:
        """添加 IMU: 预推进 GNSS 处理 → 推进 → 推进后 GNSS 处理。

        NHC/ZUPT 在 GNSS 更新时 (1Hz) 应用, 与量测更新同步。
        """
        if self.imucur is None:
            self.imucur = imu
            return

        cur_t = self.imucur.timestamp
        new_t = imu.timestamp

        # 1. 预推进: 处理更近于当前 imucur 的 pending GNSS
        self._process_pending(cur_t, new_t, pre_advance=True)

        # 2. 推进 (机械编排 + P 协方差传播)
        self.imupre = self.imucur
        self.imucur = imu
        self.est.time_update(imu)

        # 3. 推进后: 处理更近于新 imucur 的 pending GNSS
        self._process_pending(self.imucur.timestamp, self.imupre.timestamp,
                              pre_advance=False)

    def add_gnss(self, gnss: GnssSolution) -> None:
        """添加 GNSS: append 到 pending_gnss deque。"""
        self.pending_gnss.append(gnss)

    def _process_pending(self, t_ref: float, t_other: float,
                         pre_advance: bool) -> None:
        """处理 pending GNSS 队列。

        Args:
            t_ref: 参考时刻 (pre_advance: 当前 imucur; post_advance: 新 imucur)
            t_other: 对比时刻 (pre_advance: 新 imu; post_advance: 新 imupre)
            pre_advance: True 表示推进前 (用当前 state), False 表示推进后
        """
        while self.pending_gnss:
            gnss = self.pending_gnss[0]
            d_ref = abs(t_ref - gnss.timestamp)
            d_other = abs(t_other - gnss.timestamp)

            if pre_advance:
                # 推进前: 若 imucur 更近 (或并列), 用当前 state 处理
                should_process = d_ref <= d_other
            else:
                # 推进后: 若新 imucur 严格更近, 用新 state 处理
                # (并列情况已在推进前处理, 这里不重复)
                should_process = d_ref < d_other

            if should_process:
                # 丢弃严重过期的 GNSS (距参考时刻 > 阈值)
                if d_ref > self._stale_threshold and gnss.timestamp < t_ref:
                    logger.warning(
                        f"丢弃过期 GNSS t={gnss.timestamp:.6f} "
                        f"(t_ref={t_ref:.6f}, d={d_ref:.4f}s)"
                    )
                    self.pending_gnss.popleft()
                    continue
                self._apply_gnss_update(gnss)
                self.pending_gnss.popleft()
            else:
                # GNSS 更近于另一侧, 等待
                break

    def _apply_gnss_update(self, gnss: GnssSolution) -> None:
        """应用 GNSS 位置/速度量测更新 + NHC/ZUPT + 反馈。"""
        self.est.meas_update_pos(gnss)
        if gnss.velocity is not None:
            self.est.meas_update_vel(gnss)
        self._apply_nhc_or_zupt()
        self.est.feedback()

    def _apply_nhc_or_zupt(self) -> None:
        """NHC/ZUPT 互斥选择 (三阈值系统)。"""
        speed = float(np.linalg.norm(self.est.state.vel_e))
        w_b_ib = self.est.ins_update.w_b_ib
        omega_norm = float(np.linalg.norm(w_b_ib))
        if speed < self.static_speed_threshold:
            self.est.meas_update_zupt()
        elif omega_norm < self.angular_velocity_threshold:
            self.est.meas_update_nhc(self.imucur)
