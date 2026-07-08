"""松组合导航集成 (最近邻时间对齐 + 主循环)。

参考:
- KF-GINS newImuProcess (imupre/imucur/pending_gnss 结构)
- 项目约定: 最近邻匹配 (替换 KF-GINS 增量切分, 适配速率式 IMU)

主循环:
  add_imu: imupre←imucur, imucur←imu; time_update; _try_gnss_update
  add_gnss: append 到 pending_gnss
  _try_gnss_update: 最近邻判断, imucur 更近则 meas_update + feedback
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

        self.imupre: Optional[ImuMeasurement] = None
        self.imucur: Optional[ImuMeasurement] = None
        self.pending_gnss: collections.deque = collections.deque()

    def add_imu(self, imu: ImuMeasurement) -> None:
        """添加 IMU: 更新 imupre/imucur, time_update, 尝试 GNSS 量测更新。"""
        if self.imucur is not None:
            self.imupre = self.imucur
        self.imucur = imu
        if self.imupre is None:
            return
        self.est.time_update(imu)
        self._try_gnss_update()

    def add_gnss(self, gnss: GnssSolution) -> None:
        """添加 GNSS: append 到 pending_gnss deque。"""
        self.pending_gnss.append(gnss)

    def _try_gnss_update(self) -> None:
        """最近邻判断: 若 imucur 距 gnss 最近, 触发量测更新 + feedback。"""
        while self.pending_gnss:
            gnss = self.pending_gnss[0]
            d_cur = abs(self.imucur.timestamp - gnss.timestamp)
            d_pre = abs(self.imupre.timestamp - gnss.timestamp)
            if d_cur <= d_pre:
                self.est.meas_update_pos(gnss)
                if gnss.velocity is not None:
                    self.est.meas_update_vel(gnss)
                self._apply_nhc_or_zupt()
                self.est.feedback()
                self.pending_gnss.popleft()
            else:
                if gnss.timestamp < self.imupre.timestamp - 1.0:
                    logger.warning(
                        f"丢弃过时 GNSS 历元 t={gnss.timestamp:.6f} "
                        f"(imupre={self.imupre.timestamp:.6f})"
                    )
                    self.pending_gnss.popleft()
                    continue
                break

    def _apply_nhc_or_zupt(self) -> None:
        """NHC/ZUPT 互斥选择 (三阈值系统)。"""
        speed = float(np.linalg.norm(self.est.state.vel_e))
        w_b_ib = self.est.ins_update.w_b_ib
        omega_norm = float(np.linalg.norm(w_b_ib))
        if speed < self.static_speed_threshold:
            self.est.meas_update_zupt()
        elif omega_norm < self.angular_velocity_threshold:
            self.est.meas_update_nhc(self.imucur)
