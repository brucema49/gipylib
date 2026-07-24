"""紧组合估计器 (继承 LcEstimator)。

扩展:
  - 状态向量加 GNSS 参数块 (clk_bias/ambiguity)
  - time_update: 父类 INS 传播 + 钟差随机游走
  - tc_meas_update: 调 joseph_update + feedback
  - feedback: 父类 INS 反馈, 但保留 GNSS 参数 (clk/amb 为直接估计, 非 ψ-error)
  - switch_mode: 降级时状态向量重整
  - reboot: 重启保留随机游走参数

关键区别:
  - INS 部分 (pos/vel/att/bias): ψ-error, feedback 校正物理状态后清零
  - GNSS 部分 (clk_bias/ambiguity): 直接估计, x 中即真值, feedback 不清零
"""
import numpy as np

from src.core.ins.lc_estimator import LcEstimator
from src.core.ins.state_index import StateIndex
from src.core.tc.tc_state_index import TcStateIndex


class TcEstimator(LcEstimator):
    """紧组合 EKF 估计器。"""

    def __init__(self, state, P, config, mode: str):
        self._tc_config = config
        self._mode = mode
        # 先用基类构建 INS 部分 (内部创建 StateIndex)
        super().__init__(state, P, config)
        # 替换为 TcStateIndex (扩展 GNSS 参数块)
        tc_si = TcStateIndex.from_config(config, mode)
        # 保留 INS 基础部分, 扩展到 tc_si.dim
        old_dim = self.si.dim   # 基类 StateIndex dim (仅 INS+可选块, 无 GNSS 块)
        if tc_si.dim > old_dim:
            P_ext = np.eye(tc_si.dim) * 100.0 ** 2
            P_ext[:old_dim, :old_dim] = self.P[:old_dim, :old_dim]
            x_ext = np.zeros(tc_si.dim)
            x_ext[:old_dim] = self.x[:old_dim]
            self.P = P_ext
            self.x = x_ext
        # 重建 TransferMatrix (用 tc_si, dim 更大但 INS 索引相同)
        from src.core.ins.transfer_matrix import TransferMatrix
        self.tm = TransferMatrix(config, tc_si)
        self.si = tc_si
        self._clk_q = 1e-2   # 钟差随机游走 PSD (m²/s)

    def time_update(self, imu):
        """父类 INS 传播 + 钟差随机游走。"""
        prev_ts = self.ins_update._prev_timestamp
        super().time_update(imu)
        dt = imu.timestamp - prev_ts
        if dt > 0 and self.si.clk_bias >= 0:
            for k in range(3):
                self.P[self.si.clk_bias + k, self.si.clk_bias + k] += self._clk_q * dt

    def tc_meas_update(self, v, H, R, source: str = ""):
        """GNSS 量测更新 (调 joseph_update + feedback)。"""
        if len(v) == 0:
            return
        self.joseph_update(v, H, R)
        self.feedback()

    def feedback(self) -> None:
        """反馈校正: 父类 INS 部分, 但保留 GNSS 参数。

        GNSS 参数 (clk_bias/ambiguity) 是直接估计 (非 ψ-error),
        feedback 后不清零, 保留在 x 中供下次量测构造使用。
        """
        # 保存 GNSS 参数
        si = self.si
        gnss_x = {}
        if si.clk_bias >= 0:
            gnss_x['clk'] = self.x[si.clk_bias:si.clk_bias + 3].copy()
        if si.has_ambiguity():
            gnss_x['amb'] = self.x[si.amb_start:si.amb_start + si.n_amb].copy()
        # 调父类 feedback (校正 INS + 清零全部 x)
        super().feedback()
        # 恢复 GNSS 参数
        if 'clk' in gnss_x:
            self.x[si.clk_bias:si.clk_bias + 3] = gnss_x['clk']
        if 'amb' in gnss_x:
            self.x[si.amb_start:si.amb_start + si.n_amb] = gnss_x['amb']

    def switch_mode(self, new_mode: str, builder=None):
        """降级时状态向量重整。

        保留: INS + 可选块 (pos/vel/att/bias/lever/angle/leverarm/time_sync)
        重置: clk_bias (→0, P=100²), ambiguity (→清空)
        """
        old_si = self.si
        new_si = TcStateIndex.from_config(self._tc_config, new_mode)
        keep = old_si._gnss_base
        new_P = np.eye(new_si.dim) * 100.0 ** 2
        new_P[:keep, :keep] = self.P[:keep, :keep]
        new_x = np.zeros(new_si.dim)
        new_x[:keep] = self.x[:keep]
        self.si = new_si
        self.P = new_P
        self.x = new_x
        self._mode = new_mode
        if builder is not None:
            self._meas_builder = builder

    def reboot(self, keep_random_walk: bool = True):
        """重启: 位置/速度/姿态重置, 保留 gyro_bias/accel_bias/lever 等。"""
        si = self.si
        keep_idx = []
        keep_idx.extend(range(si.gyro_bias, si.gyro_bias + 3))
        keep_idx.extend(range(si.accel_bias, si.accel_bias + 3))
        if si.has_lever_arm():
            keep_idx.extend(range(si.lever_arm, si.lever_arm + 3))
        if si.has_imu_angle():
            keep_idx.extend(range(si.imu_angle, si.imu_angle + 2))
        if si.has_imu_leverarm():
            keep_idx.extend(range(si.imu_leverarm, si.imu_leverarm + 3))
        keep_set = set(keep_idx)
        for i in range(si.dim):
            if i not in keep_set:
                self.x[i] = 0.0
                self.P[i, i] = 100.0 ** 2
        # INS 物理状态重置
        self.state.pos_e[:] = 0.0
        self.state.vel_e[:] = 0.0
        self.state.C_b_e = np.eye(3)
