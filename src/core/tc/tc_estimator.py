"""紧组合估计器 (继承 LcEstimator)。

扩展:
  - 状态向量加 GNSS 参数块 (clk_bias/ambiguity)
  - time_update: 父类 INS 传播 + 钟差随机游走
  - tc_meas_update: 调 joseph_update + feedback
  - feedback: 父类 INS 反馈 + GNSS 直接状态累积到 stored
  - switch_mode: 降级时状态向量重整
  - reboot: 重启保留随机游走参数

状态语义 (与 GINav/GREAT-MSF 一致的 innovation 形式):
  - INS 部分 (pos/vel/att/bias): ψ-error, x 存误差 ε, feedback: state -= ε, ε 清零
  - GNSS 部分 (clk_bias/ambiguity): 直接估计, x 存修正量 ε (correction)
      effective = stored + ε (ADD, 修正量加到直接估计)
      feedback: stored += ε, ε 清零
"""
import numpy as np

from src.core.ins.lc_estimator import LcEstimator
from src.core.ins.state_index import StateIndex
from src.core.tc.tc_state_index import TcStateIndex


def _expm_small(A: np.ndarray, order: int = 10) -> np.ndarray:
    """小矩阵指数 (scaling-and-squaring + Taylor 级数, 不依赖 scipy)。

    供 time_update 分块优化使用 (仅 INS 块, 维度 ≤ 25)。
    """
    n = A.shape[0]
    norm = float(np.linalg.norm(A, np.inf))
    s = int(np.ceil(np.log2(norm))) if norm > 1.0 else 0
    A_scaled = A / (2.0 ** s)
    result = np.eye(n, dtype=np.float64)
    term = np.eye(n, dtype=np.float64)
    for k in range(1, order + 1):
        term = term @ A_scaled / k
        result += term
    for _ in range(s):
        result = result @ result
    return result


class TcEstimator(LcEstimator):
    """紧组合 EKF 估计器。"""

    def __init__(self, state, P, config, mode: str):
        self._tc_config = config
        self._mode = mode
        # 先用基类构建 INS 部分 (内部创建 StateIndex)
        super().__init__(state, P, config)
        # 替换为 TcStateIndex (扩展 GNSS 参数块)
        tc_si = TcStateIndex.from_config(config, mode)
        # RTK 模式: 预分配 ambiguity 槽位 (MAXSAT * nf)
        # 必须在 dim 检查前设置, 否则 has_ambiguity()=False 导致 H 矩阵维度不匹配
        if mode == "rtk":
            from src.core.gnss.rtklib.rtkcmn import uGNSS
            nf = int(config.get("gnss", {}).get("nf", 2))
            tc_si.set_ambiguity_count(uGNSS.MAXSAT * nf)
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
        # GNSS 直接估计 (stored): clk/amb 的当前最佳估计
        # x[clk_bias]/x[amb] 存修正量 ε (correction), feedback 时累积到 stored
        self._clk_stored = np.zeros(3, dtype=np.float64)
        self._N_stored = np.zeros(tc_si.n_amb if tc_si.has_ambiguity() else 0,
                                  dtype=np.float64)

    def effective_x(self) -> np.ndarray:
        """构造有效状态向量供量测构造: stored + x (ε)。

        - INS 部分 (pos/vel/att/bias): x 本身即误差 ε (量测用 nominal state, 不需 effective)
        - GNSS 直接状态 (clk_bias/ambiguity): effective = stored + ε
          (ε 为 KF 估计的修正量, stored 为累积的直接估计)
        """
        si = self.si
        x = self.x.copy()
        if si.clk_bias >= 0:
            x[si.clk_bias:si.clk_bias + 3] = (
                self._clk_stored + self.x[si.clk_bias:si.clk_bias + 3])
        if si.has_ambiguity():
            amb_slice = slice(si.amb_start, si.amb_start + si.n_amb)
            # 动态调整 _N_stored 尺寸 (set_ambiguity_count 可能改变 n_amb)
            if len(self._N_stored) != si.n_amb:
                self._N_stored = np.zeros(si.n_amb, dtype=np.float64)
            x[amb_slice] = self._N_stored + self.x[amb_slice]
        return x

    def time_update(self, imu):
        """父类 INS 传播 + 钟差白噪声 (参考 ignav propP: 每历元重置方差)。

        性能优化: 利用块结构 (INS 块 + GNSS 块) 避免全 n×n 矩阵乘法。
        - F 非零仅 INS 块 (前 _gnss_base 维), GNSS 块 F=0 → Phi_GNSS=I
        - Q 非零仅 INS 块, GNSS 块 Q=0 (ambiguity 随机游走 Q=0, clk 白噪声单独处理)
        - P 传播分块:
          P_INS = Phi_INS @ (P_INS + 0.5*Q_INS) @ Phi_INS.T + 0.5*Q_INS
          P_INS_GNSS = Phi_INS @ P_INS_GNSS  (交叉项, 仅左乘)
          P_GNSS_GNSS 不变 (Phi=I, Q=0)
        复杂度从 O(n³) 降到 O(n_INS²·n_GNSS + n_INS³), n=354 时约 500x 加速。
        """
        prev_ts = self.ins_update._prev_timestamp
        self.ins_update.update(imu)

        dt = imu.timestamp - prev_ts
        if dt <= 0.0:
            return

        si = self.si
        n_ins = si._gnss_base  # INS + 可选块维度 (clk/amb 之前)
        n_total = si.dim

        C_b_e = self.ins_update.state.C_b_e
        f_b = self.ins_update.f_b
        w_b_ib = self.ins_update.w_b_ib
        pos_e = self.ins_update.state.pos_e

        # 仅构建 INS 块的 F/Phi/Q (n_ins × n_ins), 避免全 n×n 矩阵
        F_ins = self.tm.build_F_ins(C_b_e, f_b, w_b_ib, pos_e, n_ins)
        Fdt = F_ins * dt
        if dt <= 0.005:
            Phi_ins = np.eye(n_ins, dtype=np.float64) + Fdt
        elif dt <= 0.01:
            Phi_ins = np.eye(n_ins, dtype=np.float64) + Fdt + 0.5 * (Fdt @ Fdt)
        else:
            Phi_ins = _expm_small(Fdt)

        Q_ins = self.tm.build_Q_ins(dt, C_b_e, n_ins)

        # 分块传播 P
        if n_total == n_ins:
            # 无 GNSS 块
            P0 = self.P + 0.5 * Q_ins
            self.P = Phi_ins @ P0 @ Phi_ins.T + 0.5 * Q_ins
        else:
            # 分块: P = [P_INS, P_cross; P_cross.T, P_GNSS]
            P_INS = self.P[:n_ins, :n_ins]
            P_cross = self.P[:n_ins, n_ins:]

            P0_INS = P_INS + 0.5 * Q_ins
            new_P_INS = Phi_ins @ P0_INS @ Phi_ins.T + 0.5 * Q_ins
            new_P_cross = Phi_ins @ P_cross  # 仅左乘

            self.P[:n_ins, :n_ins] = new_P_INS
            self.P[:n_ins, n_ins:] = new_P_cross
            self.P[n_ins:, :n_ins] = new_P_cross.T
            # P_GNSS 不变 (Phi=I, Q=0)

        self.P = 0.5 * (self.P + self.P.T)
        # 钟差按白噪声处理: 不在 IMU 时间更新中累积方差
        # 在 tc_meas_update 前重置钟差方差 (ignav propP initP(irc,...,UNC_CLK))

    def reset_clk_variance(self):
        """重置钟差方差 (每个 GNSS 历元前调用, 参考 ignav propP 的 initP(irc,...,UNC_CLK))。

        ignav 将钟差视为白噪声: 每个历元方差重置为 UNC_CLK² = 10000.0,
        使 KF 对钟差有高增益, 防止钟差误差累积导致发散。
        """
        if self.si.clk_bias >= 0:
            for k in range(3):
                self.P[self.si.clk_bias + k, self.si.clk_bias + k] = 100.0 ** 2

    def tc_meas_update(self, v, H, R, source: str = ""):
        """GNSS 量测更新 (调 joseph_update + feedback)。"""
        if len(v) == 0:
            return
        self.joseph_update(v, H, R)
        self.feedback()

    def feedback(self) -> None:
        """反馈校正: INS 误差减, GNSS 直接状态累积到 stored。

        GNSS 直接状态 (clk_bias/ambiguity) 的 x 存修正量 ε (correction):
          - stored += ε (累积修正到直接估计)
          - ε 清零 (由父类 feedback 完成)
        INS 误差状态由父类 feedback 处理 (state -= x, x 清零)。
        """
        si = self.si
        # 累积 GNSS 直接状态修正到 stored
        if si.clk_bias >= 0:
            self._clk_stored += self.x[si.clk_bias:si.clk_bias + 3]
        if si.has_ambiguity():
            amb_slice = slice(si.amb_start, si.amb_start + si.n_amb)
            if len(self._N_stored) != si.n_amb:
                self._N_stored = np.zeros(si.n_amb, dtype=np.float64)
            self._N_stored += self.x[amb_slice]
        # 父类 feedback: INS state -= x, 清零全部 x (含 GNSS ε)
        super().feedback()

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
        # 重建 TransferMatrix (用 new_si, dim 可能因 clk_bias 块变化而改变)
        # 否则 build_Q 会用旧 si.dim 生成 Q, 与新 P 维度不匹配
        from src.core.ins.transfer_matrix import TransferMatrix
        self.tm = TransferMatrix(self._tc_config, new_si)
        # 重置 GNSS 直接估计 (新模式从零开始)
        self._clk_stored = np.zeros(3, dtype=np.float64)
        self._N_stored = np.zeros(new_si.n_amb if new_si.has_ambiguity() else 0,
                                  dtype=np.float64)
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
        # GNSS 直接估计重置
        self._clk_stored[:] = 0.0
        self._N_stored = np.zeros(si.n_amb if si.has_ambiguity() else 0,
                                  dtype=np.float64)
