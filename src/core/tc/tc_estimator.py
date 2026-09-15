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
from src.core.tc.tc_measurement import validate_tc_postfit


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
        self._clk_stored = np.zeros(4, dtype=np.float64)
        self._N_stored = np.zeros(tc_si.n_amb if tc_si.has_ambiguity() else 0,
                                  dtype=np.float64)
        # Position feedback allocation over future IMU samples is explicitly
        # disabled.  GREAT closes the error state at the GNSS boundary; a
        # deferred correction would alter that model rather than repair it.
        # Keep no pending correction state in the estimator.
        self._last_propagation_snapshot = None
        # A trace-only post-fit payload is frozen at the Joseph boundary and
        # consumed by TcIntegration after its final acceptance/rollback gate.
        self._pending_tc_postfit_trace = None
        # Candidate row recorded when the transactional post-fit check fails;
        # no automatic scalar/satellite pruning is performed in this unified
        # TC estimator because its sparse DD state is not GREAT's outer GNSS
        # parameter filter.
        self.last_tc_outlier_indices = tuple()
        self.last_tc_outlier_row = None
        self.last_tc_postfit = None
        self.last_tc_active_row_indices = tuple()
        self.last_tc_update_v = None
        self.last_tc_update_H = None
        self.last_tc_update_R = None
        self.last_tc_update_info = None

    @property
    def last_propagation_snapshot(self):
        """Latest copied 15x15 INS propagation snapshot for diagnostics."""
        return self._last_propagation_snapshot

    def effective_x(self) -> np.ndarray:
        """构造有效状态向量供量测构造: stored + x (ε)。

        - INS 部分 (pos/vel/att/bias): x 本身即误差 ε (量测用 nominal state, 不需 effective)
        - GNSS 直接状态 (clk_bias/ambiguity): effective = stored + ε
          (ε 为 KF 估计的修正量, stored 为累积的直接估计)
        """
        si = self.si
        x = self.x.copy()
        if si.clk_bias >= 0:
            x[si.clk_bias:si.clk_bias + 4] = (
                self._clk_stored + self.x[si.clk_bias:si.clk_bias + 4])
        if si.has_ambiguity():
            amb_slice = slice(si.amb_start, si.amb_start + si.n_amb)
            # 动态调整 _N_stored 尺寸 (set_ambiguity_count 可能改变 n_amb)
            if len(self._N_stored) != si.n_amb:
                self._N_stored = np.zeros(si.n_amb, dtype=np.float64)
            x[amb_slice] = self._N_stored + self.x[amb_slice]
        return x

    def time_update(self, imu):
        """父类 INS 传播 + 钟差随机游走。

        性能优化: 利用块结构 (INS 块 + GNSS 块) 避免全 n×n 矩阵乘法。
        - F 非零仅 INS 块 (前 _gnss_base 维), GNSS 块 F=0 → Phi_GNSS=I
        - Q 非零仅 INS 块, GNSS 块 Q=0 (ambiguity 随机游走 Q=0, clk 见下)
        - P 传播分块:
          P_INS = Phi_INS @ (P_INS + 0.5*Q_INS) @ Phi_INS.T + 0.5*Q_INS
          P_INS_GNSS = Phi_INS @ P_INS_GNSS  (交叉项, 仅左乘)
          P_GNSS_GNSS 不变 (Phi=I), 但 clk 对角 += Q_clk*dt (随机游走)
        复杂度从 O(n³) 降到 O(n_INS²·n_GNSS + n_INS³), n=354 时约 500x 加速。

        钟差随机游走: Q_clk = sigma_clk² * dt, sigma_clk ≈ 0.3 m/sqrt(s)
        (典型 GPS 接收机晶振短期稳定度 ~1e-9 s/s ≈ 0.3 m/s)。
        不再每历元重置 Pclk (原 ignav 白噪声模型会导致 Pclk=100 >> Ppos,
        钟差吸收全部 innovation, 位置修正不足, 高度误差累积)。
        """
        # The nominal mechanization integrates this interval from its
        # beginning state.  Linearizing F/G (and therefore Phi/Q) at that same
        # beginning state is required for a consistent discrete EKF; otherwise the
        # attitude/force and gravity-gradient blocks describe a different
        # operating point than the state transition just applied.
        prev_C_b_e = self.ins_update.state.C_b_e.copy()
        prev_pos_e = self.ins_update.state.pos_e.copy()
        self.ins_update.update(imu)

        if not self.ins_update.last_update_accepted:
            self._last_propagation_snapshot = None
            return

        # InsUpdate is the single source of truth for the accepted segment
        # interval: increment payloads provide their exact dt, while rate
        # samples retain the legacy timestamp-delta behavior.  Reusing it
        # here keeps Phi/Q on the same interval as mechanization and avoids
        # reconstructing a segment duration from rounded Unix timestamps.
        dt = float(self.ins_update.last_dt)
        if dt <= 0.0:
            self._last_propagation_snapshot = None
            return

        si = self.si
        n_ins = si._gnss_base  # INS + 可选块维度 (clk/amb 之前)
        n_total = si.dim

        C_b_e = prev_C_b_e
        f_b = self.ins_update.f_b
        w_b_ib = self.ins_update.w_b_ib
        pos_e = prev_pos_e

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
        p_before_15 = self.P[:15, :15].copy()

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
            # P_GNSS 不变 (Phi=I, Q=0), 但 clk 对角 += Q_clk*dt (随机游走)
            if si.clk_bias >= 0:
                clk0 = si.clk_bias
                # 钟差随机游走: sigma_clk = 0.1 m/sqrt(s) (紧模型, 防止 clock 吸收 height error)
                # sigma=0.1 → Q_clk=0.01 m²/s, Pclk 平衡 ~0.1m² (远小于 Ppos~0.5)
                # 使 K[pos] >> K[clk], 位置获得足够修正对抗 IMU 高度漂移
                q_clk = 0.1 ** 2 * dt   # m² (per IMU step)
                for k in range(4):
                    self.P[clk0 + k, clk0 + k] += q_clk

        # Preserve and propagate any mean error left by partial position
        # feedback using the same INS transition as the covariance.
        if np.any(self.x[:n_ins]):
            self.x[:n_ins] = Phi_ins @ self.x[:n_ins]

        self.P = 0.5 * (self.P + self.P.T)

        self._last_propagation_snapshot = {
            "timestamp": float(self.ins_update.state.timestamp),
            "dt": float(dt),
            "p_before": p_before_15,
            "p_after": self.P[:15, :15].copy(),
            "phi": Phi_ins[:15, :15].copy(),
            "q": Q_ins[:15, :15].copy(),
        }

    def reset_clk_variance(self):
        """重置钟差误差状态 (每个 GNSS 历元前调用)。

        钟差随机游走模型: Pclk 不再重置, 由 time_update 的 Q_clk 累积。
        仅清零误差状态 ε_clk=0, 使 effective clk = _clk_stored (上一历元累积值)。

        关键: 不重置 Pclk。
        - 旧实现 (ignav 白噪声): 每历元 Pclk=100, 导致 K[clk] >> K[pos],
          钟差吸收全部 innovation, 位置修正不足 → 高度误差累积 (t+1400-1560s 偏差 21m)。
        - 新实现 (随机游走): Pclk 由 Q_clk 累积 + 量测更新收敛, 平衡 ~0.1-1 m²,
          K[pos] 与 K[clk] 量级相当, 位置能获得足够修正。
        - _clk_stored 保留跨历元记忆, 误差状态 ε_clk 清零后用 stored 作有效估计。
        """
        si = self.si
        if si.clk_bias >= 0:
            clk0 = si.clk_bias
            # 仅清零 clk 误差状态 (ε_clk=0, 用 _clk_stored 作有效估计)
            # Pclk 不重置, 由 time_update 的 Q_clk 随机游走累积
            self.x[clk0:clk0 + 4] = 0.0

    @staticmethod
    def _normalise_retry_groups(retry_groups, n_rows: int):
        """Return deterministic row groups for a GREAT-style retry.

        The groups are DD metadata supplied by the builder.  Removing one is
        an atomic satellite-level operation; it never changes a scalar
        residual threshold or any covariance/noise parameter.
        """
        if retry_groups is None:
            return []
        values = (retry_groups.values() if isinstance(retry_groups, dict)
                  else retry_groups)
        try:
            iterator = iter(values)
        except TypeError:
            return []
        result, seen = [], set()
        for group in iterator:
            try:
                rows = sorted({int(index) for index in group
                               if 0 <= int(index) < int(n_rows)})
            except (TypeError, ValueError):
                continue
            key = tuple(rows)
            if rows and key not in seen:
                result.append(rows)
                seen.add(key)
        return result

    def _postfit_result(self, v, H, R):
        """Compute GREAT-compatible normalized post-fit diagnostics."""
        postfit = np.asarray(v, dtype=np.float64) - H @ self.x
        covariance = H @ self.P @ H.T + R
        variances = np.diag(covariance)
        normalized = np.full(postfit.size, np.nan, dtype=np.float64)
        valid = variances > 0.0
        normalized[valid] = np.abs(postfit[valid]) / np.sqrt(variances[valid])
        bad_row = None
        if np.any(np.isfinite(normalized)):
            bad_row = int(np.nanargmax(normalized))
        accepted = validate_tc_postfit(
            postfit, covariance, n_parameters=3)
        return accepted, postfit, covariance, bad_row

    def tc_meas_update(self, v, H, R, source: str = "", trace=None,
                       retry_groups=None, retry_satellites=None,
                       retry_builder=None):
        """GNSS 量测更新 (Joseph + GREAT 同历元重建 + immediate feedback).

        GREAT 在同一 GNSS 端点删除最大归一化残差所属卫星并重建外层
        GNSS 方程。正式 GREAT 兼容路径通过 ``retry_builder`` 重新选择
        参考星并重算 DD 协方差；旧的冻结行切片仅保留给无重建回调的
        legacy 单元测试，不作为正式解算策略。不分配反馈到未来时间，
        也不修改任何噪声、权重或门限。
        """
        if len(v) == 0:
            return None
        if trace is not None:
            self._pending_tc_postfit_trace = {
                "trace": trace,
                "postfit": None,
                "x_post": None,
                "P_post": None,
            }
        P_before = self.P.copy()
        x_before = self.x.copy()
        v_full = np.asarray(v, dtype=np.float64).reshape(-1)
        H_full = np.asarray(H, dtype=np.float64)
        R_full = np.asarray(R, dtype=np.float64)
        self.last_tc_postfit = None
        self.last_tc_active_row_indices = tuple()
        self.last_tc_outlier_indices = tuple()
        self.last_tc_outlier_row = None
        self.last_tc_update_v = v_full.copy()
        self.last_tc_update_H = H_full.copy()
        self.last_tc_update_R = R_full.copy()
        self.last_tc_update_info = None

        # The TC path bypasses rtklib's relpos()/valpos() wrapper.  Validate
        # before closing the INS loop, and keep rejection transactional.
        self.joseph_update(v_full, H_full, R_full)
        postfit = None
        active_rows = list(range(v_full.size))
        accepted = True
        if source == "rtk":
            accepted, postfit, _postfit_covariance, bad_row = \
                self._postfit_result(v_full, H_full, R_full)
            if bad_row is not None:
                self.last_tc_outlier_row = int(bad_row)

            retry_groups = self._normalise_retry_groups(
                retry_groups, v_full.size)
            retry_satellites = list(retry_satellites or ())
            excluded_rows = set()
            excluded_sats = set()
            # GREAT removes the complete satellite owning the worst row and
            # retries the outer update at the same endpoint.  The builder has
            # already frozen DD rows, so only the normal equations are sliced;
            # there is no second ambiguity synchronization or retuning.  A
            # frozen DD model is not allowed to perform repeated removals:
            # after one conservative retry, failure means rejecting the epoch
            # rather than silently changing the reference/observability model.
            retry_attempted = False
            while (not accepted and bad_row is not None
                   and retry_groups):
                original_bad = active_rows[int(bad_row)]
                group_index = next((index for index, candidate in enumerate(
                    retry_groups)
                    if original_bad in candidate
                    and (
                        (retry_builder is not None
                         and index < len(retry_satellites)
                         and int(retry_satellites[index]) not in excluded_sats)
                        or (retry_builder is None
                            and not set(candidate).issubset(excluded_rows))
                    )), None)
                group = (retry_groups[group_index]
                         if group_index is not None else None)
                if group is None:
                    break
                sat = (retry_satellites[group_index]
                       if group_index is not None
                       and group_index < len(retry_satellites) else None)
                if retry_builder is not None and sat is not None:
                    # Rebuild the observation model at the same endpoint.
                    # The EKF state/covariance are restored to the exact
                    # pre-update snapshot before every trial, matching
                    # GREAT's outer do/while rather than slicing a frozen
                    # normal equation.
                    excluded_sats.add(int(sat))
                    excluded_rows.update(group)
                    self.P = P_before.copy()
                    self.x = x_before.copy()
                    rebuilt = retry_builder(
                        tuple(sorted(excluded_sats)),
                        x_retry=self.effective_x(),
                        P_retry=P_before)
                    if rebuilt is None or len(rebuilt) != 4:
                        break
                    v_try, H_try, R_try, info_try = rebuilt
                    v_try = np.asarray(v_try, dtype=np.float64).reshape(-1)
                    H_try = np.asarray(H_try, dtype=np.float64)
                    R_try = np.asarray(R_try, dtype=np.float64)
                    if v_try.size < 4:
                        break
                    self.P = P_before.copy()
                    self.x = x_before.copy()
                    self.joseph_update(v_try, H_try, R_try)
                    accepted, postfit, _postfit_covariance, bad_row = \
                        self._postfit_result(v_try, H_try, R_try)
                    self.last_tc_update_v = v_try.copy()
                    self.last_tc_update_H = H_try.copy()
                    self.last_tc_update_R = R_try.copy()
                    self.last_tc_update_info = dict(info_try or {})
                    # Rebuild the satellite groups from the new DD identity;
                    # reference selection can change after an exclusion, so
                    # carrying the original row numbers would remove the
                    # wrong satellite on the next GREAT-style iteration.
                    remapped = {}
                    row_indices = list(
                        self.last_tc_update_info.get("row_indices", ()))
                    for pair_order, pair in enumerate(
                            self.last_tc_update_info.get("pairs", ())):
                        row_index = (row_indices[pair_order]
                                     if pair_order < len(row_indices)
                                     else pair_order)
                        if len(pair) < 2:
                            continue
                        for candidate_sat in (pair[1], pair[0]):
                            try:
                                sat_key = int(candidate_sat)
                                row_key = int(row_index)
                            except (TypeError, ValueError):
                                continue
                            remapped.setdefault(sat_key, []).append(row_key)
                    retry_satellites = list(remapped)
                    retry_groups = list(remapped.values())
                    active_rows = list(range(v_try.size))
                    retry_attempted = True
                    if bad_row is not None:
                        self.last_tc_outlier_row = int(bad_row)
                    # Row indices refer to the rebuilt model, so the final
                    # trace intentionally reports satellite exclusions only;
                    # integration diagnostics consume the rebuilt matrices.
                    continue
                proposed = excluded_rows | set(group)
                keep = [index for index in range(v_full.size)
                        if index not in proposed]
                # Keep the same minimum-row contract as _trigger_meas.
                if len(keep) < 4:
                    break
                excluded_rows = proposed
                active_rows = keep
                retry_attempted = True
                keep_index = np.asarray(keep, dtype=int)
                self.P = P_before.copy()
                self.x = x_before.copy()
                R_keep = R_full[np.ix_(keep_index, keep_index)]
                self.joseph_update(v_full[keep_index], H_full[keep_index, :],
                                   R_keep)
                accepted, postfit, _postfit_covariance, bad_row = \
                    self._postfit_result(v_full[keep_index],
                                         H_full[keep_index, :], R_keep)
                if bad_row is not None:
                    self.last_tc_outlier_row = int(active_rows[int(bad_row)])

            if not accepted:
                self.P = P_before
                self.x = x_before
                self.last_tc_postfit = None
                self.last_tc_active_row_indices = tuple()
                return None

            self.last_tc_outlier_indices = tuple(sorted(excluded_rows))
            if retry_builder is not None and retry_attempted:
                # A true rebuild may change the number/order of DD rows when
                # the reference satellite changes.  Keep post-fit values in
                # the final model's own coordinates; the integration layer
                # consumes last_tc_update_{v,H,R} for matching diagnostics.
                self.last_tc_active_row_indices = tuple(
                    range(int(np.asarray(postfit).size)))
                self.last_tc_postfit = np.asarray(
                    postfit, dtype=np.float64).copy()
            else:
                self.last_tc_active_row_indices = tuple(active_rows)
                postfit_full = np.full(v_full.size, np.nan, dtype=np.float64)
                postfit_full[np.asarray(active_rows, dtype=int)] = postfit
                self.last_tc_postfit = postfit_full

        x_post = self.x.copy()
        P_post = self.P.copy() if trace is not None else None
        if trace is not None:
            if postfit is None:
                postfit = v_full - H_full @ x_post
                self.last_tc_postfit = np.asarray(postfit, dtype=np.float64).copy()
                self.last_tc_active_row_indices = tuple(active_rows)
            elif self.last_tc_postfit is not None:
                postfit = self.last_tc_postfit
            self._pending_tc_postfit_trace = {
                "trace": trace,
                "postfit": np.asarray(postfit, dtype=float).copy(),
                "x_post": x_post.copy(),
                "P_post": P_post.copy(),
                "excluded_rows": tuple(self.last_tc_outlier_indices),
            }
        feedback_x = x_post
        if source != "rtk":
            self.last_tc_active_row_indices = tuple(active_rows)
            self.last_tc_postfit = np.asarray(
                v_full - H_full @ x_post, dtype=np.float64).copy()
        self.feedback()
        return feedback_x

    def take_tc_postfit_trace(self, trace=None):
        """Consume the frozen TC post-fit payload for final trace emission."""
        pending = self._pending_tc_postfit_trace
        if pending is None:
            return None
        if trace is not None and pending.get("trace") is not trace:
            return None
        self._pending_tc_postfit_trace = None
        return pending

    def feedback(self) -> None:
        """反馈校正: INS 误差减, GNSS 直接状态累积到 stored。

        GNSS 直接状态 (clk_bias/ambiguity) 的 x 存修正量 ε (correction):
          - stored += ε (累积修正到直接估计)
          - ε 清零 (由父类 feedback 完成)
        INS 误差状态由父类 feedback 处理 (state -= x, x 清零)。
        """
        si = self.si
        # Full immediate closed-loop feedback at this exact GNSS boundary.
        self._feedback_gnss_params_and_base()

    def _feedback_gnss_params_and_base(self) -> None:
        """Accumulate TC direct states and run the common INS feedback."""
        si = self.si
        # 累积 GNSS 直接状态修正到 stored
        if si.clk_bias >= 0:
            self._clk_stored += self.x[si.clk_bias:si.clk_bias + 4]
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
        self._clk_stored = np.zeros(4, dtype=np.float64)
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
