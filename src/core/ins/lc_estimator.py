"""松组合 EKF 估计器 (单滤波, StateIndex 参数块管理)。

参考:
- ignav ins-gnss.cc (H 矩阵 + 序贯 Joseph form)
- GINav ins_time_updata.m (中间值法 P 传播)

状态向量 (N = si.dim):
  固定 15 维: [δr^e, δv^e, δψ^e, δb_g, δb_a] (ψ-error, E 系)
  可选: lever_arm(3), imu_angle(2), imu_leverarm(3), time_sync(1)
"""
import dataclasses
import logging
from typing import Optional

import numpy as np

from src.core.data_types import GnssSolution, ImuMeasurement, InsState
from src.core.ins.attitude import dcm2euler, dcm2quat
from src.core.ins.earth_param import cal_Ce2n, ecef2llh
from src.core.ins.ins_update import InsUpdate
from src.core.ins.state_index import StateIndex
from src.core.ins.transfer_matrix import TransferMatrix, rodrigues, skew

logger = logging.getLogger(__name__)


def _config_switch(value, name: str) -> bool:
    """Parse an on/off configuration value, accepting YAML bools and strings."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    text = str(value).strip().lower()
    if text in {"on", "true", "yes", "1"}:
        return True
    if text in {"off", "false", "no", "0"}:
        return False
    raise ValueError(
        f"{name} must be on/off or true/false, got {value!r}"
    )


class LcEstimator:
    """松组合 EKF 估计器 (单滤波, StateIndex 参数块管理)。

    参考 GREAT-MSF (block 矩阵操作)。
    算法参考 ignav (H 矩阵公式, ψ-error 模型)。
    """

    def __init__(self, state: InsState, P: np.ndarray, config: dict):
        self.ins_update = InsUpdate(state)
        self.si = StateIndex.from_config(config)
        self.tm = TransferMatrix(config, self.si)
        self.P = P.copy().astype(np.float64)
        self.x = np.zeros(self.si.dim, dtype=np.float64)
        # 可选矩阵诊断：保留最近一次时间更新、量测更新以及闭环反馈增量。
        # 这些值只用于离线对比 KF-GINS，不参与滤波计算。
        self.last_time_update_diag = None
        self.last_meas_update_diag = None
        self.meas_update_diags = []
        self.last_feedback_x = np.zeros(self.si.dim, dtype=np.float64)

        ins_cfg = config.get("ins", {})
        self._gnss_pos_std = {
            1: 0.3,    # SOLQ_FIX (RTK fix): RTK 平面精度~0.3m, 增大给 EKF 惯性平滑偶发跳变
            2: 0.3,    # SOLQ_FLOAT (RTK float): RTK FLOAT 平面精度~0.22m, sigma=0.3 给 EKF 适当惯性
            4: 1.0,    # SOLQ_DGPS
            5: 1.0,    # SOLQ_SINGLE (SPP): SPP 平面精度~0.8m, sigma=1.0 信任平面 GNSS
            0: 1.0,    # SOLQ_NONE (fallback)
        }
        # RTK (Q=1/2) 不使用 rtklib 报的 sd (偏大保守, FLOAT sd 可达 2-9m, 实际精度 0.22m);
        # SPP (Q=5) sd 较可靠, 用 max(base_sigma, sd)。
        self._use_gnss_sd_qualities = {4, 5}  # DGPS / SPP 使用 max(sigma, sd)
        self._use_reported_gnss_sd = bool(ins_cfg.get("use_reported_gnss_sd", False))
        self._gnss_sd_scale = float(ins_cfg.get("gnss_sd_scale", 1.0))
        if self._gnss_sd_scale <= 0.0:
            raise ValueError("ins.gnss_sd_scale must be positive")
        self._gnss_sd_axis_scale = np.asarray(
            ins_cfg.get("gnss_sd_axis_scale", [1.0, 1.0, 1.0]),
            dtype=np.float64)
        if self._gnss_sd_axis_scale.shape != (3,) \
                or np.any(self._gnss_sd_axis_scale <= 0.0):
            raise ValueError("ins.gnss_sd_axis_scale must contain three positive values")
        # 垂直 sigma 放大因子: SPP 高程有系统性偏差(~3m), 需放大高程 sigma 让 EKF 不信任
        # RTK 高程精度好(~0.3m), 不需放大。因子作用于 NED 的 Down 分量
        self._vertical_sigma_factor = float(ins_cfg.get("vertical_sigma_factor", 1.0))
        self._gnss_time_sync_noise_s = float(ins_cfg.get("gnss_time_sync_noise_s", 0.005))
        self._gnss_vel_std = ins_cfg.get("gnss_vel_std", 0.5)
        feedback_switch = ins_cfg.get("feedback_pos_enable")
        if feedback_switch is None:
            # Preserve the pre-switch behavior for legacy configurations that
            # already declare a feedback ratio.  A completely unspecified
            # configuration keeps the ignav-style immediate feedback.
            self._feedback_pos_enabled = "feedback_pos_fraction" in ins_cfg
        else:
            self._feedback_pos_enabled = _config_switch(
                feedback_switch,
                "ins.feedback_pos_enable",
            )
        self._feedback_pos_fraction = float(
            ins_cfg.get("feedback_pos_fraction", 1.0)
        ) if self._feedback_pos_enabled else 1.0
        if not 0.0 < self._feedback_pos_fraction <= 1.0:
            raise ValueError("ins.feedback_pos_fraction must be in (0, 1]")
        self._innov_reject_threshold = float(ins_cfg.get("innov_reject_threshold", 0.0))
        self._innov_reject_warmup = int(ins_cfg.get("innov_reject_warmup", 100))
        self._gnss_update_count = 0
        # 历史 GNSS 位置 (用于位置差分计算速度, 当 GNSS 无速度输出时)
        self._prev_gnss_pos: Optional[np.ndarray] = None
        self._prev_gnss_ts: Optional[float] = None

    @property
    def state(self) -> InsState:
        return self.ins_update.state

    # ===== 时间更新 =====

    def time_update(self, imu: ImuMeasurement) -> None:
        """IMU 机械编排 + P 协方差传播。

        P: Φ·(P+0.5Q)·Φ^T + 0.5Q (GINav 中间值法)
        """
        prev_ts = self.ins_update._prev_timestamp
        # KF-GINS forms F/G from the state and attitude at the beginning of
        # the interval (pvapre), while the nominal mechanization then advances
        # to the current IMU epoch.  Keep that same linearization point.
        prev_C_b_e = self.ins_update.state.C_b_e.copy()
        prev_pos_e = self.ins_update.state.pos_e.copy()
        self.ins_update.update(imu)

        if not self.ins_update.last_update_accepted:
            return

        dt = imu.timestamp - prev_ts
        if dt <= 0.0:
            return

        P_before = self.P.copy()
        C_b_e = prev_C_b_e
        f_b = self.ins_update.f_b
        w_b_ib = self.ins_update.w_b_ib
        pos_e = prev_pos_e

        F = self.tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        Phi = self.tm.build_Phi(F, dt)
        Q = self.tm.build_Q(dt, C_b_e)
        P0 = self.P + 0.5 * Q
        Q_effective = 0.5 * (Phi @ Q @ Phi.T + Q)
        self.P = Phi @ P0 @ Phi.T + 0.5 * Q
        # Normally x is zero after closed-loop feedback.  A configured
        # partial position feedback retains a mean error; propagate it with
        # the same transition matrix as the covariance.
        if np.any(self.x):
            self.x = Phi @ self.x
        self.P = 0.5 * (self.P + self.P.T)
        self.last_time_update_diag = {
            "timestamp": float(imu.timestamp),
            "dt": float(dt),
            "F": F.copy(),
            "Phi": Phi.copy(),
            # Q is the discrete noise actually used by the covariance
            # propagation; Q_base is the pre-symmetrization GQG^T term.
            "Q": Q_effective,
            "Q_base": Q.copy(),
            "P_before": P_before,
            "P_after": self.P.copy(),
        }

    # ===== 量测更新 =====

    def meas_update_pos(self, gnss: GnssSolution) -> None:
        """GNSS 位置量测更新 (3 维, Joseph form)。

        H 矩阵参考 ignav build_HVR:
          pos: I(3)
          lever_arm: -C_b_e (jacobian: H[ila,pos]=-Cbe)
          time_sync: C_b_e @ skew(w_b_ib) @ lever + v_e (jacobian_p_dt)

        当位置创新范数超过 innov_reject_threshold 时跳过量测更新,
        防止 INS 跟随 GNSS 系统偏差; 速度更新不受影响。
        """
        state = self.ins_update.state
        si = self.si
        lever_e = state.C_b_e @ state.leverarm
        fixed_lever = (np.linalg.norm(state.leverarm) > 1e-12
                        and not si.has_lever_arm())
        if fixed_lever:
            Z = state.pos_e + lever_e - gnss.position
        else:
            Z = state.pos_e - gnss.position

        self._gnss_update_count += 1
        if self._innov_reject_threshold > 0 and self._gnss_update_count > self._innov_reject_warmup:
            innov_norm = float(np.linalg.norm(Z - self.x[si.pos:si.pos+3]))
            if innov_norm > self._innov_reject_threshold:
                logger.debug(
                    "pos update rejected: innov=%.3fm > %.3fm, epoch=%d, Q=%d, ns=%d",
                    innov_norm, self._innov_reject_threshold,
                    self._gnss_update_count, gnss.quality, gnss.num_sv
                )
                return

        H = np.zeros((3, si.dim), dtype=np.float64)
        H[:, si.pos:si.pos+3] = np.eye(3)

        if fixed_lever:
            # The project attitude error is the negative of KF-GINS' phi
            # state (both use left multiplication, with opposite feedback
            # sign).  Keep the GNSS lever-arm Jacobian in that convention.
            H[:, si.att:si.att+3] = -skew(lever_e)

        if si.has_lever_arm():
            H[:, si.lever_arm:si.lever_arm+3] = -state.C_b_e

        if si.has_time_sync():
            lever = state.leverarm
            dt1 = state.C_b_e @ skew(self.ins_update.w_b_ib) @ lever + state.vel_e
            H[:, si.time_sync] = dt1

        R = self._build_pos_R(gnss)
        self.joseph_update(Z, H, R, update_kind="position",
                           update_timestamp=gnss.timestamp)

    def _build_pos_R(self, gnss: GnssSolution) -> np.ndarray:
        """构造位置量测噪声协方差 (自适应 sigma + 全协方差矩阵 + 各向异性高程)。

        对角线: max(fixed_sigma, rtk_sd) — 良好历元 rtk_sd 很小, sigma 取固定值;
        偏差历元 rtk_sd 增大, sigma 随之增大以降低 K。
        非对角线: 从 RTK 全协方差矩阵 gnss.cov 提取, 捕获轴向相关性,
        使 EKF 能针对性降低偏差方向上的 Kalman 增益。

        各向异性高程: SPP 高程有系统性偏差(~3m), 需放大 Down 方向 sigma 让 EKF
        不信任高程, 依赖 INS 加速度计积分。通过 NED 旋转实现:
          R_ned = C_e^n @ R_ecef @ (C_e^n)^T
          R_ned[2,2] *= vertical_sigma_factor²  (Down 分量放大)
          R_ecef = (C_e^n)^T @ R_ned @ C_e^n

        time_sync 未估计时, R 中加入时间偏差不确定性 (speed × dt_offset)。
        time_sync 估计时, 时间偏差由状态处理, R 不含 timing 项。
        """
        base_sigma = self._gnss_pos_std.get(gnss.quality, 0.5)
        sigma = np.array([base_sigma] * 3, dtype=np.float64)

        # 仅对 DGPS/SPP 使用 max(sigma, gnss.sd): 这些模式 sd 较可靠;
        # RTK (Q=1/2) 的 rtklib sd 偏大保守 (FLOAT sd 可达 2-9m, 实际 0.22m),
        # 用 base_sigma 即可, 避免 K 过低致 INS 自由漂移。
        if (self._use_reported_gnss_sd and gnss.sd is not None
                and np.all(gnss.sd > 0)):
            sigma = gnss.sd * self._gnss_sd_scale * self._gnss_sd_axis_scale
        elif (gnss.quality in self._use_gnss_sd_qualities
                and gnss.sd is not None and np.all(gnss.sd > 0)):
            sigma = np.maximum(sigma, gnss.sd)

        R = np.diag(sigma ** 2).astype(np.float64)

        if gnss.cov is not None:
            cov_off_diag = gnss.cov - np.diag(np.diag(gnss.cov))
            axis = self._gnss_sd_scale * self._gnss_sd_axis_scale
            R = R + (axis[:, None] * cov_off_diag * axis[None, :])

        # 各向异性高程: 在 NED 系放大 Down 分量 (SPP 高程偏差大)
        if self._vertical_sigma_factor != 1.0:
            lat, lon, _ = ecef2llh(self.ins_update.state.pos_e)
            C_e_n = cal_Ce2n(lat, lon)  # ECEF → NED
            R_ned = C_e_n @ R @ C_e_n.T
            R_ned[2, 2] *= self._vertical_sigma_factor ** 2
            R = C_e_n.T @ R_ned @ C_e_n

        if not self.si.has_time_sync() and self._gnss_time_sync_noise_s > 0.0:
            speed = float(np.linalg.norm(self.ins_update.state.vel_e))
            dt_offset = self._gnss_time_sync_noise_s
            sigma_timing = speed * dt_offset
            R = R + (sigma_timing ** 2) * np.eye(3) / 3.0

        return 0.5 * (R + R.T)

    def meas_update_vel(self, gnss: GnssSolution) -> None:
        """GNSS 速度量测更新 (3 维, Joseph form)。

        H 矩阵参考 ignav build_HVR:
          vel: I(3)
          att: -skew(C_b_e @ lever) (杆臂姿态贡献)
          lever_arm: skew(w_ie_e) @ C_b_e - C_b_e @ skew(w_b_ib) (jacobian_v_dla)
          time_sync: C_b_e @ skew(w_b_ib)² @ lever + a_e (jacobian_v_dt)
        """
        if gnss.velocity is None:
            return
        state = self.ins_update.state
        si = self.si
        Z = state.vel_e - gnss.velocity
        H = np.zeros((3, si.dim), dtype=np.float64)
        H[:, si.vel:si.vel+3] = np.eye(3)

        if np.linalg.norm(state.leverarm) > 1e-9:
            H[:, si.att:si.att+3] = -skew(state.C_b_e @ state.leverarm)

        if si.has_lever_arm():
            dla = skew(self.tm.w_ie_e) @ state.C_b_e \
                - state.C_b_e @ skew(self.ins_update.w_b_ib)
            H[:, si.lever_arm:si.lever_arm+3] = dla

        if si.has_time_sync():
            lever = state.leverarm
            w_skew = skew(self.ins_update.w_b_ib)
            dt2 = state.C_b_e @ w_skew @ w_skew @ lever + self.ins_update.a_e
            H[:, si.time_sync] = dt2

        R = self._build_vel_R(gnss)
        self.joseph_update(Z, H, R, update_kind="velocity",
                           update_timestamp=gnss.timestamp)

    def _build_vel_R(self, gnss: GnssSolution) -> np.ndarray:
        """构造速度量测噪声协方差。

        RTK relpos 的 vel_sd 可能极大 (如 20+ m/s), 导致 K≈0 速度更新无效。
        对此类情况 cap 到 _gnss_vel_std, 使 EKF 能有效利用速度量测。
        """
        if gnss.vel_sd is not None and np.all(gnss.vel_sd > 0):
            vel_sd = np.minimum(gnss.vel_sd, self._gnss_vel_std)
            R = np.diag(vel_sd ** 2).astype(np.float64)
        else:
            R = np.diag([self._gnss_vel_std ** 2] * 3).astype(np.float64)
        return 0.5 * (R + R.T)

    def joseph_update(self, Z: np.ndarray, H: np.ndarray,
                      R: np.ndarray, innov_clip: float = 0.0,
                      update_kind: str = "generic",
                      update_timestamp: Optional[float] = None) -> None:
        """Joseph form 量测更新 (单滤波, N = si.dim)。

        对应 ignav filter(): 供 GNSS 量测更新和 Constraints 模块调用。

        innov_clip > 0 时启用创新自适应降权: 当创新范数超过 innov_clip
        时, 按比例放大 R 以降低 Kalman 增益, 防止滤波器跟随持续偏差。
        """
        n = self.si.dim
        innov = Z - H @ self.x

        if innov_clip > 0:
            innov_norm = float(np.linalg.norm(innov))
            if innov_norm > innov_clip:
                scale = (innov_norm / innov_clip) ** 2
                R = R * scale

        P_before = self.P.copy()
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        I_KH = np.eye(n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        diag = {
            "kind": str(update_kind),
            "timestamp": (float(update_timestamp)
                          if update_timestamp is not None else None),
            "H": H.copy(),
            "R": R.copy(),
            "S": S.copy(),
            "K": K.copy(),
            "innovation": innov.copy(),
            "Z": np.asarray(Z, dtype=np.float64).copy(),
            "P_before": P_before,
            "P_after": self.P.copy(),
        }
        self.last_meas_update_diag = diag
        self.meas_update_diags.append(diag)
        # 仅保留最近的少量更新，避免长时间运行时无限增长。
        if len(self.meas_update_diags) > 8:
            del self.meas_update_diags[:-8]

    # ===== 反馈 =====

    def feedback(self) -> None:
        """统一反馈校正 (ψ-error + 可选参数块)。"""
        # 闭环反馈前保存完整误差向量，供 458699 等历元的离线对比使用。
        self.last_feedback_x = self.x.copy()
        state = self.ins_update.state
        si = self.si

        # 基础 15 维 (ψ-error, 对齐 ignav lcclp)
        delta_pos = self.x[si.pos:si.pos+3].copy()
        delta_vel = self.x[si.vel:si.vel+3]
        delta_psi = self.x[si.att:si.att+3]
        delta_bg = self.x[si.gyro_bias:si.gyro_bias+3]
        delta_ba = self.x[si.accel_bias:si.accel_bias+3]
        delta_gs = (self.x[si.gyro_scale:si.gyro_scale+3]
                    if si.has_imu_scale() else np.zeros(3))
        delta_as = (self.x[si.accel_scale:si.accel_scale+3]
                    if si.has_imu_scale() else np.zeros(3))

        applied_pos = self._feedback_pos_fraction * delta_pos
        new_pos = state.pos_e - applied_pos
        new_vel = state.vel_e - delta_vel
        C_b_e_new = rodrigues(-delta_psi) @ state.C_b_e
        U, _, Vt = np.linalg.svd(C_b_e_new)
        C_b_e_new = U @ Vt
        lat, lon, _ = ecef2llh(new_pos)
        C_e_n = cal_Ce2n(lat, lon)
        C_b_n_new = C_e_n @ C_b_e_new
        att_rpy = dcm2euler(C_b_n_new)

        new_state = dataclasses.replace(state)
        new_state.pos_e = new_pos
        new_state.vel_e = new_vel
        new_state.C_b_e = C_b_e_new
        new_state.q_b_e = dcm2quat(C_b_e_new)
        new_state.att_rpy = att_rpy
        new_state.gyro_bias = state.gyro_bias + delta_bg
        new_state.accel_bias = state.accel_bias + delta_ba
        new_state.gyro_scale = state.gyro_scale + delta_gs
        new_state.accel_scale = state.accel_scale + delta_as

        # 可选: GNSS 杆臂 (δlever = lever_true - lever_est, 反馈加)
        if si.has_lever_arm():
            new_state.leverarm = state.leverarm + self.x[si.lever_arm:si.lever_arm+3]

        # 可选: 安装角 (δangle = angle_true - angle_est, 反馈加)
        if si.has_imu_angle():
            new_state.imu_angle = state.imu_angle + self.x[si.imu_angle:si.imu_angle+2]

        # 可选: IMU 杆臂 (δlever = lever_est - lever_true, 反馈减)
        if si.has_imu_leverarm():
            new_state.imu_leverarm = state.imu_leverarm - self.x[si.imu_leverarm:si.imu_leverarm+3]

        # 可选: 时间对齐 (δt = t_true - t_est, 反馈加)
        if si.has_time_sync():
            new_state.time_sync = state.time_sync + float(self.x[si.time_sync])

        self.ins_update.state = new_state
        self.x[:] = 0.0
        remaining_pos = (1.0 - self._feedback_pos_fraction) * delta_pos
        if np.any(remaining_pos):
            self.x[si.pos:si.pos+3] = remaining_pos
