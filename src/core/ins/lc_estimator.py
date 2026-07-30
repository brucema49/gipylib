"""松组合 EKF 估计器 (单滤波, StateIndex 参数块管理)。

参考:
- gnss_ins_lc_nhc navfilter.cc (TimeUpdate/MeasureUpdate/ReviseState)
- ignav ins-gnss.cc (H 矩阵 + 序贯 Joseph form)
- GINav ins_time_updata.m (中间值法 P 传播)

状态向量 (N = si.dim):
  固定 15 维: [δr^e, δv^e, δψ^e, δb_g, δb_a] (ψ-error, E 系)
  可选: lever_arm(3), imu_angle(2), imu_leverarm(3), time_sync(1)
"""
import dataclasses
import logging

import numpy as np

from src.core.data_types import GnssSolution, ImuMeasurement, InsState
from src.core.ins.attitude import dcm2euler, dcm2quat
from src.core.ins.earth_param import cal_Ce2n, ecef2llh
from src.core.ins.ins_update import InsUpdate
from src.core.ins.state_index import StateIndex
from src.core.ins.transfer_matrix import TransferMatrix, rodrigues, skew

logger = logging.getLogger(__name__)


class LcEstimator:
    """松组合 EKF 估计器 (单滤波, StateIndex 参数块管理)。

    参考 gnss_ins_lc_nhc (单滤波 + StateIndex) 和 GREAT-MSF (block 矩阵操作)。
    算法参考 ignav (H 矩阵公式, ψ-error 模型)。
    """

    def __init__(self, state: InsState, P: np.ndarray, config: dict):
        self.ins_update = InsUpdate(state)
        self.si = StateIndex.from_config(config)
        self.tm = TransferMatrix(config, self.si)
        self.P = P.copy().astype(np.float64)
        self.x = np.zeros(self.si.dim, dtype=np.float64)

        ins_cfg = config.get("ins", {})
        self._gnss_pos_std = {
            1: 0.15,   # SOLQ_FIX (RTK fix): 增大以给 EKF 惯性, 平滑 RTK 偶发跳变
            2: 0.05,   # SOLQ_FLOAT (RTK float)
            4: 1.0,    # SOLQ_DGPS
            5: 10.0,   # SOLQ_SINGLE (SPP)
            0: 10.0,   # SOLQ_NONE (fallback)
        }
        self._gnss_vel_std = ins_cfg.get("gnss_vel_std", 0.5)
        self._innov_reject_threshold = float(ins_cfg.get("innov_reject_threshold", 0.0))
        self._innov_reject_warmup = int(ins_cfg.get("innov_reject_warmup", 100))
        self._gnss_update_count = 0

    @property
    def state(self) -> InsState:
        return self.ins_update.state

    # ===== 时间更新 =====

    def time_update(self, imu: ImuMeasurement) -> None:
        """IMU 机械编排 + P 协方差传播。

        P: Φ·(P+0.5Q)·Φ^T + 0.5Q (GINav 中间值法)
        """
        prev_ts = self.ins_update._prev_timestamp
        self.ins_update.update(imu)

        dt = imu.timestamp - prev_ts
        if dt <= 0.0:
            return

        C_b_e = self.ins_update.state.C_b_e
        f_b = self.ins_update.f_b
        w_b_ib = self.ins_update.w_b_ib
        pos_e = self.ins_update.state.pos_e

        F = self.tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        Phi = self.tm.build_Phi(F, dt)
        Q = self.tm.build_Q(dt, C_b_e)
        P0 = self.P + 0.5 * Q
        self.P = Phi @ P0 @ Phi.T + 0.5 * Q
        self.P = 0.5 * (self.P + self.P.T)

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

        if si.has_lever_arm():
            H[:, si.lever_arm:si.lever_arm+3] = -state.C_b_e

        if si.has_time_sync():
            lever = state.leverarm
            dt1 = state.C_b_e @ skew(self.ins_update.w_b_ib) @ lever + state.vel_e
            H[:, si.time_sync] = dt1

        R = self._build_pos_R(gnss)
        self.joseph_update(Z, H, R)

    def _build_pos_R(self, gnss: GnssSolution) -> np.ndarray:
        """构造位置量测噪声协方差 (自适应 sigma + 全协方差矩阵)。

        对角线: max(fixed_sigma, rtk_sd) — 良好历元 rtk_sd 很小, sigma 取固定值;
        偏差历元 rtk_sd 增大, sigma 随之增大以降低 K。
        非对角线: 从 RTK 全协方差矩阵 gnss.cov 提取, 捕获轴向相关性,
        使 EKF 能针对性降低偏差方向上的 Kalman 增益。

        time_sync 未估计时, R 中加入时间偏差不确定性 (speed × dt_offset)。
        time_sync 估计时, 时间偏差由状态处理, R 不含 timing 项。
        """
        base_sigma = self._gnss_pos_std.get(gnss.quality, 0.5)
        sigma = np.array([base_sigma] * 3, dtype=np.float64)

        if gnss.sd is not None and np.all(gnss.sd > 0):
            sigma = np.maximum(sigma, gnss.sd)

        R = np.diag(sigma ** 2).astype(np.float64)

        if gnss.cov is not None:
            cov_off_diag = gnss.cov - np.diag(np.diag(gnss.cov))
            R = R + cov_off_diag

        if not self.si.has_time_sync():
            speed = float(np.linalg.norm(self.ins_update.state.vel_e))
            dt_offset = 0.005
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
        self.joseph_update(Z, H, R)

    def _build_vel_R(self, gnss: GnssSolution) -> np.ndarray:
        """构造速度量测噪声协方差。"""
        if gnss.vel_sd is not None and np.all(gnss.vel_sd > 0):
            R = np.diag(gnss.vel_sd ** 2).astype(np.float64)
        else:
            R = np.diag([self._gnss_vel_std ** 2] * 3).astype(np.float64)
        return 0.5 * (R + R.T)

    def joseph_update(self, Z: np.ndarray, H: np.ndarray,
                      R: np.ndarray, innov_clip: float = 0.0) -> None:
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

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        I_KH = np.eye(n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    # ===== 反馈 =====

    def feedback(self) -> None:
        """统一反馈校正 (ψ-error + 可选参数块)。"""
        state = self.ins_update.state
        si = self.si

        # 基础 15 维 (ψ-error, 对齐 ignav lcclp)
        delta_pos = self.x[si.pos:si.pos+3]
        delta_vel = self.x[si.vel:si.vel+3]
        delta_psi = self.x[si.att:si.att+3]
        delta_bg = self.x[si.gyro_bias:si.gyro_bias+3]
        delta_ba = self.x[si.accel_bias:si.accel_bias+3]

        new_pos = state.pos_e - delta_pos
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
