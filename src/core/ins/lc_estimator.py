"""松组合 EKF 估计器 (P1 主滤波 + P2 NHC 子滤波)。

参考:
- gnss_ins_lc_nhc navfilter.cc (TimeUpdate/MeasureUpdate/ReviseState)
- ignav ins-gnss.cc (H 矩阵 + 序贯 Joseph form)
- GINav ins_time_updata.m (中间值法 P 传播)

P1: 15 维 E 系 [δr^e, δv^e, δψ^e, δb_g, δb_a] (ψ-error)
P2: 5 维 v 系 [δθ_imu(2), δl_imu(3)] (NHC 子滤波)
"""
import dataclasses
import logging
import math

import numpy as np

from src.core.data_types import GnssSolution, ImuMeasurement, InsState
from src.core.ins.attitude import dcm2euler, dcm2quat
from src.core.ins.earth_param import cal_Ce2n, ecef2llh
from src.core.ins.ins_update import InsUpdate
from src.core.ins.nhc import Nhc
from src.core.ins.transfer_matrix import TransferMatrix, skew

logger = logging.getLogger(__name__)


class LcEstimator:
    """松组合 EKF 估计器 (双滤波 P1 + P2)。

    P1 主滤波: 15 维 E 系, 复用 TransferMatrix (F/Φ/Q, ψ-error)
    P2 NHC 子滤波: 5 维 v 系, F2=0 (常数过程)
    """

    def __init__(self, state: InsState, P1: np.ndarray, P2: np.ndarray,
                 config: dict):
        self.ins_update = InsUpdate(state)
        self.tm = TransferMatrix(config)
        self._nhc = Nhc(config)
        self.P1 = P1.copy().astype(np.float64)
        self.P2 = P2.copy().astype(np.float64)
        self.x1 = np.zeros(15, dtype=np.float64)
        self.x2 = np.zeros(5, dtype=np.float64)

        ins_cfg = config.get("ins", {})
        self.static_speed_threshold = ins_cfg.get("static_speed_threshold", 0.5)
        self.angular_velocity_threshold = ins_cfg.get(
            "angular_velocity_threshold", 30.0 * math.pi / 180.0)
        self.zupt_std = ins_cfg.get("zupt_std", 0.05)
        self._gnss_pos_std = {
            1: 10.0,   # SPP
            2: 1.0,    # RTD
            4: 1.0,    # DGPS
            5: 0.02,   # RTK fix
            0: 0.5,    # RTK float / unknown
        }
        self._gnss_vel_std = ins_cfg.get("gnss_vel_std", 0.5)

    @property
    def state(self) -> InsState:
        return self.ins_update.state

    @property
    def nhc(self) -> Nhc:
        return self._nhc

    # ===== 时间更新 =====

    def time_update(self, imu: ImuMeasurement) -> None:
        """IMU 机械编排 + P1/P2 协方差传播。

        P1: Φ1·(P1+0.5Q1)·Φ1^T + 0.5Q1 (GINav 中间值法)
        P2: P2 + Q2·dt (Φ2=I, F2=0)
        """
        # 捕获 prev_timestamp (update 会覆盖它)
        prev_ts = self.ins_update._prev_timestamp
        # 机械编排 (更新 state, f_b, w_b_ib, _prev_timestamp)
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
        Q1 = self.tm.build_Q(dt, C_b_e)
        P0 = self.P1 + 0.5 * Q1
        self.P1 = Phi @ P0 @ Phi.T + 0.5 * Q1
        self.P1 = 0.5 * (self.P1 + self.P1.T)

        Q2 = self._build_Q2(dt)
        self.P2 = self.P2 + Q2
        self.P2 = 0.5 * (self.P2 + self.P2.T)

    def _build_Q2(self, dt: float) -> np.ndarray:
        """P2 过程噪声 (5x5, 小量随机游走)。"""
        sigma_angle = 1e-3
        sigma_lever = 1e-4
        q = np.array([sigma_angle ** 2, sigma_angle ** 2,
                      sigma_lever ** 2, sigma_lever ** 2, sigma_lever ** 2])
        return np.diag(q * dt).astype(np.float64)

    # ===== 量测更新 =====

    def meas_update_pos(self, gnss: GnssSolution) -> None:
        """GNSS 位置量测更新 (仅 P1, 3 维, Joseph form)。"""
        state = self.ins_update.state
        Z = state.pos_e - gnss.position
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 0:3] = np.eye(3)
        if gnss.sd is not None and np.all(gnss.sd > 0):
            R = np.diag(gnss.sd ** 2).astype(np.float64)
        else:
            sigma = self._gnss_pos_std.get(gnss.quality, 0.5)
            R = np.diag([sigma ** 2] * 3).astype(np.float64)
        self._joseph_update_P1(Z, H, R)

    def meas_update_vel(self, gnss: GnssSolution) -> None:
        """GNSS 速度量测更新 (仅 P1, 3 维, Joseph form)。"""
        if gnss.velocity is None:
            return
        state = self.ins_update.state
        Z = state.vel_e - gnss.velocity
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 3:6] = np.eye(3)
        if np.linalg.norm(state.leverarm) > 1e-9:
            H[:, 6:9] = -skew(state.C_b_e @ state.leverarm)
        if gnss.vel_sd is not None and np.all(gnss.vel_sd > 0):
            R = np.diag(gnss.vel_sd ** 2).astype(np.float64)
        else:
            R = np.diag([self._gnss_vel_std ** 2] * 3).astype(np.float64)
        self._joseph_update_P1(Z, H, R)

    def meas_update_zupt(self) -> None:
        """ZUPT 量测更新 (仅 P1, 3 维速度约束)。"""
        state = self.ins_update.state
        Z = state.vel_e.copy()
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 3:6] = np.eye(3)
        R = np.diag([self.zupt_std ** 2] * 3).astype(np.float64)
        self._joseph_update_P1(Z, H, R)

    def meas_update_nhc(self, imu: ImuMeasurement) -> None:
        """NHC 量测更新 (P1 的 H1 部分 + P2 的 H2 部分, 2 维)。"""
        Z, H1, H2, R_nhc = self._nhc.build_meas(self.ins_update.state, imu)
        self._joseph_update_P1(Z, H1, R_nhc)
        self._joseph_update_P2(Z, H2, R_nhc)

    def _joseph_update_P1(self, Z: np.ndarray, H: np.ndarray,
                          R: np.ndarray) -> None:
        """P1 Joseph form 量测更新。"""
        S = H @ self.P1 @ H.T + R
        K = self.P1 @ H.T @ np.linalg.inv(S)
        innov = Z - H @ self.x1
        self.x1 = self.x1 + K @ innov
        I_KH = np.eye(15) - K @ H
        self.P1 = I_KH @ self.P1 @ I_KH.T + K @ R @ K.T
        self.P1 = 0.5 * (self.P1 + self.P1.T)

    def _joseph_update_P2(self, Z: np.ndarray, H: np.ndarray,
                          R: np.ndarray) -> None:
        """P2 Joseph form 量测更新。"""
        S = H @ self.P2 @ H.T + R
        K = self.P2 @ H.T @ np.linalg.inv(S)
        innov = Z - H @ self.x2
        self.x2 = self.x2 + K @ innov
        I_KH = np.eye(5) - K @ H
        self.P2 = I_KH @ self.P2 @ I_KH.T + K @ R @ K.T
        self.P2 = 0.5 * (self.P2 + self.P2.T)

    # ===== 反馈 =====

    def feedback(self) -> None:
        """双滤波独立反馈校正。P1: pos/vel/att(ψ+)/bias; P2: 安装角+杆臂。"""
        self._feedback_P1()
        self._feedback_P2()
        self.x1[:] = 0.0
        self.x2[:] = 0.0

    def _feedback_P1(self) -> None:
        """P1 反馈 (ψ-error: 姿态加号)。"""
        state = self.ins_update.state
        delta_pos = self.x1[0:3]
        delta_vel = self.x1[3:6]
        delta_psi = self.x1[6:9]
        delta_bg = self.x1[9:12]
        delta_ba = self.x1[12:15]

        new_pos = state.pos_e - delta_pos
        new_vel = state.vel_e - delta_vel
        C_b_e_new = (np.eye(3) + skew(delta_psi)) @ state.C_b_e
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
        new_state.gyro_bias = state.gyro_bias - delta_bg
        new_state.accel_bias = state.accel_bias - delta_ba
        self.ins_update.state = new_state

    def _feedback_P2(self) -> None:
        """P2 反馈: 安装角 + 杆臂。"""
        new_state = self._nhc.feedback(self.x2, self.ins_update.state)
        self.ins_update.state = new_state
        self._nhc.update_from_state(new_state)
