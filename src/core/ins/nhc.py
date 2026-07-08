"""NHC 量测构造 (v 系下, H1/H2 拆分)。

参考 gnss_ins_lc_nhc navstate.cc:343-353。
v 系 = 车体坐标系 (前-右-下), b 系 = IMU 坐标系 (FRD)。
R_b^v 由 IMU 安装角 [pitch, yaw] 构造 (roll 假设 0)。

NHC 约束: 车体侧向/垂向速度为零
  v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b ×] · l_imu^b
  Z_nhc = -v^v[1:3]  (侧向 right + 垂向 down, 期望为 0)

H 矩阵拆分 (双滤波):
  H1 (作用 P1, 15 维): 速度/姿态/陀螺零偏贡献 (ψ-error 正号)
  H2 (作用 P2, 5 维): 安装角/杆臂贡献
"""
import dataclasses
import math

import numpy as np

from src.core.data_types import ImuMeasurement, InsState
from src.core.ins.transfer_matrix import skew


def _build_R_b_v(pitch: float, yaw: float) -> np.ndarray:
    """IMU 安装角 [pitch, yaw] → R_b^v (b→v 旋转矩阵, roll=0)。

    R_b^v = R_z(yaw) · R_y(pitch)  (ZYX 顺序, roll=0)
    参考 gnss_ins_lc_nhc 安装角旋转矩阵构造。
    """
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0.0],
                   [sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]], dtype=np.float64)
    Ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]], dtype=np.float64)
    return Rz @ Ry


class Nhc:
    """NHC 量测构造 (v 系下, H1/H2 拆分)。

    维护 R_b^v (b→v 旋转矩阵), 由 IMU 安装角 [pitch, yaw] 构造。
    每次 P2 反馈后调用 update_from_state 刷新 R_b^v。
    """

    def __init__(self, config: dict):
        ins_cfg = config.get("ins", {}) if config else {}
        self.nhc_std = ins_cfg.get("nhc_std", 0.1)  # m/s
        self.R_b_v = np.eye(3, dtype=np.float64)
        self.imu_angle = np.zeros(2, dtype=np.float64)
        self.imu_leverarm = np.zeros(3, dtype=np.float64)

    def update_from_state(self, state: InsState) -> None:
        """从 InsState 刷新 R_b^v / imu_angle / imu_leverarm。"""
        self.imu_angle = state.imu_angle.copy()
        self.imu_leverarm = state.imu_leverarm.copy()
        self.R_b_v = _build_R_b_v(self.imu_angle[0], self.imu_angle[1])

    def build_meas(self, state: InsState,
                   imu: ImuMeasurement) -> tuple:
        """构造 NHC 量测。

        Returns:
            (Z[2], H1[2,15], H2[2,5], R[2,2])
        """
        self.update_from_state(state)

        C_b_e = state.C_b_e
        C_e_b = C_b_e.T
        v_e = state.vel_e
        w_b_ib = imu.gyro
        l_imu_b = self.imu_leverarm

        # v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b ×] · l_imu^b
        v_v = self.R_b_v @ C_e_b @ v_e + self.R_b_v @ skew(w_b_ib) @ l_imu_b
        # Z = -v_v[1:3]  (侧向 right, 垂向 down; 期望为 0)
        Z = -v_v[1:3].copy()

        # H1 [2, 15]: 速度/姿态/陀螺零偏 (ψ-error 正号)
        # H1[:, VEL(3:6)]   = (R_b^v · C_e^b)[1:3, :]
        # H1[:, ATT(6:9)]   = (R_b^v · C_e^b · [v^e ×])[1:3, :]   (ψ-error +)
        # H1[:, GYRO(9:12)] = (R_b^v · [l_imu^b ×])[1:3, :]
        RbCe = self.R_b_v @ C_e_b
        H1 = np.zeros((2, 15), dtype=np.float64)
        H1[:, 3:6] = RbCe[1:3, :]
        H1[:, 6:9] = (RbCe @ skew(v_e))[1:3, :]
        H1[:, 9:12] = (self.R_b_v @ skew(l_imu_b))[1:3, :]

        # H2 [2, 5]: 安装角/杆臂
        # H2[:, ANGLE(0:2)] = [v^v ×][1:3, 1:3]  (skew(v^v) 第 1,2 列行, 对应 pitch/yaw)
        # H2[:, LEVER(2:5)] = (R_b^v · [ω_eb^b ×])[1:3, :]
        H2 = np.zeros((2, 5), dtype=np.float64)
        v_v_skew = skew(v_v)
        H2[:, 0:2] = v_v_skew[1:3, 1:3]
        H2[:, 2:5] = (self.R_b_v @ skew(w_b_ib))[1:3, :]

        R = np.diag([self.nhc_std ** 2, self.nhc_std ** 2]).astype(np.float64)
        return Z, H1, H2, R

    def feedback(self, x2: np.ndarray, state: InsState) -> InsState:
        """P2 反馈: 安装角 + 杆臂修正。

        Args:
            x2: [δpitch, δyaw, δlx, δly, δlz]
            state: 当前 InsState

        Returns:
            更新后的 InsState (imu_angle, imu_leverarm 修改)
        """
        delta_pitch = float(x2[0])
        delta_yaw = float(x2[1])
        delta_lever = x2[2:5].copy()

        new_imu_angle = np.array([
            self.imu_angle[0] + delta_pitch,
            self.imu_angle[1] + delta_yaw,
        ], dtype=np.float64)
        new_imu_leverarm = self.imu_leverarm - delta_lever

        new_state = dataclasses.replace(state)
        new_state.imu_angle = new_imu_angle
        new_state.imu_leverarm = new_imu_leverarm
        return new_state
