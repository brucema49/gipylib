"""NHC 量测构造 (v 系下, 单 H 矩阵)。

参考 gnss_ins_lc_nhc navstate.cc:343-353。
v 系 = 车体坐标系 (前-右-下), b 系 = IMU 坐标系 (FRD)。
R_b^v 由 IMU 安装角 [pitch, yaw] 构造 (roll 假设 0)。

NHC 约束: 车体侧向/垂向速度为零
  v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b ×] · l_imu^b
  Z_nhc = v^v[1:3]  (侧向 right + 垂向 down, 期望为 0)

H 矩阵 (单滤波, N = si.dim):
  基础 15 维: 速度/姿态/陀螺零偏贡献 (ψ-error 正号)
  可选 imu_angle(2): 安装角贡献
  可选 imu_leverarm(3): IMU 杆臂贡献
"""
import math

import numpy as np

from src.core.data_types import ImuMeasurement, InsState
from src.core.ins.state_index import StateIndex
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
    """NHC 量测构造 (v 系下, 单 H 矩阵)。

    维护 R_b^v (b→v 旋转矩阵), 由 IMU 安装角 [pitch, yaw] 构造。
    每次反馈后调用 update_from_state 刷新 R_b^v。
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
                   imu: ImuMeasurement, si: StateIndex) -> tuple:
        """构造 NHC 量测 (单 H 矩阵, N = si.dim)。

        使用补偿后角速度 w_comp = imu.gyro - gyro_bias (估计真实角速度)。
        陀螺零偏 δb_g = b_true - b_est (ignav 约定), 故 δw_comp = δb_g。

        Returns:
            (Z[2], H[2, N], R[2,2])
        """
        self.update_from_state(state)

        C_b_e = state.C_b_e
        C_e_b = C_b_e.T
        v_e = state.vel_e
        w_b_ib = imu.gyro - state.gyro_bias
        l_imu_b = self.imu_leverarm

        # v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b ×] · l_imu^b
        v_v = self.R_b_v @ C_e_b @ v_e + self.R_b_v @ skew(w_b_ib) @ l_imu_b
        Z = v_v[1:3].copy()

        H = np.zeros((2, si.dim), dtype=np.float64)
        RbCe = self.R_b_v @ C_e_b

        # 基础 15 维: 速度/姿态/陀螺零偏
        H[:, si.vel:si.vel+3] = RbCe[1:3, :]
        H[:, si.att:si.att+3] = (RbCe @ skew(v_e))[1:3, :]
        H[:, si.gyro_bias:si.gyro_bias+3] = -(self.R_b_v @ skew(l_imu_b))[1:3, :]

        # 可选: 安装角 (H2 的 angle 部分)
        if si.has_imu_angle():
            v_v_skew = skew(v_v)
            H[:, si.imu_angle:si.imu_angle+2] = v_v_skew[1:3, 0:2]

        # 可选: IMU 杆臂 (H2 的 lever 部分)
        if si.has_imu_leverarm():
            H[:, si.imu_leverarm:si.imu_leverarm+3] = (self.R_b_v @ skew(w_b_ib))[1:3, :]

        R = np.diag([self.nhc_std ** 2, self.nhc_std ** 2]).astype(np.float64)
        return Z, H, R
