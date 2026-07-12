"""NHC/ZUPT/ZARU 约束量测更新 (独立模块, 参考 ignav 分离架构)。

参考:
- ignav ins-nhc.cc (bldnhc + nhc): NHC 量测构造 + 滤波
- ignav ins-zvu.cc  (detstc + zvu): 静态检测 + 零速更新
- ignav ins-zaru.cc (zaru):         零角速率更新

架构 (参考 ignav 分离模式):
  本模块 = 约束逻辑 (guards + Z/H/R 构造 + filter 调用)
  LcEstimator.joseph_update = 滤波步骤 (对应 ignav filter())
  LcEstimator.feedback      = 反馈校正 (对应 ignav clp())

约束互斥 (参考 ignav postpos.cc):
  静态 → ZUPT + ZARU
  运动 → NHC
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

    对应 ignav bldnhc: 构造 Z/H/R, 不含 guards 和滤波。
    v 系 = 车体坐标系 (前-右-下), b 系 = IMU 坐标系 (FRD)。
    R_b^v 由 IMU 安装角 [pitch, yaw] 构造 (roll 假设 0)。

    NHC 约束: 车体侧向/垂向速度为零
      v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b ×] · l_imu^b
      Z_nhc = v^v[1:3]  (侧向 right + 垂向 down, 期望为 0)
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


class Constraints:
    """NHC/ZUPT/ZARU 约束更新器 (独立于 LcEstimator)。

    参考 ignav ins-nhc.cc / ins-zvu.cc / ins-zaru.cc 的分离架构:
    - 约束逻辑 (guards + Z/H/R 构造) 在本模块
    - 滤波步骤 (Joseph update) 由 LcEstimator.joseph_update 提供
    - 反馈校正 (close-loop) 由 LcEstimator.feedback 提供

    用法:
        constraints = Constraints(config)
        if constraints.nhc(estimator, imu):
            estimator.feedback()
    """

    def __init__(self, config: dict):
        ins_cfg = config.get("ins", {}) if config else {}
        self._nhc = Nhc(config)

        # NHC guards (ignav bldnhc: MAXVEL=0.5 m/s, MAXGYRO=30°/s)
        self.nhc_max_vel = ins_cfg.get("nhc_max_vel", 0.5)
        self.nhc_max_gyro = ins_cfg.get("nhc_max_gyro", 30.0) * math.pi / 180.0

        # ZUPT guards (ignav zvu: MAXVEL=0.1 m/s, MAXGYRO=10°/s)
        self.zupt_std = ins_cfg.get("zupt_std", 0.05)
        self.zupt_max_vel = ins_cfg.get("zupt_max_vel", 0.1)
        self.zupt_max_gyro = ins_cfg.get("zupt_max_gyro", 10.0) * math.pi / 180.0

        # ZARU (ignav zaru: MAXVEL=0.1 m/s, MAXGYRO=5°/s, VARARE=SQR(1°/s))
        self.zaru_std = ins_cfg.get("zaru_std", 1.0 * math.pi / 180.0)
        self.zaru_max_vel = ins_cfg.get("zaru_max_vel", 0.1)
        self.zaru_max_gyro = ins_cfg.get("zaru_max_gyro", 5.0) * math.pi / 180.0

    @property
    def nhc_builder(self) -> Nhc:
        """NHC 量测构造器 (对应 ignav bldnhc)。"""
        return self._nhc

    def nhc(self, estimator, imu: ImuMeasurement) -> bool:
        """NHC 量测更新 (2 维, 参考 ignav nhc/bldnhc)。

        Guards (ignav bldnhc: MAXVEL=0.5, MAXGYRO=30°/s):
          - ‖ω‖ < nhc_max_gyro (剧烈转弯跳过整个 NHC)
          - 单维 |v^v[i]| < nhc_max_vel (逐维检查, 超阈剔除该维)

        Args:
            estimator: LcEstimator (提供 state/ins_update/si/joseph_update)
            imu: IMU 测量

        Returns:
            True=已执行更新, False=guards 未通过
        """
        gyro_norm = float(np.linalg.norm(estimator.ins_update.w_b_ib))
        if gyro_norm >= self.nhc_max_gyro:
            return False
        Z, H, R = self._nhc.build_meas(estimator.state, imu, estimator.si)
        # 单维速度 guard: 逐维检查 |Z[i]|, 超阈剔除该维
        keep = np.abs(Z) < self.nhc_max_vel
        if not np.any(keep):
            return False
        if not np.all(keep):
            Z = Z[keep]
            H = H[keep, :]
            R = R[np.ix_(keep, keep)]
        estimator.joseph_update(Z, H, R)
        return True

    def zupt(self, estimator) -> bool:
        """ZUPT 量测更新 (3 维速度约束, 参考 ignav zvu)。

        Guards (ignav zvu: MAXVEL=0.1, MAXGYRO=10°/s):
          - ‖v^e‖ < zupt_max_vel
          - ‖ω‖ < zupt_max_gyro

        Returns:
            True=已执行更新, False=guards 未通过
        """
        state = estimator.state
        si = estimator.si
        vel_norm = float(np.linalg.norm(state.vel_e))
        gyro_norm = float(np.linalg.norm(estimator.ins_update.w_b_ib))
        if vel_norm >= self.zupt_max_vel or gyro_norm >= self.zupt_max_gyro:
            return False
        Z = state.vel_e.copy()
        H = np.zeros((3, si.dim), dtype=np.float64)
        H[:, si.vel:si.vel+3] = np.eye(3)
        R = np.diag([self.zupt_std ** 2] * 3).astype(np.float64)
        estimator.joseph_update(Z, H, R)
        return True

    def zaru(self, estimator, imu: ImuMeasurement) -> bool:
        """ZARU 量测更新 (3 维 gyro_bias 约束, 参考 ignav zaru)。

        静止时 ω_true = 0, imu.gyro = b_g。
        Z = -imu.gyro, H = -I(3) for gyro_bias, R = diag(zaru_std² × 3)。

        Guards (ignav zaru: MAXVEL=0.1, MAXGYRO=5°/s):
          - ‖v^e‖ < zaru_max_vel
          - ‖imu.gyro‖ < zaru_max_gyro

        Returns:
            True=已执行更新, False=guards 未通过
        """
        state = estimator.state
        si = estimator.si
        vel_norm = float(np.linalg.norm(state.vel_e))
        gyro_norm = float(np.linalg.norm(imu.gyro))
        if vel_norm >= self.zaru_max_vel or gyro_norm >= self.zaru_max_gyro:
            return False
        Z = -imu.gyro.copy()
        H = np.zeros((3, si.dim), dtype=np.float64)
        H[:, si.gyro_bias:si.gyro_bias+3] = -np.eye(3)
        R = np.diag([self.zaru_std ** 2] * 3).astype(np.float64)
        estimator.joseph_update(Z, H, R)
        return True
