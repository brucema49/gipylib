"""状态转移矩阵 F / Φ / Q 构造。

参考:
- GINav ins_time_updata.m / update_trans_mat.m (Q 构造与中间值法)
- 项目约定: 保留 Coriolis 项 (F_vv = -2*[ω_ie^e×], F_φφ = -[ω_ie^e×])
- ignav ins-gnss.cc (可选参数块 F=0, Q=random walk PSD)

状态顺序 (基础 15 维 + 可选块):
  固定: [δr^e(3), δv^e(3), δφ^e(3), δb_g(3), δb_a(3)]
  可选: [lever_arm(3), imu_angle(2), imu_leverarm(3), time_sync(1)]
"""
import math

import numpy as np

from src.core.ins.earth_param import (
    EARTH_ROTATION_RATE,
    ecef2llh,
    georadi,
    gravity_ecef,
)
from src.core.ins.state_index import StateIndex

# 传感器零偏相关时间 (h): 0.01h = 36s, 一阶 Gauss-Markov 过程相关时间
_CORR_TIME_BIAS_H = 0.01
# 可选状态参数过程噪声 PSD (随机游走, 无需调参)
_LEVER_ARM_PSD = 0.0          # 杆臂视为常数 (m²/s)
_IMU_ANGLE_PSD = 1.0e-6       # 安装角随机游走 (rad²/s)
_IMU_LEVERARM_PSD = 1.0e-8     # IMU 杆臂随机游走 (m²/s)
_TIME_SYNC_PSD = 1.0e-4       # 时间对齐随机游走 (s²/s)


def skew(v: np.ndarray) -> np.ndarray:
    """3 维向量 → 反对称矩阵。"""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([
        [0.0, -z,  y],
        [z,   0.0, -x],
        [-y,  x,  0.0],
    ], dtype=np.float64)


def rodrigues(phi: np.ndarray) -> np.ndarray:
    """旋转向量 → 旋转矩阵 (Rodrigues 公式)。"""
    phi = np.asarray(phi, dtype=np.float64)
    angle = float(np.linalg.norm(phi))
    if angle < 1e-12:
        return np.eye(3, dtype=np.float64)
    axis = phi / angle
    s = math.sin(angle)
    c = math.cos(angle)
    K = skew(axis)
    return np.eye(3, dtype=np.float64) + s * K + (1.0 - c) * (K @ K)


def _expm(A: np.ndarray, order: int = 10) -> np.ndarray:
    """矩阵指数 (scaling-and-squaring + Taylor 级数, 不依赖 scipy)。

    参考 ignav precPhi (ins-gnss.cc line 1064-1088) 的矩阵指数实现。

    Args:
        A: 方阵
        order: Taylor 级数阶数

    Returns:
        exp(A)
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


class TransferMatrix:
    """F / Φ / Q 矩阵构造器 (支持 StateIndex 动态维度)。

    基础 15 维: [pos(3), vel(3), att(3), gyro_bias(3), accel_bias(3)]
    可选块: lever_arm(3), imu_angle(2), imu_leverarm(3), time_sync(1)

    可选块 F=0 (常数或随机游走), Q 按参数类型填充。
    """

    def __init__(self, config: dict, state_index: StateIndex = None):
        ins_cfg = config.get("ins", {}) if config else {}
        self.si = state_index if state_index is not None else StateIndex.from_config(config or {})
        # 相关时间 (常数, h → s): 0.01h = 36s, 传感器零偏相关时间, 无需调参
        self.tau_gyro = _CORR_TIME_BIAS_H * 3600.0
        self.tau_acce = _CORR_TIME_BIAS_H * 3600.0
        # 过程噪声 PSD (从 config 直接读取, SI 单位)
        self.gyro_psd = ins_cfg.get("gyro_psd", 3.38802348178723e-09)
        self.accel_psd = ins_cfg.get("accel_psd", 2.60420170553977e-06)
        self.gyro_bias_psd = ins_cfg.get("gyro_bias_psd", 2.61160339323310e-14)
        self.acce_bias_psd = ins_cfg.get("acce_bias_psd", 1.66067346797506e-09)
        # 位置随机游走 PSD (m²/s): 计入未建模的位置不确定性 (RTK 跳变/多径等)
        self.pos_psd = ins_cfg.get("pos_psd", 0.0)
        # 速度随机游走 PSD (m²/s²): 计入未建模的速度不确定性, 防止 P_vel 坍缩致 K_vel→0
        # pos_psd 通过 F_rv 耦合也会增长 P_vel, 但 LC 模式下不够; vel_psd 直接注入 P_vel
        self.vel_psd = ins_cfg.get("vel_psd", 0.0)
        # 可选参数过程噪声 PSD (常数, 无需调参)
        self.lever_arm_psd = _LEVER_ARM_PSD
        self.imu_angle_psd = _IMU_ANGLE_PSD
        self.imu_leverarm_psd = _IMU_LEVERARM_PSD
        self.time_sync_psd = _TIME_SYNC_PSD
        # 地球自转角速度 (E 系常数向量)
        self.w_ie_e = np.array([0.0, 0.0, EARTH_ROTATION_RATE], dtype=np.float64)

    def build_F(self, C_b_e: np.ndarray, f_b: np.ndarray,
                w_b_ib: np.ndarray, pos_e: np.ndarray) -> np.ndarray:
        """构造 N×N 连续时间 F 矩阵 (ψ-error 模型, 对齐 ignav)。

        基础 15×15 块复用现有逻辑, 可选块 F=0 (常数/随机游走)。

        Args:
            C_b_e: 3x3 旋转矩阵 b→e
            f_b: 3 比力 (b 系, m/s²)
            w_b_ib: 3 角速度 (b 系, rad/s)
            pos_e: 3 ECEF 位置 (m)

        Returns:
            N×N F 矩阵 (N = si.dim)
        """
        n = self.si.dim
        F = np.zeros((n, n), dtype=np.float64)

        # F_rv = I (位置-速度耦合)
        F[0:3, 3:6] = np.eye(3)

        # F_vr = -2/(re·|pos|) · ge ⊗ pos  (重力梯度, 参考 ignav getF)
        ge = gravity_ecef(pos_e)
        lat, _, _ = ecef2llh(pos_e)
        re = georadi(lat)
        pos_norm = np.linalg.norm(pos_e)
        if pos_norm > 1.0:
            F[3:6, 0:3] = -2.0 / (re * pos_norm) * np.outer(ge, pos_e)

        # F_vv = -2*[ω_ie^e×]  (Coriolis)
        F[3:6, 3:6] = -2.0 * skew(self.w_ie_e)

        # F_vψ = -[C_b_e·f_b×]  (ψ-error: 负号, 对齐 ignav)
        f_e = C_b_e @ f_b
        F[3:6, 6:9] = -skew(f_e)

        # F_vba = C_b_e  (加计零偏 → 速度)
        F[3:6, 12:15] = C_b_e

        # F_ψψ = -[ω_ie^e×]  (Coriolis)
        F[6:9, 6:9] = -skew(self.w_ie_e)

        # F_ψbg = +C_b_e  (ψ-error: 正号, 对齐 ignav)
        F[6:9, 9:12] = C_b_e

        # F_bgbg = -I / tau_gyro  (陀螺零偏一阶马尔可夫)
        F[9:12, 9:12] = -np.eye(3) / self.tau_gyro

        # F_baba = -I / tau_acce  (加计零偏一阶马尔可夫)
        F[12:15, 12:15] = -np.eye(3) / self.tau_acce

        # 可选块: F=0 (lever_arm/imu_angle/imu_leverarm/time_sync 均为常数或随机游走)
        return F

    def build_F_ins(self, C_b_e: np.ndarray, f_b: np.ndarray,
                    w_b_ib: np.ndarray, pos_e: np.ndarray,
                    n_ins: int) -> np.ndarray:
        """仅构建 INS 块 F 矩阵 (n_ins × n_ins), 性能优化用。

        GNSS 块 (clk_bias/ambiguity) F=0, 无需构建全 n×n 矩阵。
        """
        F = np.zeros((n_ins, n_ins), dtype=np.float64)
        F[0:3, 3:6] = np.eye(3)

        ge = gravity_ecef(pos_e)
        lat, _, _ = ecef2llh(pos_e)
        re = georadi(lat)
        pos_norm = np.linalg.norm(pos_e)
        if pos_norm > 1.0:
            F[3:6, 0:3] = -2.0 / (re * pos_norm) * np.outer(ge, pos_e)

        F[3:6, 3:6] = -2.0 * skew(self.w_ie_e)
        f_e = C_b_e @ f_b
        F[3:6, 6:9] = -skew(f_e)
        F[3:6, 12:15] = C_b_e
        F[6:9, 6:9] = -skew(self.w_ie_e)
        F[6:9, 9:12] = C_b_e
        F[9:12, 9:12] = -np.eye(3) / self.tau_gyro
        F[12:15, 12:15] = -np.eye(3) / self.tau_acce
        return F

    def build_Q_ins(self, dt: float, C_b_e: np.ndarray,
                     n_ins: int) -> np.ndarray:
        """仅构建 INS 块 Q 矩阵 (n_ins × n_ins), 性能优化用。

        GNSS 块 Q=0 (ambiguity 随机游走 Q=0, clk 白噪声单独处理)。
        """
        si = self.si
        G = np.zeros((n_ins, 15), dtype=np.float64)
        G[0:3, 0:3] = np.eye(3)
        G[6:9, 6:9] = C_b_e
        G[3:6, 3:6] = C_b_e
        G[9:12, 9:12] = np.eye(3)
        G[12:15, 12:15] = np.eye(3)

        Q_diag = np.zeros((15, 15), dtype=np.float64)
        Q_diag[0:3, 0:3] = np.diag([self.pos_psd * dt] * 3)
        Q_diag[3:6, 3:6] = np.diag([self.accel_psd * dt] * 3)
        Q_diag[6:9, 6:9] = np.diag([self.gyro_psd * dt] * 3)
        Q_diag[9:12, 9:12] = np.diag([self.gyro_bias_psd * dt] * 3)
        Q_diag[12:15, 12:15] = np.diag([self.acce_bias_psd * dt] * 3)

        Q = G @ Q_diag @ G.T

        # 可选块 Q (仅在 n_ins 范围内)
        if si.has_lever_arm() and self.lever_arm_psd > 0.0:
            i = si.lever_arm
            if i + 3 <= n_ins:
                Q[i:i+3, i:i+3] = np.diag([self.lever_arm_psd * dt] * 3)
        if si.has_imu_angle():
            i = si.imu_angle
            if i + 2 <= n_ins:
                Q[i:i+2, i:i+2] = np.diag([self.imu_angle_psd * dt] * 2)
        if si.has_imu_leverarm():
            i = si.imu_leverarm
            if i + 3 <= n_ins:
                Q[i:i+3, i:i+3] = np.diag([self.imu_leverarm_psd * dt] * 3)
        if si.has_time_sync():
            i = si.time_sync
            if i < n_ins:
                Q[i, i] = self.time_sync_psd * dt
        return Q

    def build_Phi(self, F: np.ndarray, dt: float) -> np.ndarray:
        """离散化: 自适应精度 (对齐 ignav precPhi)。

        - dt <= 0.005s  (≥200Hz): 一阶 Φ = I + F·dt
        - dt <= 0.01s   (100-200Hz): 二阶 Φ = I + F·dt + 0.5·(F·dt)²
        - dt > 0.01s    (<100Hz): 矩阵指数 Φ = expm(F·dt)
        """
        n = self.si.dim
        Fdt = F * dt
        if dt <= 0.005:
            return np.eye(n, dtype=np.float64) + Fdt
        elif dt <= 0.01:
            return np.eye(n, dtype=np.float64) + Fdt + 0.5 * (Fdt @ Fdt)
        else:
            return _expm(Fdt)

    def build_Q(self, dt: float, C_b_e: np.ndarray) -> np.ndarray:
        """构造 N×N 离散 Q 矩阵 (GINav G·Q_diag·G^T 风格 + 可选块)。

        Args:
            dt: 时间步长 (s)
            C_b_e: 3x3 旋转矩阵 b→e

        Returns:
            N×N Q 矩阵 (N = si.dim)
        """
        n = self.si.dim
        si = self.si

        # 基础 15×15: G·Q_diag·G^T
        G = np.zeros((n, 15), dtype=np.float64)
        G[0:3, 0:3] = np.eye(3)
        G[6:9, 6:9] = C_b_e
        G[3:6, 3:6] = C_b_e
        G[9:12, 9:12] = np.eye(3)
        G[12:15, 12:15] = np.eye(3)

        Q_diag = np.zeros((15, 15), dtype=np.float64)
        Q_diag[0:3, 0:3] = np.diag([self.pos_psd * dt] * 3)
        Q_diag[3:6, 3:6] = np.diag([self.accel_psd * dt] * 3)
        Q_diag[6:9, 6:9] = np.diag([self.gyro_psd * dt] * 3)
        Q_diag[9:12, 9:12] = np.diag([self.gyro_bias_psd * dt] * 3)
        Q_diag[12:15, 12:15] = np.diag([self.acce_bias_psd * dt] * 3)

        Q = G @ Q_diag @ G.T

        # 直接速度过程噪声 (不经过 G 旋转, 直接注入 ECEF vel 对角块)
        if self.vel_psd > 0.0:
            Q[3:6, 3:6] += np.diag([self.vel_psd * dt] * 3)

        # 可选块 Q (随机游走: PSD × dt)
        if si.has_lever_arm() and self.lever_arm_psd > 0.0:
            i = si.lever_arm
            Q[i:i+3, i:i+3] = np.diag([self.lever_arm_psd * dt] * 3)

        if si.has_imu_angle():
            i = si.imu_angle
            Q[i:i+2, i:i+2] = np.diag([self.imu_angle_psd * dt] * 2)

        if si.has_imu_leverarm():
            i = si.imu_leverarm
            Q[i:i+3, i:i+3] = np.diag([self.imu_leverarm_psd * dt] * 3)

        if si.has_time_sync():
            i = si.time_sync
            Q[i, i] = self.time_sync_psd * dt

        return Q
