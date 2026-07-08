"""状态转移矩阵 F / Φ / Q 构造。

参考:
- gnss_ins_lc_nhc navmech.cc MechTransferMat (E 系 F 矩阵结构)
- GINav ins_time_updata.m / update_trans_mat.m (Q 构造与中间值法)
- 项目约定: 保留 Coriolis 项 (F_vv = -2*[ω_ie^e×], F_φφ = -[ω_ie^e×])

状态顺序 (15 维):
  [δr^e(3), δv^e(3), δφ^e(3), δb_g(3), δb_a(3)]
"""
import math

import numpy as np

from src.core.ins.earth_param import (
    EARTH_ROTATION_RATE,
    ecef2llh,
    georadi,
    gravity_ecef,
)


def skew(v: np.ndarray) -> np.ndarray:
    """3 维向量 → 反对称矩阵。"""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([
        [0.0, -z,  y],
        [z,   0.0, -x],
        [-y,  x,  0.0],
    ], dtype=np.float64)


def rodrigues(phi: np.ndarray) -> np.ndarray:
    """旋转向量 → 旋转矩阵 (Rodrigues 公式)。

    参考 gnss_ins_lc_nhc RotationVector2Matrix。
    """
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
    """F / Φ / Q 矩阵构造器。

    15 维状态: [pos(3), vel(3), att(3), gyro_bias(3), accel_bias(3)]
    """

    def __init__(self, config: dict):
        ins_cfg = config.get("ins", {})
        # 相关时间 (h → s)
        self.tau_gyro = ins_cfg.get("corr_time_of_gyro_bias", 0.01) * 3600.0
        self.tau_acce = ins_cfg.get("corr_time_of_acce_bias", 0.01) * 3600.0
        # 过程噪声 PSD (从 config 直接读取, SI 单位)
        self.gyro_psd = ins_cfg.get("gyro_psd", 3.38802348178723e-09)
        self.accel_psd = ins_cfg.get("accel_psd", 2.60420170553977e-06)
        self.gyro_bias_psd = ins_cfg.get("gyro_bias_psd", 2.61160339323310e-14)
        self.acce_bias_psd = ins_cfg.get("acce_bias_psd", 1.66067346797506e-09)
        # 地球自转角速度 (E 系常数向量)
        self.w_ie_e = np.array([0.0, 0.0, EARTH_ROTATION_RATE], dtype=np.float64)

    def build_F(self, C_b_e: np.ndarray, f_b: np.ndarray,
                w_b_ib: np.ndarray, pos_e: np.ndarray) -> np.ndarray:
        """构造 15x15 连续时间 F 矩阵 (ψ-error 模型, 对齐 ignav)。

        Args:
            C_b_e: 3x3 旋转矩阵 b→e
            f_b: 3 比力 (b 系, m/s²)
            w_b_ib: 3 角速度 (b 系, rad/s)
            pos_e: 3 ECEF 位置 (m)

        Returns:
            15x15 F 矩阵
        """
        F = np.zeros((15, 15), dtype=np.float64)

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

        return F

    def build_Phi(self, F: np.ndarray, dt: float) -> np.ndarray:
        """离散化: Φ = I + F·dt + 0.5·(F·dt)² (二阶 Taylor)。"""
        Fdt = F * dt
        return np.eye(15, dtype=np.float64) + Fdt + 0.5 * (Fdt @ Fdt)

    def build_Q(self, dt: float, C_b_e: np.ndarray) -> np.ndarray:
        """构造 15x15 离散 Q 矩阵 (GINav G·Q_diag·G^T 风格)。

        Args:
            dt: 时间步长 (s)
            C_b_e: 3x3 旋转矩阵 b→e

        Returns:
            15x15 Q 矩阵 (状态噪声协方差)
        """
        # G 矩阵 (15x15): 将噪声映射到状态空间
        G = np.zeros((15, 15), dtype=np.float64)
        G[6:9, 6:9] = -C_b_e      # 陀螺噪声 → 姿态
        G[3:6, 3:6] = C_b_e       # 加计噪声 → 速度
        G[9:12, 9:12] = np.eye(3)  # 陀螺零偏驱动噪声
        G[12:15, 12:15] = np.eye(3)  # 加计零偏驱动噪声

        # Q_diag (15x15 对角): 噪声 PSD × dt
        Q_diag = np.zeros((15, 15), dtype=np.float64)
        Q_diag[3:6, 3:6] = np.diag([self.accel_psd * dt] * 3)
        Q_diag[6:9, 6:9] = np.diag([self.gyro_psd * dt] * 3)
        Q_diag[9:12, 9:12] = np.diag([self.gyro_bias_psd * dt] * 3)
        Q_diag[12:15, 12:15] = np.diag([self.acce_bias_psd * dt] * 3)

        # Q0 = G · Q_diag · G^T
        return G @ Q_diag @ G.T
