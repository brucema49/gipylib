"""NHC/ZUPT/ZARU 约束量测更新 (独立模块, 参考 ignav 分离架构)。

参考:
- ignav ins-nhc.cc (bldnhc + nhc): NHC 量测构造 + 滤波
- ignav ins-zvu.cc  (detstc + zvu): 静态检测 + 零速更新
- ignav ins-zaru.cc (zaru):         零角速率更新
- gici-open nhc_error.cpp:          NHC 残差/协方差由安装角不确定度传播
- GREAT-MSF gins.cpp motion_state(): 约束触发的运动学判据

架构 (参考 ignav 分离模式):
  本模块 = 约束逻辑 (guards + Z/H/R 构造 + filter 调用)
  LcEstimator.joseph_update = 滤波步骤 (对应 ignav filter())
  LcEstimator.feedback      = 反馈校正 (对应 ignav clp())

约束互斥 (参考 ignav postpos.cc):
  静态 → ZUPT + ZARU
  运动 → NHC

符号约定 (skills/NHC_ZUPT.md §1, 本模块所有 H 矩阵都遵守):
  Z = h(x̂) - z        h=预测函数, z=量测值 (NHC/ZUPT 的 z=0, ZARU 的 z=imu.gyro)
  H = -M⁻¹·∂h/∂x_true  M 为反馈映射: pos/vel/imu_leverarm 为 -I, bias/imu_angle 为 +I
  ξ = K·Z, K = P Hᵀ (H P Hᵀ + R)⁻¹   (LcEstimator.joseph_update)
  施加反馈后 r(new) = h(new) - z ≈ Z - H·ξ
"""
import math

import numpy as np

from src.core.data_types import ImuMeasurement, InsState
from src.core.ins.state_index import StateIndex
from src.core.ins.transfer_matrix import skew


def _build_R_b_v(pitch: float, yaw: float) -> np.ndarray:
    """IMU 安装角 [pitch, yaw] → R_b^v (b→v 旋转矩阵, roll=0)。

    R_b^v = R_z(yaw) · R_y(pitch)  (ZYX 顺序, roll=0)
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

    R 矩阵 (参考 gici-open nhc_error.cpp):
      R = diag(nhc_std²) + σ_att² · J_att · J_attᵗ
      J_att = (R_b^v · C_e^b · [v^e ×])[1:3, :]
    第二项把"安装角不确定度"按速度大小传播成侧向/垂向速度的等效噪声:
    真实的 v^v[1:3] 并不严格为 0, 安装角误差 δθ 会产生 |v|·δθ 的虚假速度。
    这一项随速度线性增长, 是 gici 用 σ_att=3° 代替固定 σ 的原因。
    注意: 该项**只用安装角 σ**, 不使用滤波器当前的 P_att/P_vel —— 若用 P,
    NHC 本身把 P 压小后会反过来把 R 压小, 形成正反馈直至滤波器发散。
    """

    def __init__(self, config: dict):
        ins_cfg = config.get("ins", {}) if config else {}
        self.nhc_std = float(ins_cfg.get("nhc_std", 0.1))  # m/s
        # 安装角不确定度 [rad] → NHC 自适应 R (gici body_to_imu_rotation_std)
        self.attitude_std = math.radians(float(
            ins_cfg.get("nhc_attitude_std_deg", 3.0)))
        # 约束分量: "2d"=侧向+垂向 (ignav/gici), "lateral"=仅侧向
        self.axis_mode = str(ins_cfg.get("nhc_axis", "2d")).lower()
        self.R_b_v = np.eye(3, dtype=np.float64)
        self.imu_angle = np.zeros(2, dtype=np.float64)
        self.imu_leverarm = np.zeros(3, dtype=np.float64)

    @property
    def rows(self) -> np.ndarray:
        """被约束的 v^v 分量下标。"""
        return np.array([1] if self.axis_mode == "lateral" else [1, 2],
                        dtype=int)

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
            (Z[n], H[n, N], R[n,n])  n = 1 (lateral) 或 2 (2d)
        """
        self.update_from_state(state)

        C_b_e = state.C_b_e
        C_e_b = C_b_e.T
        v_e = state.vel_e
        w_b_ib = imu.rate_view().gyro - state.gyro_bias
        l_imu_b = self.imu_leverarm
        rows = self.rows

        # v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b ×] · l_imu^b
        Rbv_Ceb = self.R_b_v @ C_e_b
        v_v = Rbv_Ceb @ v_e + self.R_b_v @ skew(w_b_ib) @ l_imu_b
        Z = v_v[rows].copy()

        H = np.zeros((rows.size, si.dim), dtype=np.float64)

        # --- 基础 15 维 ---
        # H[:, vel] = (R_b^v · C_e^b)[rows, :]
        H[:, si.vel:si.vel + 3] = Rbv_Ceb[rows, :]
        # H[:, att] = (R_b^v · C_e^b · [v^e ×])[rows, :]
        J_att = (Rbv_Ceb @ skew(v_e))[rows, :]
        H[:, si.att:si.att + 3] = J_att
        # H[:, gyro_bias] = -(R_b^v · [l_imu^b ×])[rows, :]
        H[:, si.gyro_bias:si.gyro_bias + 3] = \
            -(self.R_b_v @ skew(l_imu_b))[rows, :]

        # --- 可选: 安装角 (δangle = angle_true - angle_est, 反馈加) ---
        # R_b^v(θ+Δ) = (I + [η×]) R_b^v,  η = ẑ·Δyaw + (R_b^v ŷ)·Δpitch
        # ∂v^v/∂Δ = η × v^v = -[v^v ×] η
        # 反馈加 ⇒ H = -∂h/∂x_true = +[v^v ×] [R_b^v ŷ, ẑ]
        if si.has_imu_angle():
            J_angle = skew(v_v) @ np.column_stack(
                (self.R_b_v[:, 1], np.array([0.0, 0.0, 1.0])))
            H[:, si.imu_angle:si.imu_angle + 2] = J_angle[rows, :]

        # --- 可选: IMU 杆臂 (δl = l_est - l_true, 反馈减 ⇒ H = +∂h/∂l_true) ---
        if si.has_imu_leverarm():
            H[:, si.imu_leverarm:si.imu_leverarm + 3] = \
                (self.R_b_v @ skew(w_b_ib))[rows, :]

        # --- R: 固定项 + 安装角不确定度传播项 (gici) ---
        R = np.eye(rows.size) * self.nhc_std ** 2
        if self.attitude_std > 0.0:
            R = R + (self.attitude_std ** 2) * (J_att @ J_att.T)
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

        # NHC guards (ignav bldnhc: MAXVEL=0.5 m/s, MAXGYRO=30°/s;
        # gici car_motion_min_velocity=3 m/s 可选)
        self.nhc_max_vel = float(ins_cfg.get("nhc_max_vel", 0.5))
        self.nhc_max_gyro = float(ins_cfg.get("nhc_max_gyro", 30.0)) * math.pi / 180.0
        self.nhc_min_vel = float(ins_cfg.get("nhc_min_vel", 0.0))

        # ZUPT guards (ignav zvu: MAXVEL=0.1 m/s, MAXGYRO=10°/s)
        self.zupt_std = float(ins_cfg.get("zupt_std", 0.05))
        self.zupt_max_vel = float(ins_cfg.get("zupt_max_vel", 0.1))
        self.zupt_max_gyro = float(ins_cfg.get("zupt_max_gyro", 10.0)) * math.pi / 180.0

        # ZARU (ignav zaru: MAXVEL=0.1 m/s, MAXGYRO=5°/s, VARARE=SQR(1°/s))
        self.zaru_std = float(ins_cfg.get("zaru_std", 1.0 * math.pi / 180.0))
        self.zaru_max_vel = float(ins_cfg.get("zaru_max_vel", 0.1))
        self.zaru_max_gyro = float(ins_cfg.get("zaru_max_gyro", 5.0)) * math.pi / 180.0

    @property
    def nhc_builder(self) -> Nhc:
        """NHC 量测构造器 (对应 ignav bldnhc)。"""
        return self._nhc

    def build_nhc(self, estimator, imu: ImuMeasurement):
        """NHC 的 guards + 量测构造，**不做**滤波更新。

        供 MotionConstraintManager 使用（它需要拿到 (Z, H, R) 以便做残差记录
        与抗差调整）；单独使用时用 :meth:`nhc`。

        Returns:
            (Z, H, R) 或 None（guard 未通过）
        """
        gyro_norm = float(np.linalg.norm(estimator.ins_update.w_b_ib))
        if gyro_norm >= self.nhc_max_gyro:
            return None
        vel_norm = float(np.linalg.norm(estimator.state.vel_e))
        if vel_norm < self.nhc_min_vel:
            return None
        Z, H, R = self._nhc.build_meas(estimator.state, imu, estimator.si)
        # 单维速度 guard: 逐维检查 |Z[i]|, 超阈剔除该维 (ignav bldnhc)
        keep = np.abs(Z) < self.nhc_max_vel
        if not np.any(keep):
            return None
        if not np.all(keep):
            Z = Z[keep]
            H = H[keep, :]
            R = R[np.ix_(keep, keep)]
        return Z, H, R

    def nhc(self, estimator, imu: ImuMeasurement) -> bool:
        """NHC 量测更新 (参考 ignav nhc/bldnhc + gici addNHCResidualBlock)。

        等价于 ``build_nhc`` + ``estimator.joseph_update``。

        Guards:
          - ‖ω‖ < nhc_max_gyro (剧烈转弯跳过整个 NHC, ignav)
          - ‖v^e‖ >= nhc_min_vel (低速不施加, gici car_motion_min_velocity)
          - 单维 |Z[i]| < nhc_max_vel (逐维检查, 超阈剔除该维, ignav)

        R 由 Nhc.build_meas 按 nhc_std + 安装角 σ 传播构造 (gici), 不使用
        滤波器当前 P 自适应放大 —— 见 Nhc 类文档中的正反馈说明。

        Args:
            estimator: LcEstimator (提供 state/ins_update/si/joseph_update)
            imu: IMU 测量

        Returns:
            True=已执行更新, False=guards 未通过
        """
        built = self.build_nhc(estimator, imu)
        if built is None:
            return False
        Z, H, R = built
        estimator.joseph_update(Z, H, R)
        return True

    def build_zupt(self, estimator):
        """ZUPT 的 guards + 量测构造，不做滤波更新（同 :meth:`build_nhc`）。"""
        state = estimator.state
        si = estimator.si
        vel_norm = float(np.linalg.norm(state.vel_e))
        gyro_norm = float(np.linalg.norm(estimator.ins_update.w_b_ib))
        if vel_norm >= self.zupt_max_vel or gyro_norm >= self.zupt_max_gyro:
            return None
        Z = state.vel_e.copy()
        H = np.zeros((3, si.dim), dtype=np.float64)
        H[:, si.vel:si.vel + 3] = np.eye(3)
        R = np.diag([self.zupt_std ** 2] * 3).astype(np.float64)
        return Z, H, R

    def zupt(self, estimator) -> bool:
        """ZUPT 量测更新 (3 维速度约束, 参考 ignav zvu)。

        Z = v^e (z = 0), H[:, vel] = I(3), R = diag(zupt_std²)。

        Guards (ignav zvu: MAXVEL=0.1, MAXGYRO=10°/s):
          - ‖v^e‖ < zupt_max_vel
          - ‖ω‖ < zupt_max_gyro

        Returns:
            True=已执行更新, False=guards 未通过
        """
        built = self.build_zupt(estimator)
        if built is None:
            return False
        Z, H, R = built
        estimator.joseph_update(Z, H, R)
        return True

    def build_zaru(self, estimator, imu: ImuMeasurement):
        """ZARU 的 guards + 量测构造，不做滤波更新（同 :meth:`build_nhc`）。"""
        state = estimator.state
        si = estimator.si
        vel_norm = float(np.linalg.norm(state.vel_e))
        omega = imu.rate_view().gyro - state.gyro_bias
        gyro_norm = float(np.linalg.norm(omega))
        if vel_norm >= self.zaru_max_vel or gyro_norm >= self.zaru_max_gyro:
            return None
        Z = state.gyro_bias - imu.rate_view().gyro
        H = np.zeros((3, si.dim), dtype=np.float64)
        H[:, si.gyro_bias:si.gyro_bias + 3] = -np.eye(3)
        R = np.diag([self.zaru_std ** 2] * 3).astype(np.float64)
        return Z, H, R

    def zaru(self, estimator, imu: ImuMeasurement) -> bool:
        """ZARU 量测更新 (3 维 gyro_bias 约束, 参考 ignav zaru)。

        静止时 ω_true = 0 ⇒ imu.gyro = b_g。预测 h(x) = b_g, 量测 z = imu.gyro:
          Z = h(x̂) - z = gyro_bias - imu.gyro
          H[:, gyro_bias] = -I(3)   (bias 块反馈为 "+", 故 H = -∂h/∂x_true)
          R = diag(zaru_std² × 3)

        > ignav ins-zaru.cc 写 v = -imu.gyro, 漏掉了 +b_est 项。那样每次更新
        > 都会给零偏估计叠加 k·imu.gyro, 静止段内零偏会单调发散 (见
        > tests/test_motion_constraints.py::test_zaru_residual_includes_bias_estimate)。

        Guards (ignav zaru: MAXVEL=0.1, MAXGYRO=5°/s):
          - ‖v^e‖ < zaru_max_vel
          - ‖ω_comp‖ = ‖imu.gyro - b_est‖ < zaru_max_gyro
            (ignav 用原始 imu.gyro; 原始量含零偏, 大零偏 MEMS 会永久失效)

        Returns:
            True=已执行更新, False=guards 未通过
        """
        built = self.build_zaru(estimator, imu)
        if built is None:
            return False
        Z, H, R = built
        estimator.joseph_update(Z, H, R)
        return True
