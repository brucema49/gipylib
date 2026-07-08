"""INS 机械编排核心 (E 系 ECEF)。

参考:
- gnss_ins_lc_nhc navmech.cc (E 系机械编排方程)
- GINav ins_mech.m (算法结构: 锥补/划桨/旋转补偿/中点)

本项目 IMU 为速率式 (gyro: rad/s, accel: m/s²),
需 ×dt 转为增量后套用增量式算法。

状态更新顺序: 姿态 → 速度 → 位置 (参考 gnss_ins_lc_nhc MechanicalArrangement)
"""
import logging
import math

import numpy as np

from src.core.data_types import ImuMeasurement, InsState
from src.core.ins.earth_param import (
    EARTH_ROTATION_RATE,
    cal_Ce2n,
    ecef2llh,
    gravity_ecef,
)
from src.core.ins.attitude import dcm2euler, dcm2quat
from src.core.ins.transfer_matrix import rodrigues, skew

logger = logging.getLogger(__name__)


class InsUpdate:
    """INS 机械编排 (E 系)。

    维护 InsState, 每历元执行姿态/速度/位置正向递推。
    不持有协方差 (协方差由 InsPropagate 维护)。

    """

    def __init__(self, state: InsState):
        self.state = state
        # 上一历元的增量 (用于锥补/划桨补偿)
        self._prev_dtheta = np.zeros(3, dtype=np.float64)
        self._prev_dvel = np.zeros(3, dtype=np.float64)
        self._prev_timestamp = state.timestamp
        # 当前历元的比力/角速度 (b 系), 供 InsPropagate 构造 F 矩阵
        self._f_b = np.zeros(3, dtype=np.float64)
        self._w_b_ib = np.zeros(3, dtype=np.float64)

    @property
    def f_b(self) -> np.ndarray:
        """当前历元比力 (b 系, m/s²)。"""
        return self._f_b

    @property
    def w_b_ib(self) -> np.ndarray:
        """当前历元角速度 (b 系, rad/s)。"""
        return self._w_b_ib

    def update(self, imu: ImuMeasurement) -> InsState:
        """一步 INS 递推: 姿态 → 速度 → 位置 (E 系)。

        Args:
            imu: IMU 测量 (速率式: gyro rad/s, accel m/s²)

        Returns:
            更新后的 InsState
        """
        dt = imu.timestamp - self._prev_timestamp
        if dt <= 0.0:
            logger.warning(
                f"非正 dt={dt:.6f} (t_curr={imu.timestamp:.6f}, "
                f"t_prev={self._prev_timestamp:.6f}), 跳过"
            )
            return self.state
        if dt > 1.0:
            logger.warning(f"异常大 dt={dt:.4f}s, 继续递推")

        # 1. IMU 补偿: 速率 → 增量, 减零偏, 乘 (1-scale)
        dtheta = imu.gyro * dt
        dvel = imu.accel * dt
        dtheta_comp = (dtheta - self.state.gyro_bias * dt) * (1.0 - self.state.gyro_scale)
        dvel_comp = (dvel - self.state.accel_bias * dt) * (1.0 - self.state.accel_scale)

        # 记录当前历元比力/角速度 (b 系, 速率) 供 InsPropagate 使用
        self._w_b_ib = dtheta_comp / dt
        self._f_b = dvel_comp / dt

        # 2. 姿态更新 (含锥补)
        C_b_e_new = self._attitude_update(dtheta_comp)

        # 3. 速度更新 (含旋转/划桨补偿)
        vel_e_new = self._velocity_update(dtheta_comp, dvel_comp, dt)

        # 4. 位置更新 (梯形)
        pos_e_new = self._position_update(vel_e_new, dt)

        # 5. 装配新状态
        # 欧拉角需从 C_b^n 提取 (dcm2euler 设计用于 C_b^n, 非 C_b^e)
        lat, lon, _ = ecef2llh(pos_e_new)
        C_e_n = cal_Ce2n(lat, lon)
        C_b_n_new = C_e_n @ C_b_e_new
        att_rpy = dcm2euler(C_b_n_new)
        q_b_e = dcm2quat(C_b_e_new)
        self.state = InsState(
            timestamp=imu.timestamp,
            pos_e=pos_e_new,
            vel_e=vel_e_new,
            C_b_e=C_b_e_new,
            q_b_e=q_b_e,
            att_rpy=att_rpy,
            gyro_bias=self.state.gyro_bias,
            accel_bias=self.state.accel_bias,
            gyro_scale=self.state.gyro_scale,
            accel_scale=self.state.accel_scale,
            imu_angle=self.state.imu_angle,
            imu_leverarm=self.state.imu_leverarm,
            leverarm=self.state.leverarm,
        )

        # 6. 更新历史增量 (用于下一历元锥补/划桨, 存储已补偿值, 参考 ignav omgbp/fbp)
        self._prev_dtheta = dtheta_comp.copy()
        self._prev_dvel = dvel_comp.copy()
        self._prev_timestamp = imu.timestamp

        return self.state

    def _attitude_update(self, dtheta_comp: np.ndarray) -> np.ndarray:
        """姿态更新 (E 系, 含锥补)。

        参考 gnss_ins_lc_nhc MechAttitudeUpdate:
          phi_b = dtheta + skew(dtheta_prev) * dtheta / 12  (锥补)
          C_bb = rodrigues(phi_b)
          zeta = [0,0,ω_ie] * dt
          C_ee = rodrigues(-zeta)  (地球自转补偿)
          C_b_e_new = C_ee @ C_b_e_prev @ C_bb
        """
        # 锥补
        phi_b = dtheta_comp + skew(self._prev_dtheta) @ dtheta_comp / 12.0
        C_bb = rodrigues(phi_b)

        # 地球自转补偿 (E 系下 ECEF 随地球自转)
        dt = self.state.timestamp - self._prev_timestamp if self._prev_timestamp else 0.0
        zeta = np.array([0.0, 0.0, EARTH_ROTATION_RATE], dtype=np.float64) * dt
        C_ee = rodrigues(-zeta)

        return C_ee @ self.state.C_b_e @ C_bb

    def _velocity_update(self, dtheta_comp: np.ndarray,
                         dvel_comp: np.ndarray, dt: float) -> np.ndarray:
        """速度更新 (E 系, 含旋转/划桨补偿)。

        参考 gnss_ins_lc_nhc MechVelocityUpdate:
          v_rot  = 0.5 * cross(dtheta, dvel)            (旋转补偿)
          v_scul = (cross(dtheta_prev, dvel) + cross(dvel_prev, dtheta)) / 12  (划桨)
          delta_v_cor = (g_e - 2*cross(ω_ie, vel)) * dt  (重力+科氏)
          C_ee_v = I - skew(ω_ie * 0.5 * dt)
          delta_v = C_ee_v @ C_b_e @ (dvel + v_rot + v_scul)
          vel_new = vel + delta_v_cor + delta_v
        """
        # 旋转补偿 (精确 Rodrigues, 参考 ignav rotscull_corr)
        dak = dtheta_comp
        dvk = dvel_comp
        dak_norm = float(np.linalg.norm(dak))
        if dak_norm < 1e-12:
            v_rot = np.zeros(3, dtype=np.float64)
        else:
            dak_sq = dak_norm * dak_norm
            a1 = (1.0 - math.cos(dak_norm)) / dak_sq
            a2 = (1.0 - math.sin(dak_norm) / dak_norm) / dak_sq
            v_rot = a1 * np.cross(dak, dvk) + a2 * np.cross(dak, np.cross(dak, dvk))
        # 划桨补偿
        v_scul = (np.cross(self._prev_dtheta, dvel_comp)
                  + np.cross(self._prev_dvel, dtheta_comp)) / 12.0

        # 重力 + 科氏
        g_e = gravity_ecef(self.state.pos_e)
        w_ie_e = np.array([0.0, 0.0, EARTH_ROTATION_RATE], dtype=np.float64)
        delta_v_cor = (g_e - 2.0 * np.cross(w_ie_e, self.state.vel_e)) * dt

        # C_ee_v = I - skew(ω_ie * 0.5 * dt)
        C_ee_v = np.eye(3, dtype=np.float64) - skew(w_ie_e * 0.5 * dt)

        delta_v = C_ee_v @ self.state.C_b_e @ (dvel_comp + v_rot + v_scul)
        return self.state.vel_e + delta_v_cor + delta_v

    def _position_update(self, vel_e_new: np.ndarray, dt: float) -> np.ndarray:
        """位置更新 (E 系, 梯形积分)。

        参考 gnss_ins_lc_nhc MechPositionUpdate:
          pos_new = pos + 0.5 * (vel_prev + vel_new) * dt
        """
        return self.state.pos_e + 0.5 * (self.state.vel_e + vel_e_new) * dt
