"""IMU 静态检测 (滑动窗口, 多方法)。

参考 tools/ignav/ins-gnss/ins-static-detect.cc:
  - detstatic_GLRT: 广义似然比检验
  - detstatic_MV:   加速度滑动方差
  - detstatic_MAG:  加速度模值
  - detstatic_ARE:  角速率能量
  - detstatic_ALL:  四法与运算

窗口机制: 维护最近 ws 个 IMU 历元的 accel/gyro, 每次调用 detect()
对窗口数据计算检验统计量 T, 与阈值 gamma 比较。
"""
from collections import deque
from typing import Deque, List

import numpy as np

from src.core.data_types import ImuMeasurement
from src.core.ins.earth_param import gravity_ecef


class StaticDetect:
    """IMU 静态检测 (滑动窗口)。

    维护最近 ws 个 IMU 历元的 accel/gyro, 按配置方法计算统计量。
    """

    def __init__(self, config: dict):
        ins_cfg = config.get("ins", {}) if config else {}
        self.method = ins_cfg.get("static_detect_method", "GLRT").upper()
        self.ws = int(ins_cfg.get("static_window_size", 20))
        if self.ws < 2:
            self.ws = 2
        self.sig_g = float(ins_cfg.get("static_sig_gyro", 0.1))      # rad/s
        self.sig_a = float(ins_cfg.get("static_sig_accl", 0.1))      # m/s²
        self.gamma = {
            "GLRT": float(ins_cfg.get("static_gamma_glrt", 100.0)),
            "MV":   float(ins_cfg.get("static_gamma_mv",   50.0)),
            "MAG":  float(ins_cfg.get("static_gamma_mag",  50.0)),
            "ARE":  float(ins_cfg.get("static_gamma_are",  50.0)),
        }
        self._buf: Deque[ImuMeasurement] = deque(maxlen=self.ws)

    @property
    def window_size(self) -> int:
        return self.ws

    @property
    def buffer_count(self) -> int:
        return len(self._buf)

    def push(self, imu: ImuMeasurement) -> None:
        """压入一个 IMU 测量到滑动窗口。"""
        self._buf.append(imu)

    def detect(self, pos_e: np.ndarray) -> bool:
        """对当前窗口执行静态检测。

        Args:
            pos_e: ECEF 位置 [3], 用于计算局部重力大小 (GLRT/MAG 需要)

        Returns:
            True=静态, False=运动; 窗口未填满时返回 False
        """
        n = len(self._buf)
        if n < self.ws:
            return False

        imus: List[ImuMeasurement] = list(self._buf)
        accel = np.array([imu.rate_view().accel for imu in imus], dtype=np.float64)  # [n, 3]
        gyro = np.array([imu.rate_view().gyro for imu in imus], dtype=np.float64)    # [n, 3]

        if self.method == "GLRT":
            return self._glrt(accel, gyro, pos_e)
        if self.method == "MV":
            return self._mv(accel)
        if self.method == "MAG":
            return self._mag(accel, pos_e)
        if self.method == "ARE":
            return self._are(gyro)
        if self.method == "ALL":
            return (self._glrt(accel, gyro, pos_e) and
                    self._mv(accel) and
                    self._mag(accel, pos_e) and
                    self._are(gyro))
        # 默认 GLRT
        return self._glrt(accel, gyro, pos_e)

    def _glrt(self, accel: np.ndarray, gyro: np.ndarray,
              pos_e: np.ndarray) -> bool:
        """广义似然比检验 (detstatic_GLRT)。

        T = (1/n) * Σ [ ‖ω‖²/σ_g² + ‖a - (‖g‖/‖ā‖)·ā‖²/σ_a² ]
        """
        n = accel.shape[0]
        ym = accel.mean(axis=0)                          # [3] 窗口加速度均值
        norm_ym = float(np.linalg.norm(ym))
        if norm_ym < 1e-9:
            return False
        g_mag = float(np.linalg.norm(gravity_ecef(pos_e)))
        scale = g_mag / norm_ym
        tmp = accel - scale * ym                         # [n, 3]
        gyro_sq = np.sum(gyro * gyro, axis=1)            # ‖ω‖²
        tmp_sq = np.sum(tmp * tmp, axis=1)               # ‖tmp‖²
        T = float(np.sum(gyro_sq / (self.sig_g ** 2) +
                         tmp_sq / (self.sig_a ** 2)) / n)
        return T < self.gamma["GLRT"]

    def _mv(self, accel: np.ndarray) -> bool:
        """加速度滑动方差 (detstatic_MV)。

        T = Σ ‖a - ā‖² / (σ_a² · n)
        """
        n = accel.shape[0]
        ym = accel.mean(axis=0)
        tmp = accel - ym                                 # [n, 3]
        tmp_sq = np.sum(tmp * tmp, axis=1)
        T = float(np.sum(tmp_sq) / (self.sig_a ** 2 * n))
        return T < self.gamma["MV"]

    def _mag(self, accel: np.ndarray, pos_e: np.ndarray) -> bool:
        """加速度模值 (detstatic_MAG)。

        T = Σ (‖g‖ - ‖a‖)² / (σ_a² · n)
        """
        n = accel.shape[0]
        g_mag = float(np.linalg.norm(gravity_ecef(pos_e)))
        accel_mag = np.linalg.norm(accel, axis=1)        # ‖a‖ per sample
        diff = g_mag - accel_mag
        T = float(np.sum(diff * diff) / (self.sig_a ** 2 * n))
        return T < self.gamma["MAG"]

    def _are(self, gyro: np.ndarray) -> bool:
        """角速率能量 (detstatic_ARE)。

        T = Σ ‖ω‖² / (σ_g² · n)
        """
        n = gyro.shape[0]
        gyro_sq = np.sum(gyro * gyro, axis=1)
        T = float(np.sum(gyro_sq) / (self.sig_g ** 2 * n))
        return T < self.gamma["ARE"]
