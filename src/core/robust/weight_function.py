"""抗差权函数。

输入标准化残差 `std_res = |v_i| / sqrt(Q_ii)`，`Q = H P Hᵀ + R`；
输出 **R 的膨胀因子** `rfact`（>= 1，越大越降权）。

参考:
- GINav ``ddres_rtkins`` IGG-3 段 (L181-215)
- Yang YuanXi, IGG-3 模型
"""
from abc import ABC, abstractmethod

import numpy as np


class WeightFunction(ABC):
    """权函数抽象基类。

    输入: 标准化残差 std_res (n_obs,)  (非负)
    输出: 膨胀因子 rfact (n_obs,)
        - rfact = 1      : 正常观测, 不调整
        - rfact > 1      : 降权观测 (可疑)
        - rfact = 很大   : 等效剔除
    """

    @abstractmethod
    def compute(self, std_res: np.ndarray) -> np.ndarray:
        """计算膨胀因子。"""
        ...


class IGG3Weight(WeightFunction):
    """IGG-3 权函数 (三段)。

    参考 GINav ddres_rtkins:

      |u| <= c0              :  rfact = 1
      c0 < |u| <= c1         :  rfact = (|u|/c0) · ((c1−c0)/(c1−|u|))²
      |u| >  c1              :  rfact = REJECT_FACTOR (等效剔除)

    在 |u| = c0 处连续 (=1)，|u| → c1⁻ 时单调发散到剔除。
    """

    REJECT_FACTOR = 1.0e6

    def __init__(self, c0: float = 2.0, c1: float = 5.0):
        if not 0.0 < c0 < c1:
            raise ValueError("IGG3 requires 0 < c0 < c1")
        self.c0 = float(c0)
        self.c1 = float(c1)

    def compute(self, std_res: np.ndarray) -> np.ndarray:
        u = np.abs(np.asarray(std_res, dtype=np.float64))
        rfact = np.ones_like(u)
        mid = (u > self.c0) & (u <= self.c1)
        if np.any(mid):
            um = u[mid]
            ratio = (self.c1 - self.c0) / np.maximum(self.c1 - um, 1e-12)
            rfact[mid] = (um / self.c0) * ratio * ratio
        rfact[u > self.c1] = self.REJECT_FACTOR
        return rfact


class HuberWeight(WeightFunction):
    """Huber 权函数 (两段)。

    Huber 的观测权为 `w = 1` (|u| <= c) 或 `w = c/|u|` (|u| > c)。
    换算成 `R` 的膨胀因子 `rfact = 1/w`:

      |u| <= c   :  rfact = 1
      |u| >  c   :  rfact = |u| / c
    """

    def __init__(self, c: float = 2.0):
        if c <= 0.0:
            raise ValueError("Huber threshold c must be positive")
        self.c = float(c)

    def compute(self, std_res: np.ndarray) -> np.ndarray:
        u = np.abs(np.asarray(std_res, dtype=np.float64))
        return np.maximum(1.0, u / self.c)
