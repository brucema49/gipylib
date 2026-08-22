"""紧组合模糊度固定 (调 mlambda, 不操作全局 nav.x/P)。

策略 (参考 rtklib manage_amb_LAMBDA + fix-and-hold):
  - posvar (P 对角均值) > thresar_var → 跳过 (thresar1 逻辑)
  - mlambda 搜索, ratio = s[1]/s[0]
  - ratio > thresar → 接受固定解
  - 连续 hold_count 历元固定解一致 → hold (锁定)
  - 降级/重启时 reset()
"""
import numpy as np

from src.core.gnss.rtklib.mlambda import mlambda


class TcAmbiguity:
    """RTK-INS 模糊度管理器。

    直接调 mlambda(a, Q, m=2) 做整数搜索,
    不依赖 rtklib 全局 nav.x/P 状态。
    """

    def __init__(self, thresar: float = 3.0, thresar_var: float = 0.5,
                 hold_count: int = 10):
        self.thresar = thresar
        self.thresar_var = thresar_var
        self.hold_threshold = hold_count
        self._last_fixed = None
        self._hold_count = 0
        self._is_holding = False

    def try_fix(self, x_amb: np.ndarray, P_amb: np.ndarray,
                posvar: float = 0.0):
        """尝试 LAMBDA 固定。

        Args:
            x_amb: 浮点模糊度 (n,)
            P_amb: 模糊度协方差 (n, n)
            posvar: 位置方差 (P[pos,pos] 对角均值), 超阈值时跳过 AR
                    (rtklib thresar1 逻辑: 检查位置方差而非模糊度方差)

        Returns:
            (fixed[n], ratio, ok: bool)
            fixed: 整数解 (ok=False 时为空数组)
            ratio: s[1]/s[0] (ok=False 时仍返回 ratio 供诊断)
            ok: 是否成功固定
        """
        n = len(x_amb)
        if n == 0:
            return np.array([]), 0.0, False
        # 位置方差过大跳过 AR (rtklib thresar1 逻辑)
        # 注: 检查位置方差而非模糊度方差, 与 rtklib manage_amb_LAMBDA 一致
        if posvar > self.thresar_var:
            return np.array([]), 0.0, False
        # 数值防护: TC 估计器的 P_amb 子块实测存在非对称/半负定 (约30%历元,
        # 见 issue/8-22 第12节), 对称化 + 按谱下界加微扰, 保证 LD 分解可解。
        P = np.array(P_amb, dtype=np.float64)
        P = 0.5 * (P + P.T)
        eig_min = float(np.min(np.linalg.eigvalsh(P)))
        if eig_min <= 1e-9:
            jitter = max(1e-6, 1e-9 * float(np.trace(P)) / n)
            P += np.eye(n) * (jitter - min(eig_min, 0.0))
        try:
            afix, s = mlambda(x_amb, P, m=2)
        except np.linalg.LinAlgError:
            return np.array([]), 0.0, False
        ratio = float(s[1] / s[0]) if s[0] > 1e-12 else 0.0
        if ratio < self.thresar:
            return np.array([]), ratio, False
        fixed = afix[:, 0].copy()
        # hold 逻辑 (fix-and-hold)
        if self._last_fixed is not None and np.array_equal(fixed, self._last_fixed):
            self._hold_count += 1
            if self._hold_count >= self.hold_threshold:
                self._is_holding = True
        else:
            self._last_fixed = fixed.copy()
            self._hold_count = 1
        return fixed, ratio, True

    @property
    def is_holding(self) -> bool:
        return self._is_holding

    def reset(self):
        """降级/重启时清空。"""
        self._last_fixed = None
        self._hold_count = 0
        self._is_holding = False

    def apply_correction(self, x: np.ndarray, fixed: np.ndarray,
                         amb_slice: slice):
        """固定后将浮点解替换为整数解。"""
        x[amb_slice] = fixed
