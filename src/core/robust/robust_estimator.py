"""抗差估计入口。

参考 skills/robust.md §3。输入先验残差快照，输出调整后的 `R`。
默认关闭；关闭时 `adjust_R` 是纯透传（不修改输入），主滤波行为不变。
"""
import logging

import numpy as np

from src.core.robust.weight_function import HuberWeight, IGG3Weight

logger = logging.getLogger(__name__)

_DEFAULT_SOURCES = ("gnss", "nhc", "zupt", "zaru")


class RobustEstimator:
    """抗差估计器。

    标准化残差:  std_res_i = |v_i| / sqrt(Q_ii),  Q = H P Hᵀ + R
    R 调整:      R_new[i,i] = R[i,i] · rfact_i

    配置 (config.yaml 顶层 `robust` 段, 见 skills/robust.md §6):
        enabled   : 总开关 (默认 false)
        method    : "igg3" / "huber"
        c0, c1    : IGG-3 参数
        c         : Huber 参数
        sources   : 参与抗差的量测来源列表 (默认 gnss/nhc/zupt/zaru)
    """

    def __init__(self, config: dict):
        cfg = (config or {}).get("robust", {}) or {}
        self.enabled = bool(cfg.get("enabled", False))
        self.method = str(cfg.get("method", "igg3")).lower()
        sources = cfg.get("sources", list(_DEFAULT_SOURCES))
        self.sources = set(str(s) for s in sources)
        self.log_adjustments = bool(cfg.get("log_adjustments", False))
        self._adjust_count = 0
        self._reject_count = 0

        if self.method == "igg3":
            self._weight_fn = IGG3Weight(
                c0=float(cfg.get("c0", 2.0)),
                c1=float(cfg.get("c1", 5.0)),
            )
        elif self.method == "huber":
            self._weight_fn = HuberWeight(c=float(cfg.get("c", 2.0)))
        else:
            raise ValueError(f"unknown robust method: {self.method!r}")

    @property
    def weight_function(self):
        return self._weight_fn

    @property
    def stats(self) -> dict:
        return {"adjusted": self._adjust_count, "rejected": self._reject_count}

    def applies_to(self, source: str) -> bool:
        return self.enabled and source in self.sources

    def adjust_R(self, v_prior: np.ndarray, H: np.ndarray, R: np.ndarray,
                 P: np.ndarray, source: str = "gnss") -> np.ndarray:
        """根据先验残差调整 R 矩阵（纯函数语义，不修改入参）。

        Args:
            v_prior: 先验残差/新息 (n_obs,)
            H: 设计矩阵 (n_obs, dim)
            R: 原始量测噪声协方差 (n_obs, n_obs)
            P: 先验状态协方差 (dim, dim)
            source: 量测来源 ("gnss" / "nhc" / "zupt" / "zaru")

        Returns:
            R_new (n_obs, n_obs)。未启用或来源不匹配时原样返回。
        """
        if not self.applies_to(source):
            return R
        v = np.asarray(v_prior, dtype=np.float64)
        H = np.asarray(H, dtype=np.float64)
        R = np.asarray(R, dtype=np.float64)
        P = np.asarray(P, dtype=np.float64)

        Q = H @ P @ H.T + R
        diag_Q = np.diag(Q)
        if diag_Q.size != v.size:
            raise ValueError("adjust_R: dimension mismatch between v and H/R")
        std_res = np.abs(v) / np.sqrt(np.diag(Q) + 1e-12)

        rfact = self._weight_fn.compute(std_res)
        R_new = R.copy()
        idx = np.diag_indices(R_new.shape[0])
        R_new[idx] = R[idx] * rfact
        R_new = 0.5 * (R_new + R_new.T)

        self._adjust_count += 1
        if np.any(rfact >= IGG3Weight.REJECT_FACTOR):
            self._reject_count += 1
            if self.log_adjustments:
                logger.info("robust[%s]: reject std_res=%s", source,
                            np.array2string(std_res, precision=2))
        elif self.log_adjustments and np.any(rfact > 1.0):
            logger.debug("robust[%s]: std_res=%s rfact=%s", source,
                         np.array2string(std_res, precision=2),
                         np.array2string(rfact, precision=2))
        return R_new


def build_robust_estimator(config: dict):
    """工厂: 未配置 `robust` 段或不启用时返回 None，避免主滤波多一层判断。"""
    cfg = (config or {}).get("robust", {}) or {}
    if not cfg:
        return None
    est = RobustEstimator(config)
    return est if est.enabled else None
