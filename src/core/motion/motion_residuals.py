"""约束伪量测的残差输出。

字段与 skills/紧组合.md §17 的 ``ResidualSnapshot`` 对齐，因此这些快照可以
直接喂给 ``src/core/robust/`` 的 ``RobustEstimator.adjust_R``，也可以在
``TcResiduals`` 落地后按同一 schema 合并。

先验残差 (新息)
    v_prior = Z − H·x_prior
标准化残差
    std_res_i = |v_prior_i| / sqrt(Q_ii),  Q = H P_prior Hᵀ + R
后验残差
    v_post = v_prior − H·dx,   dx = x_posterior − x_prior (闭环前)
"""
from __future__ import annotations

import json
import logging
from dataclasses import asdict, dataclass, field
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

CONSTRAINT_SOURCES = ("nhc", "zupt", "zaru")


@dataclass
class ConstraintResidualSnapshot:
    """单次约束量测更新的残差快照。"""

    timestamp: float
    source: str                             # "nhc" / "zupt" / "zaru"
    v_prior: np.ndarray                     # 先验残差/新息 (n,)
    H: np.ndarray                           # 设计矩阵 (n, dim)
    R: np.ndarray                           # 原始量测噪声 (n, n)
    P_prior: np.ndarray                     # 更新前状态协方差
    x_prior: np.ndarray                     # 更新前误差状态
    v_posterior: np.ndarray                 # 后验残差 (n,)
    P_posterior: np.ndarray                 # 更新后状态协方差
    x_posterior: np.ndarray                 # 更新后误差状态
    R_used: Optional[np.ndarray] = None     # 抗差调整后实际使用的 R
    std_res: Optional[np.ndarray] = None    # 标准化残差
    info: dict = field(default_factory=dict)

    def as_record(self) -> dict:
        """转成可 JSON 序列化的字典（数组降为 list）。"""
        def enc(a):
            return None if a is None else np.asarray(a).tolist()

        return {
            "timestamp": float(self.timestamp),
            "source": self.source,
            "v_prior": enc(self.v_prior),
            "std_res": enc(self.std_res),
            "v_posterior": enc(self.v_posterior),
            "info": dict(self.info),
        }


class ConstraintResidualLog:
    """约束残差记录器（默认关闭，开销为零）。

    配置 (``ins`` 段):
        constraint_residual_enable       : 0/1（默认 0）
        constraint_residual_max_history  : 每来源保留的最大条数（默认 1000）
        constraint_residual_dump         : 可选 jsonl 输出路径
    """

    def __init__(self, config: dict):
        ins_cfg = (config or {}).get("ins", {}) or {}
        self.enabled = bool(int(ins_cfg.get("constraint_residual_enable", 0)))
        self.max_history = max(1, int(
            ins_cfg.get("constraint_residual_max_history", 1000)))
        self.dump_path = str(ins_cfg.get("constraint_residual_dump", "") or "")
        self._history: dict[str, list] = {}
        self._latest: Optional[ConstraintResidualSnapshot] = None
        self._dump = None

    # ---- 生命周期 ----

    def open(self) -> None:
        if self.enabled and self.dump_path:
            try:
                self._dump = open(self.dump_path, "w", encoding="utf-8")
            except OSError as exc:  # pragma: no cover - 环境相关
                logger.warning("constraint residual dump unavailable: %s", exc)
                self._dump = None

    def close(self) -> None:
        if self._dump is not None:
            self._dump.close()
            self._dump = None

    # ---- 记录 ----

    def record(self, snapshot: ConstraintResidualSnapshot) -> None:
        if not self.enabled:
            return
        self._latest = snapshot
        bucket = self._history.setdefault(snapshot.source, [])
        bucket.append(snapshot)
        if len(bucket) > self.max_history:
            del bucket[: len(bucket) - self.max_history]
        if self._dump is not None:
            self._dump.write(json.dumps(snapshot.as_record()) + "\n")

    # ---- 查询 ----

    @property
    def latest(self) -> Optional[ConstraintResidualSnapshot]:
        return self._latest

    def history(self, source: Optional[str] = None) -> list:
        if source is not None:
            return list(self._history.get(source, []))
        out = []
        for src in CONSTRAINT_SOURCES:
            out.extend(self._history.get(src, []))
        return out

    def counts(self) -> dict:
        return {src: len(self._history.get(src, [])) for src in CONSTRAINT_SOURCES}

    def clear(self) -> None:
        self._history.clear()
        self._latest = None


def standardized_residual(v_prior: np.ndarray, H: np.ndarray,
                          R: np.ndarray, P: np.ndarray) -> np.ndarray:
    """标准化残差 ``|v_i| / sqrt(Q_ii)``, ``Q = H P Hᵀ + R``。"""
    Q = H @ P @ H.T + R
    return np.abs(np.asarray(v_prior)) / np.sqrt(np.diag(Q) + 1e-12)
