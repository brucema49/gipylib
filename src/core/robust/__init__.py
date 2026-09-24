"""基于残差的抗差估计。

接口与配置见 skills/robust.md。权因子语义统一为 **R 的膨胀因子**:

    R_new[i, i] = R[i, i] * rfact[i],   rfact[i] >= 1

`rfact = 1` 表示不调整，`rfact >> 1` 等效剔除。注意这与"观测权"
(`w = 1/rfact`) 相反，实现时不要混淆 —— robust.md 早期草稿里 Huber 段
写的是观测权 `c/|u|`，接入 `adjust_R` 时必须用膨胀因子 `|u|/c`。
"""
from src.core.robust.robust_estimator import RobustEstimator
from src.core.robust.weight_function import (
    HuberWeight,
    IGG3Weight,
    WeightFunction,
)

__all__ = [
    "RobustEstimator",
    "WeightFunction",
    "IGG3Weight",
    "HuberWeight",
]
