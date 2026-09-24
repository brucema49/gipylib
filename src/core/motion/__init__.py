"""运动约束层（架构与调度）。

分层约定:
  - ``src/core/motion/``            : 调度 / decimation / 残差输出 / 抗差接入（本包）
  - ``src/core/ins/constraints.py`` : 约束算法本身 (guards + Z/H/R 构造)
  - ``src/core/ins/static_detect.py``: 零速检测算法

算法权威性在 skills/NHC_ZUPT.md，架构说明在 skills/motion.md。
"""
from src.core.motion.decimation_counter import DecimationCounter
from src.core.motion.motion_manager import MotionConstraintManager
from src.core.motion.motion_residuals import (
    ConstraintResidualLog,
    ConstraintResidualSnapshot,
)

__all__ = [
    "DecimationCounter",
    "MotionConstraintManager",
    "ConstraintResidualLog",
    "ConstraintResidualSnapshot",
]
