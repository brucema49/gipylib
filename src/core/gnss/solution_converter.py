"""rtklib-py Sol → 本项目 GnssSolution 转换器。"""
from typing import Optional

import numpy as np

from src.core.data_types import GnssSolution

SOLQ_NONE = 0
SOLQ_FIX = 1
SOLQ_FLOAT = 2
SOLQ_DGPS = 4
SOLQ_SINGLE = 5


def sol_to_gnss_solution(sol) -> Optional[GnssSolution]:
    """把 rtklib-py Sol 对象转换为本项目 GnssSolution。

    Returns:
        GnssSolution 或 None（解算失败 stat==SOLQ_NONE 时）
    """
    if sol.stat == SOLQ_NONE:
        return None

    SECONDS_PER_WEEK = 604800
    total_sec = sol.t.time + sol.t.sec
    week = int(total_sec // SECONDS_PER_WEEK)
    sow = total_sec - week * SECONDS_PER_WEEK

    position = np.array(sol.rr[0:3], dtype=float)
    sd = np.sqrt(np.abs(np.diag(sol.qr[0:3, 0:3])))

    return GnssSolution(
        timestamp=sow,
        week=week,
        position=position,
        quality=int(sol.stat),
        num_sv=int(sol.ns),
        sd=sd,
    )
