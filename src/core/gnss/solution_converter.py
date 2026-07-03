"""rtklib-py Sol → 本项目 GnssSolution 转换器。"""
from typing import Optional

import numpy as np

from src.core.data_types import GnssSolution
from src.core.time_utils import unix_to_gpst

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

    # rtklib-py 的 gtime_t.time 是 Unix 秒（从 1970-01-01 起算），
    # gtime_t.sec 是不足秒的小数部分。timestamp 直接使用 Unix 时间戳。
    unix_ts = sol.t.time + sol.t.sec
    week, _ = unix_to_gpst(unix_ts)

    position = np.array(sol.rr[0:3], dtype=float)
    qr3 = np.array(sol.qr[0:3, 0:3], dtype=float)
    sd = np.sqrt(np.abs(np.diag(qr3)))

    return GnssSolution(
        timestamp=unix_ts,
        week=week,
        position=position,
        quality=int(sol.stat),
        num_sv=int(sol.ns),
        sd=sd,
        cov=qr3,
    )
