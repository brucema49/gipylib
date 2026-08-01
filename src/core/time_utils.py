"""时间转换工具。

本项目内部统一使用 Unix 时间戳（float 秒，从 1970-01-01 00:00:00 UTC 起算），
与 rtklib-py 的 gtime_t.time + gtime_t.sec 一致。

GPST 起点：1980-01-06 00:00:00 UTC（GPS 周 0，周内秒 0）= Unix 315964800。
一周 = 604800 秒。
"""
from datetime import datetime, timedelta
from typing import Tuple

GPS_EPOCH = datetime(1980, 1, 6)
SECONDS_PER_WEEK = 604800
GPST_EPOCH_UNIX = 315964800  # 1980-01-06 00:00:00 UTC 的 Unix 时间戳


def ymdhms_to_gpst(year: int, month: int, day: int,
                   hour: int, minute: int, second: float):
    dt = datetime(year, month, day, hour, minute) + timedelta(seconds=second)
    delta = (dt - GPS_EPOCH).total_seconds()
    week = int(delta // SECONDS_PER_WEEK)
    sow = delta - week * SECONDS_PER_WEEK
    return week, sow


def sow_to_ymdhms(week: int, sow: float):
    dt = GPS_EPOCH + timedelta(seconds=week * SECONDS_PER_WEEK + sow)
    return (dt.year, dt.month, dt.day, dt.hour, dt.minute,
            dt.second + dt.microsecond * 1e-6)


def gpst_to_unix(week: int, sow: float) -> float:
    """GPS 周 + 周内秒 → Unix 时间戳（秒）。"""
    return GPST_EPOCH_UNIX + week * SECONDS_PER_WEEK + sow


def unix_to_gpst(unix_ts: float) -> Tuple[int, float]:
    """Unix 时间戳（秒）→ (GPS 周, 周内秒)。"""
    total = unix_ts - GPST_EPOCH_UNIX
    week = int(total // SECONDS_PER_WEEK)
    sow = total - week * SECONDS_PER_WEEK
    return week, sow
