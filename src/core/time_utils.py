"""GPST 时间转换工具。

GPST 起点：1980-01-06 00:00:00 UTC（GPS 周 0，周内秒 0）。
一周 = 604800 秒。
"""
from datetime import datetime, timedelta

GPS_EPOCH = datetime(1980, 1, 6)
SECONDS_PER_WEEK = 604800


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
