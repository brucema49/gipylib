"""gnutlib — LibGnut（GREAT-MSF/src/LibGnut）频点映射与观测值读取的 Python 移植。

处理逻辑以 LibGnut 为第一权威（LibGnut first），输出结构兼容 pyrinrx
（rtklib-py）风格。三个子模块：

- :mod:`gnss`    — GOBS 枚举、系统/波段/属性优先级静态表（gnss.h + gnss.cpp）
- :mod:`gsys`    — 频率数值表与波段↔序号转换（gsys.h + gsys.cpp + gobsgnss）
- :mod:`rinexo3` — RINEX 3 观测头/数据读取（rinexo3.cpp + rinexo2.cpp）
"""

from .gnss import (
    GOBSATTR,
    GOBSBAND,
    GOBSTYPE,
    GNSS_BAND_PRIORITY,
    GNSS_BAND_SORTED,
    GNSS_DATA_PRIORITY,
    GNSS_FREQ_PRIORITY,
    GNSS_SATS,
    GSYS,
    PHASE_ORDER_ATTR_RAW,
    RANGE_ORDER_ATTR_RAW,
    attr_priority,
    band_priority,
    select_attr,
    band2freq_seq,
    char2gobsattr,
    char2gobsband,
    char2gobstype,
    freq_priority,
    gobs2band,
    gobs2str,
    gobs_code,
    gobs_doppler,
    gobs_phase,
    gobs_snr,
    gobsattr2str,
    gobsband2str,
    gobstype2str,
    pha2snr,
    pl2snr,
    sort_band,
    str2gobs,
    str2gobsattr,
    str2gobsband,
    str2gobstype,
    tba2gobs,
)
from .gsys import (
    CLIGHT,
    frequency,
    glo_l1_frequency,
    glo_l2_frequency,
    wavelength,
    wavelength_L3,
    wavelength_NL,
    wavelength_WL,
)
from .rinexo3 import (
    EpochObs,
    RinexObsHeader,
    SatelliteObs,
    band_frequency_hz,
    band_slot_view,
    declared_bands,
    fix_band,
    iter_obs_epochs,
    parse_obs_header,
)

__all__ = [
    # gnss.py
    "GSYS", "GOBSTYPE", "GOBSBAND", "GOBSATTR",
    "GNSS_BAND_PRIORITY", "GNSS_FREQ_PRIORITY", "GNSS_DATA_PRIORITY",
    "GNSS_BAND_SORTED", "GNSS_SATS",
    "RANGE_ORDER_ATTR_RAW", "PHASE_ORDER_ATTR_RAW", "select_attr",
    "str2gobs", "gobs2str", "tba2gobs",
    "char2gobstype", "gobstype2str", "char2gobsband", "str2gobsband",
    "gobsband2str", "char2gobsattr", "str2gobsattr", "gobsattr2str",
    "gobs2band", "gobs_code", "gobs_phase", "gobs_doppler", "gobs_snr",
    "pha2snr", "pl2snr",
    "band_priority", "freq_priority", "band2freq_seq", "attr_priority",
    "sort_band",
    # gsys.py
    "CLIGHT", "frequency", "wavelength",
    "wavelength_L3", "wavelength_WL", "wavelength_NL",
    "glo_l1_frequency", "glo_l2_frequency",
    # rinexo3.py
    "RinexObsHeader", "EpochObs", "SatelliteObs",
    "parse_obs_header", "iter_obs_epochs", "band_slot_view",
    "declared_bands", "fix_band", "band_frequency_hz",
]
