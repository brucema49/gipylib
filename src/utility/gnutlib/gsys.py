"""LibGnut 频率数值移植（GREAT-MSF/src/LibGnut/gutils/gsys.h + gsys.cpp +
gdata/gobsgnss.cpp t_gobsgnss::frequency / wavelength）。

所有频率由基准频率 10.23 MHz（GLONASS FDMA 为 178 MHz）乘以乘数因子导出，
与 C++ 逐常量对应；BDS-2/3 全部频点（B1I/B2I/B3I/B2a/B2b/B2/B1C）齐备。
"""

from .gnss import GOBSBAND, GSYS

CLIGHT = 2.99792458e8  # m/s（gutils/gconst.h）

# ---------------------------------------------------------------------------
# 基准频率与乘数因子（gsys.h 44-127 行）
# ---------------------------------------------------------------------------

GPS_FRQ = 10.23e6   # 非 Glowide 系统基准 10.23 MHz
GLO_FRQ = 178.0e6   # GLONASS FDMA 基准 178 MHz

# 乘数因子（gsys.h #define *_MLT）
MLT = {
    "G01": 154.0, "G02": 120.0, "G05": 115.0,
    "R01": 9.0, "R02": 7.0,
    "R01_CDMA": 9.0, "R02_CDMA": 7.0, "R03_CDMA": 117.5, "R05_CDMA": 115.5,
    "E01": 154.0, "E05": 115.0, "E06": 125.0, "E07": 118.0, "E08": 116.5,
    "C02": 152.6, "C06": 124.0, "C07": 118.0, "C01": 154.0,
    "C08": 116.5, "C05": 115.0, "C09": 118.0,
    "J01": 154.0, "J02": 120.0, "J05": 115.0, "J06": 125.0,
    "S01": 154.0, "S05": 115.0,
    "I05": 115.0,
}

# GLONASS FDMA 通道间隔（gsys.h R01_STEP / R02_STEP）
R01_STEP = 0.5625e6
R02_STEP = 0.4375e6


def _f(multiplier: float) -> float:
    return multiplier * GPS_FRQ

# 通道号 0 的标称频率（Hz）
G01_F = _f(MLT["G01"])          # 1575.42e6
G02_F = _f(MLT["G02"])          # 1227.60e6
G05_F = _f(MLT["G05"])          # 1176.45e6
R01_F0 = MLT["R01"] * GLO_FRQ   # 1602.00e6（+k*R01_STEP）
R02_F0 = MLT["R02"] * GLO_FRQ   # 1246.00e6（+k*R02_STEP）
R03_F_CDMA = _f(MLT["R03_CDMA"])  # 1202.025e6
R05_F_CDMA = _f(MLT["R05_CDMA"])  # 1181.565e6
E01_F = _f(MLT["E01"])          # 1575.42e6
E05_F = _f(MLT["E05"])          # 1176.45e6
E06_F = _f(MLT["E06"])          # 1278.75e6
E07_F = _f(MLT["E07"])          # 1207.14e6
E08_F = _f(MLT["E08"])          # 1191.795e6
C02_F = _f(MLT["C02"])          # 1561.098e6  B1I
C06_F = _f(MLT["C06"])          # 1268.52e6   B3I
C07_F = _f(MLT["C07"])          # 1207.14e6   B2I
C01_F = _f(MLT["C01"])          # 1575.42e6   B1C
C05_F = _f(MLT["C05"])          # 1176.45e6   B2a
C08_F = _f(MLT["C08"])          # 1191.795e6  B2 (B1C+B2a)
C09_F = _f(MLT["C09"])          # 1207.14e6   B2b
J01_F = _f(MLT["J01"])
J02_F = _f(MLT["J02"])
J05_F = _f(MLT["J05"])
J06_F = _f(MLT["J06"])
S01_F = _f(MLT["S01"])
S05_F = _f(MLT["S05"])
I05_F = _f(MLT["I05"])


def glo_l1_frequency(channel: int = 0) -> float:
    """GLONASS L1 频率（通道号 k）：1602 + 0.5625k MHz。"""
    return R01_F0 + R01_STEP * float(channel)


def glo_l2_frequency(channel: int = 0) -> float:
    """GLONASS L2 频率（通道号 k）：1246 + 0.4375k MHz。"""
    return R02_F0 + R02_STEP * float(channel)

# ---------------------------------------------------------------------------
# band/freq 序号与 GFRQ 转换（gsys.cpp 38-78、162-195、382-395）
# ---------------------------------------------------------------------------

def band2gfrq(gs: GSYS, band: GOBSBAND) -> int:
    """GOBSBAND → GFRQ 标识（gsys.cpp band2gfrq；未声明返回 LAST_GFRQ=999）。"""
    from .gnss import band2freq_seq, freq_priority
    iseq = band2freq_seq(gs, band)
    if iseq == 999:
        return 999
    return freq_priority(gs, iseq)


def gfrq2band(gs: GSYS, gfreq: int) -> GOBSBAND:
    """GFRQ 标识 → GOBSBAND（gsys.cpp gfrq2band；未声明返回 BAND=999）。"""
    from .gnss import GNSS_FREQ_PRIORITY, band_priority
    seq = GNSS_FREQ_PRIORITY.get(GSYS(gs), [])
    for i, g in enumerate(seq):
        if g == int(gfreq):
            return band_priority(gs, i)
    return GOBSBAND.BAND


def frequency(gs: GSYS, band: GOBSBAND, channel: int = 0) -> float:
    """(系统, 波段) → 载波频率 [Hz]（gobsgnss.cpp t_gobsgnss::frequency 移植）。

    GLONASS FDMA 的 BAND_1/BAND_2 按``channel``（通道号）加通道间隔。
    未声明波段返回 0.0。
    """
    gs = GSYS(gs)
    band = GOBSBAND(band)
    if gs == GSYS.GPS:
        return {GOBSBAND.BAND_1: G01_F, GOBSBAND.BAND_2: G02_F,
                GOBSBAND.BAND_5: G05_F}.get(band, 0.0)
    if gs == GSYS.GLO:
        if band == GOBSBAND.BAND_1:
            return glo_l1_frequency(channel)
        if band == GOBSBAND.BAND_2:
            return glo_l2_frequency(channel)
        if band == GOBSBAND.BAND_3:
            return R03_F_CDMA
        if band == GOBSBAND.BAND_5:
            return R05_F_CDMA
        return 0.0
    if gs == GSYS.GAL:
        return {GOBSBAND.BAND_1: E01_F, GOBSBAND.BAND_5: E05_F,
                GOBSBAND.BAND_7: E07_F, GOBSBAND.BAND_8: E08_F,
                GOBSBAND.BAND_6: E06_F}.get(band, 0.0)
    if gs == GSYS.BDS:
        return {GOBSBAND.BAND_2: C02_F, GOBSBAND.BAND_6: C06_F,
                GOBSBAND.BAND_7: C07_F, GOBSBAND.BAND_5: C05_F,
                GOBSBAND.BAND_8: C08_F, GOBSBAND.BAND_9: C09_F,
                GOBSBAND.BAND_1: C01_F}.get(band, 0.0)
    if gs == GSYS.QZS:
        return {GOBSBAND.BAND_1: J01_F, GOBSBAND.BAND_2: J02_F,
                GOBSBAND.BAND_5: J05_F, GOBSBAND.BAND_6: J06_F}.get(band, 0.0)
    if gs == GSYS.SBS:
        return {GOBSBAND.BAND_1: S01_F, GOBSBAND.BAND_5: S05_F}.get(band, 0.0)
    if gs == GSYS.IRN:
        return {GOBSBAND.BAND_5: I05_F}.get(band, 0.0)
    return 0.0


def wavelength(gs: GSYS, band: GOBSBAND, channel: int = 0) -> float:
    """(系统, 波段) → 载波波长 [m]（gobsgnss.cpp wavelength）。"""
    freq = frequency(gs, band, channel)
    return CLIGHT / freq if freq > 0 else 0.0


def wavelength_L3(gs: GSYS, band1: GOBSBAND, band2: GOBSBAND,
                  channel: int = 0) -> float:
    """双频无电离层组合等效波长（gobsgnss.cpp wavelength_L3）。"""
    f1 = frequency(gs, band1, channel)
    f2 = frequency(gs, band2, channel)
    if f1 <= 0 or f2 <= 0 or f1 == f2:
        return 0.0
    c1 = f1 ** 2 / (f1 ** 2 - f2 ** 2)
    c2 = -f2 ** 2 / (f1 ** 2 - f2 ** 2)
    return c1 * (CLIGHT / f1) + c2 * (CLIGHT / f2)


def wavelength_WL(gs: GSYS, band1: GOBSBAND, band2: GOBSBAND,
                  channel: int = 0) -> float:
    """宽巷等效波长 c/(f1-f2)（gobsgnss.cpp wavelength_WL）。"""
    f1 = frequency(gs, band1, channel)
    f2 = frequency(gs, band2, channel)
    if f1 <= 0 or f2 <= 0 or f1 == f2:
        return 0.0
    return CLIGHT / (f1 - f2)


def wavelength_NL(gs: GSYS, band1: GOBSBAND, band2: GOBSBAND,
                  channel: int = 0) -> float:
    """窄巷等效波长 c/(f1+f2)（gobsgnss.cpp wavelength_NL）。"""
    f1 = frequency(gs, band1, channel)
    f2 = frequency(gs, band2, channel)
    if f1 <= 0 or f2 <= 0:
        return 0.0
    return CLIGHT / (f1 + f2)
