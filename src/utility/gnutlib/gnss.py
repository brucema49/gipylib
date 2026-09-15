"""LibGnut 频点映射移植（GREAT-MSF/src/LibGnut/gutils/gnss.h + gnss.cpp）。

本模块是 LibGnut ``gutils/gnss`` 的 Python 移植：

- GSYS / GOBSTYPE / GOBSBAND / GOBSATTR / GOBS 枚举数值与 C++ 完全一致
  （GOBS 数值区间有语义：0-99 伪距 / 100-199 载波 / 200-299 多普勒 /
  300-399 信噪比 / 1000+ RINEX2 遗留码）；
- GNSS_BAND_PRIORITY / GNSS_FREQ_PRIORITY / GNSS_DATA_PRIORITY 三张静态表
  逐条对应 gnss.h:542-563 与 gnss.cpp:174-335；
- str2gobs / gobs2str / str2gobsband 等转换函数对应 gnss.cpp 337-530、
  1280-2143 行的 switch 逻辑（含 BDS-3 B2b 的 C7D/P/Z→C9D/P/Z 特判）。

与 gipylib 内部其他编号域（solver freq_ix、simplifier normalized band）
刻意保持正交；转换只在 src/stream/gnss_band_mapping.py 与
src/utility/rinex_improve.py 的边界完成。
"""

from collections.abc import Iterable
from enum import IntEnum


# ---------------------------------------------------------------------------
# 枚举（gnss.h 37-170 行）
# ---------------------------------------------------------------------------


class GSYS(IntEnum):
    """GNSS systems and augmentations (gnss.h GSYS)."""

    GPS = 1
    GAL = 2
    GLO = 3
    BDS = 4
    QZS = 5
    SBS = 6
    IRN = 7
    GNS = 999


class GOBSTYPE(IntEnum):
    """Observation type enumeration (gnss.h GOBSTYPE)."""

    TYPE_C = 1
    TYPE_L = 2
    TYPE_D = 3
    TYPE_S = 4
    TYPE_P = 101  # only for P-code!
    TYPE_SLR = 102
    TYPE_KBRRANGE = 103
    TYPE_LRIRANGE = 104
    TYPE_KBRRATE = 105
    TYPE_LRIRATE = 106
    TYPE = 999  # ""  UNKNOWN


class GOBSBAND(IntEnum):
    """Observation band enumeration (gnss.h GOBSBAND; BAND_9 is BDS-3 B2b)."""

    BAND_1 = 1
    BAND_2 = 2
    BAND_3 = 3
    BAND_5 = 5
    BAND_6 = 6
    BAND_7 = 7
    BAND_8 = 8
    BAND_9 = 9  # BDS-3 B2b
    BAND_A = 101
    BAND_B = 102
    BAND_C = 103
    BAND_D = 104
    BAND_SLR = 105
    BAND_KBR = 106
    BAND_LRI = 107
    BAND = 999  # ""  UNKNOWN


class GOBSATTR(IntEnum):
    """Observation tracking attribute enumeration (gnss.h GOBSATTR)."""

    ATTR_A = 1
    ATTR_B = 2
    ATTR_C = 3
    ATTR_D = 4
    ATTR_I = 5
    ATTR_L = 6
    ATTR_M = 7
    ATTR_N = 8
    ATTR_P = 9
    ATTR_Q = 10
    ATTR_S = 11
    ATTR_W = 12
    ATTR_X = 13
    ATTR_Y = 14
    ATTR_Z = 15
    ATTR_NULL = 16  # " " 2CHAR code
    ATTR = 999  # ""  UNKNOWN


GOBS_X = 9999  # gnss.h GOBS::X (LAST_GOBS / unknown)

# ---------------------------------------------------------------------------
# GOBS 三字符码表（gnss.h 173-462 行枚举的忠实映射）
# 每个频带的基址与属性偏移完全按 C++ 枚举数值复刻。
# ---------------------------------------------------------------------------

_BAND_BASE = {"1": 0, "2": 15, "3": 27, "5": 30, "6": 38, "7": 47, "8": 51, "9": 56}

_BAND_ATTRS = {
    "1": {"A": 0, "B": 1, "C": 2, "D": 3, "I": 4, "L": 5, "M": 6,
          "P": 8, "S": 9, "Q": 10, "W": 11, "X": 12, "Y": 13, "Z": 14},
    "2": {"C": 0, "D": 1, "I": 2, "L": 3, "M": 4, "P": 6, "S": 7,
          "Q": 8, "W": 9, "X": 10, "Y": 11},
    "3": {"I": 0, "Q": 1, "X": 2},
    "5": {"A": 0, "B": 1, "C": 2, "D": 3, "I": 4, "P": 5, "Q": 6, "X": 7},
    "6": {"A": 0, "B": 1, "C": 2, "I": 3, "L": 4, "S": 5, "Q": 6, "X": 7, "Z": 8},
    "7": {"I": 0, "Q": 1, "X": 3},
    "8": {"D": 0, "I": 1, "P": 2, "Q": 3, "X": 4},
    "9": {"D": 0, "P": 1, "Z": 2},  # BDS-3 B2b
}

_TYPE_BASE = {"C": 0, "L": 100, "D": 200, "S": 300}

# RINEX 2.x 遗留两字符码（gnss.h 414-460 行）
_LEGACY_2CHAR: dict[str, int] = {}
for _i, _c in enumerate("P1 P2 P5 C1 C2 C5 C6 C7 C8 CA CB CC CD".split()):
    _LEGACY_2CHAR[_c] = 1000 + _i
for _i, _c in enumerate("L1 L2 L5 L6 L7 L8 LA LB LC LD".split()):
    _LEGACY_2CHAR[_c] = 1100 + _i
for _i, _c in enumerate("D1 D2 D5 D6 D7 D8 DA DB DC DD".split()):
    _LEGACY_2CHAR[_c] = 1200 + _i
for _i, _c in enumerate("S1 S2 S5 S6 S7 S8 SA SB SC SD".split()):
    _LEGACY_2CHAR[_c] = 1300 + _i


def _build_gobs_table() -> dict[str, int]:
    """生成 3 字符观测码 → GOBS 枚举值表（含遗留两字符码）。"""
    table: dict[str, int] = dict(_LEGACY_2CHAR)
    for tchar, tbase in _TYPE_BASE.items():
        for band, base in _BAND_BASE.items():
            for attr, off in _BAND_ATTRS[band].items():
                table[f"{tchar}{band}{attr}"] = tbase + base + off
    return table


_GOBS_TABLE: dict[str, int] = _build_gobs_table()
_GOBS_REVERSE: dict[int, str] = {v: k for k, v in _GOBS_TABLE.items()}


def str2gobs(code: str) -> int:
    """观测码字符串 → GOBS 枚举值；未知码返回 GOBS_X（对应 str2gobs 默认分支）。"""
    code = (code or "").strip()
    # BDS-3 B2b 特判（gnss.cpp 1404-1409 等）：C7D/C7P/C7Z → C9D/C9P/C9Z
    if len(code) == 3 and code[1] == "7" and code[2] in "DPZ" and code[0] in "CLDS":
        code = code[0] + "9" + code[2]
    return _GOBS_TABLE.get(code, GOBS_X)


def gobs2str(gobs: int) -> str:
    """GOBS 枚举值 → 观测码字符串；未知返回空串。"""
    return _GOBS_REVERSE.get(int(gobs), "")


def tba2gobs(t: GOBSTYPE, b: GOBSBAND, a: GOBSATTR) -> int:
    """type/band/attr 三元组 → GOBS（对应 gnss.cpp tba2gobs 的字符串拼接）。"""
    return str2gobs(gobstype2str(t) + gobsband2str(b) + gobsattr2str(a))


def char2gobstype(c: str) -> GOBSTYPE:
    """观测类型字符 → GOBSTYPE（gnss.cpp 337-353）。"""
    return {
        "C": GOBSTYPE.TYPE_C, "L": GOBSTYPE.TYPE_L, "D": GOBSTYPE.TYPE_D,
        "S": GOBSTYPE.TYPE_S, "R": GOBSTYPE.TYPE_SLR, "P": GOBSTYPE.TYPE_P,
    }.get(c, GOBSTYPE.TYPE)


def gobstype2str(t: GOBSTYPE) -> str:
    """GOBSTYPE → 类型字符（gnss.cpp gobstype2str）。"""
    try:
        t = GOBSTYPE(t)
    except ValueError:
        return ""
    return {
        GOBSTYPE.TYPE_C: "C", GOBSTYPE.TYPE_L: "L", GOBSTYPE.TYPE_D: "D",
        GOBSTYPE.TYPE_S: "S", GOBSTYPE.TYPE_SLR: "R", GOBSTYPE.TYPE_P: "P",
    }.get(t, "")


def str2gobstype(code: str) -> GOBSTYPE:
    """观测码 → GOBSTYPE：取 s[0]（gnss.cpp 391-394）。"""
    return char2gobstype(code[0] if code else "")


def char2gobsband(c: str) -> GOBSBAND:
    """频带字符 → GOBSBAND（gnss.cpp 396-427）。"""
    return {
        "1": GOBSBAND.BAND_1, "2": GOBSBAND.BAND_2, "3": GOBSBAND.BAND_3,
        "5": GOBSBAND.BAND_5, "6": GOBSBAND.BAND_6, "7": GOBSBAND.BAND_7,
        "8": GOBSBAND.BAND_8, "9": GOBSBAND.BAND_9,
        "A": GOBSBAND.BAND_A, "B": GOBSBAND.BAND_B, "C": GOBSBAND.BAND_C,
        "D": GOBSBAND.BAND_D, "R": GOBSBAND.BAND_SLR,
    }.get(c, GOBSBAND.BAND)


def str2gobsband(code: str) -> GOBSBAND:
    """观测码 → GOBSBAND：单字符取 s[0]，多字符取 s[1]（gnss.cpp 451-459）。"""
    if not code:
        return GOBSBAND.BAND
    return char2gobsband(code[0] if len(code) == 1 else code[1])


def gobsband2str(b: GOBSBAND) -> str:
    """GOBSBAND → 频带字符（gnss.cpp gobsband2str）。"""
    try:
        b = GOBSBAND(b)
    except ValueError:
        return ""
    return {
        GOBSBAND.BAND_1: "1", GOBSBAND.BAND_2: "2", GOBSBAND.BAND_3: "3",
        GOBSBAND.BAND_5: "5", GOBSBAND.BAND_6: "6", GOBSBAND.BAND_7: "7",
        GOBSBAND.BAND_8: "8", GOBSBAND.BAND_9: "9",
        GOBSBAND.BAND_A: "A", GOBSBAND.BAND_B: "B", GOBSBAND.BAND_C: "C",
        GOBSBAND.BAND_D: "D", GOBSBAND.BAND_SLR: "R",
    }.get(b, "")


def char2gobsattr(c: str) -> GOBSATTR:
    """属性字符 → GOBSATTR；空格/空字符 → ATTR_NULL（gnss.cpp 481-519）。"""
    if c in (" ", "", "\0"):
        return GOBSATTR.ATTR_NULL
    try:
        return GOBSATTR["ATTR_" + c]
    except KeyError:
        return GOBSATTR.ATTR


def str2gobsattr(code: str) -> GOBSATTR:
    """观测码 → GOBSATTR：取 s[2]（gnss.cpp 521-524；2 字符码取到空 → NULL）。"""
    return char2gobsattr(code[2] if len(code) > 2 else " ")


def gobsattr2str(a: GOBSATTR) -> str:
    """GOBSATTR → 属性字符（ATTR_NULL → 空格）。"""
    try:
        a = GOBSATTR(a)
    except ValueError:
        return ""
    if a == GOBSATTR.ATTR_NULL:
        return " "
    if a == GOBSATTR.ATTR:
        return ""
    return a.name[-1]


def gobs2band(gobs: int) -> int:
    """GOBS 枚举 → 字面频带数字（gnss.cpp 2057-2097）。"""
    code = gobs2str(gobs)
    if len(code) < 2:
        return 0
    b = code[1]
    if b in "1ABC":
        return 1
    if b in "2D":
        return 2
    if b == "5":
        return 5
    if b == "6":
        return 6
    if b == "7":
        return 7
    if b == "8":
        return 8
    if b == "9":
        return 9
    return 0


def gobs_code(gobs: int) -> bool:
    """伪距观测判断（数值区间 0-99，gnss.cpp 2117-2143）。"""
    return 0 <= int(gobs) < 100


def gobs_phase(gobs: int) -> bool:
    """载波相位观测判断（数值区间 100-199）。"""
    return 100 <= int(gobs) < 200


def gobs_doppler(gobs: int) -> bool:
    """多普勒观测判断（数值区间 200-299）。"""
    return 200 <= int(gobs) < 300


def gobs_snr(gobs: int) -> bool:
    """信噪比观测判断（数值区间 300-399）。"""
    return 300 <= int(gobs) < 400


def pha2snr(gobs: int) -> int:
    """载波观测码 → 对应 SNR 观测码（+200，gnss.cpp 2099-2102）。"""
    return int(gobs) + 200


def pl2snr(gobs: int) -> int:
    """伪距或载波观测码 → 对应 SNR 观测码。"""
    gobs = int(gobs)
    if gobs_code(gobs):
        return gobs + 300
    if gobs_phase(gobs):
        return pha2snr(gobs)
    return gobs

# ---------------------------------------------------------------------------
# 静态优先级表（gnss.h 542-563 与 gnss.cpp 71-104、174-335）
# 下标 0 为占位符（BAND / LAST_GFRQ），下标 i 与 FREQ_SEQ(i) 一一对应。
# ---------------------------------------------------------------------------

BAND_PLACEHOLDER = int(GOBSBAND.BAND)   # LAST_GFRQ 占位（freq 表）/ BAND 占位（band 表）
FREQ_PLACEHOLDER = 999                  # gnss.h LAST_GFRQ

# gnss.h GNSS_FREQ_PRIORITY：每系统 GFRQ 标识的优先顺序
GNSS_FREQ_PRIORITY: dict[GSYS, list[int]] = {
    GSYS.GPS: [FREQ_PLACEHOLDER, 10, 11, 12],          # G01 G02 G05
    GSYS.GLO: [FREQ_PLACEHOLDER, 20, 21, 32, 33],      # R01 R02 R03_CDMA R05_CDMA
    GSYS.GAL: [FREQ_PLACEHOLDER, 50, 51, 52, 53, 54],  # E01 E05 E07 E08 E06
    GSYS.BDS: [FREQ_PLACEHOLDER, 60, 61, 62, 63, 64, 65, 66],
    # C02(B1I) C07(B2I) C06(B3I) C05(B2a) C09(B2b) C08(B2) C01(B1C)
    GSYS.QZS: [FREQ_PLACEHOLDER, 70, 71, 72, 73],      # J01 J02 J05 J06
    GSYS.SBS: [FREQ_PLACEHOLDER, 80, 81],              # S01 S05
    GSYS.GNS: [],
}

# gnss.h GNSS_BAND_PRIORITY：每系统 GOBSBAND 的优先顺序（按波长降序）
GNSS_BAND_PRIORITY: dict[GSYS, list[GOBSBAND]] = {
    GSYS.GPS: [GOBSBAND.BAND, GOBSBAND.BAND_1, GOBSBAND.BAND_2, GOBSBAND.BAND_5],
    GSYS.GLO: [GOBSBAND.BAND, GOBSBAND.BAND_1, GOBSBAND.BAND_2, GOBSBAND.BAND_3,
               GOBSBAND.BAND_5],
    GSYS.GAL: [GOBSBAND.BAND, GOBSBAND.BAND_1, GOBSBAND.BAND_5, GOBSBAND.BAND_7,
               GOBSBAND.BAND_8, GOBSBAND.BAND_6],
    # BDS：B1I(BAND_2) B2I(BAND_7) B3I(BAND_6) B2a(BAND_5) B2b(BAND_9) B2(BAND_8) B1C(BAND_1)
    GSYS.BDS: [GOBSBAND.BAND, GOBSBAND.BAND_2, GOBSBAND.BAND_7, GOBSBAND.BAND_6,
               GOBSBAND.BAND_5, GOBSBAND.BAND_9, GOBSBAND.BAND_8, GOBSBAND.BAND_1],
    GSYS.QZS: [GOBSBAND.BAND, GOBSBAND.BAND_1, GOBSBAND.BAND_2, GOBSBAND.BAND_5,
               GOBSBAND.BAND_6],
    GSYS.SBS: [GOBSBAND.BAND, GOBSBAND.BAND_1, GOBSBAND.BAND_5],
    GSYS.GNS: [],
}

# gnss.cpp GNSS_BAND_SORTED()（与 GNSS_BAND_PRIORITY 相同, "order is important"）
GNSS_BAND_SORTED = GNSS_BAND_PRIORITY


def sort_band(gs: GSYS, bands: Iterable[GOBSBAND]) -> list[GOBSBAND]:
    """按波长顺序（GNSS_BAND_SORTED）过滤并排序波段集合（gnss.cpp 87-104）。"""
    gs = GSYS(gs)
    order = GNSS_BAND_SORTED.get(gs, [])
    wanted = {int(b) for b in bands}
    return [b for b in order if int(b) in wanted]

# 同频跟踪属性优先级（gnss.cpp 174-335 GNSS_DATA_PRIORITY）。
# 表为 {GSYS: {GOBSBAND: 属性字符列表}}；C++ 中同一 (sys, band) 的
# TYPE_C/L/D/S 使用同一属性向量（TYPE_P 仅 ATTR_NULL），故此处按 band 合并。
GNSS_DATA_PRIORITY: dict[GSYS, dict[GOBSBAND, list[str]]] = {
    GSYS.GPS: {
        GOBSBAND.BAND_1: ["C", "S", "L", "X", "P", "W", "Y", "M", " "],
        GOBSBAND.BAND_2: ["C", "D", "S", "L", "X", "P", "W", "Y", "M", " "],
        GOBSBAND.BAND_5: ["I", "Q", "X", " "],
    },
    GSYS.GLO: {
        GOBSBAND.BAND_1: ["C", "P", " "],
        GOBSBAND.BAND_2: ["C", "P", " "],
        GOBSBAND.BAND_3: ["I", "Q", "X", " "],
    },
    GSYS.GAL: {
        GOBSBAND.BAND_1: ["A", "B", "C", "X", "Z", " "],
        GOBSBAND.BAND_5: ["I", "Q", "X", " "],
        GOBSBAND.BAND_6: ["A", "B", "C", "X", "Z", " "],
        GOBSBAND.BAND_7: ["I", "Q", "X", " "],
        GOBSBAND.BAND_8: ["I", "Q", "X", " "],
    },
    GSYS.BDS: {
        GOBSBAND.BAND_2: ["I", "Q", "X", " "],   # B1I
        GOBSBAND.BAND_6: ["I", "Q", "X", " "],   # B3I
        GOBSBAND.BAND_7: ["I", "Q", "X", " "],   # B2I
        GOBSBAND.BAND_5: ["D", "P", "X", " "],   # B2a
        GOBSBAND.BAND_9: ["D", "P", "Z", " "],   # B2b
        GOBSBAND.BAND_8: ["D", "P", "X", " "],   # B2
        GOBSBAND.BAND_1: ["D", "P", "X", " "],   # B1C
    },
    GSYS.SBS: {
        GOBSBAND.BAND_1: ["C"],
        GOBSBAND.BAND_5: ["I", "Q", "X"],
    },
    GSYS.QZS: {
        GOBSBAND.BAND_1: ["C", "S", "L", "X", "Z"],
        GOBSBAND.BAND_2: ["S", "L", "X"],
        GOBSBAND.BAND_5: ["I", "Q", "X"],
        GOBSBAND.BAND_6: ["S", "L", "X"],
    },
}


def attr_priority(gs: GSYS, band: GOBSBAND, otype: GOBSTYPE) -> list[str]:
    """(系统, 波段, 类型) 的跟踪属性优先级（gnss.cpp gsys.cpp attr_priority）。

    返回属性字符列表（首元素最优先）；该 (sys, band) 未声明时返回空表。
    """
    gs = GSYS(gs)
    table = GNSS_DATA_PRIORITY.get(gs, {})
    attrs = table.get(GOBSBAND(band))
    if attrs is None:
        return []
    # TYPE_P（P 码）在 C++ 中仅允许 ATTR_NULL；其余类型共用属性向量。
    if int(otype) == int(GOBSBAND.BAND_A):  # pragma: no cover - 防御分支
        return []
    return list(attrs)


def band_priority(gs: GSYS, iseq: int) -> GOBSBAND:
    """FREQ_SEQ 序号 → 该系统的 GOBSBAND（gsys.cpp band_priority，下标 1 起）。"""
    seq = GNSS_BAND_PRIORITY[GSYS(gs)]
    if iseq < 1 or iseq >= len(seq):
        return GOBSBAND.BAND
    return seq[iseq]

# ---------------------------------------------------------------------------
# 同频跟踪属性优先级字符串表（gdata/gobsgnss.h 73-104 行）
# LibGnut select_range/select_phase 的语义：属性字符在字符串中的**位置最大**
# （越靠后）者胜出 —— 例如 GPS BAND_1 "CPW" 中 C1W 优先于 C1C。
# ---------------------------------------------------------------------------

# 伪距（code）属性优先级（raw 读入语义）
RANGE_ORDER_ATTR_RAW: dict[GSYS, dict[GOBSBAND, str]] = {
    GSYS.GPS: {GOBSBAND.BAND_1: "CPW", GOBSBAND.BAND_2: "CLXPW", GOBSBAND.BAND_5: "QX"},
    GSYS.GAL: {GOBSBAND.BAND_1: "CX", GOBSBAND.BAND_5: "IQX", GOBSBAND.BAND_7: "IQX",
               GOBSBAND.BAND_8: "IQX", GOBSBAND.BAND_6: "ABCXZ"},
    GSYS.BDS: {GOBSBAND.BAND_2: "IQX", GOBSBAND.BAND_7: "IQX", GOBSBAND.BAND_6: "IQX",
               GOBSBAND.BAND_5: "DPX", GOBSBAND.BAND_9: "DPZ", GOBSBAND.BAND_8: "DPX",
               GOBSBAND.BAND_1: "DPX"},
    GSYS.GLO: {GOBSBAND.BAND_1: "CP", GOBSBAND.BAND_2: "CP"},
    GSYS.QZS: {GOBSBAND.BAND_1: "CSLX", GOBSBAND.BAND_2: "LX", GOBSBAND.BAND_5: "IQX"},
}

# 载波相位属性优先级（raw 读入语义；注意 BDS BAND_2 为 "XIQ"）
PHASE_ORDER_ATTR_RAW: dict[GSYS, dict[GOBSBAND, str]] = {
    GSYS.GPS: {GOBSBAND.BAND_1: "CSLXPWYM", GOBSBAND.BAND_2: "CDLXPWYM",
               GOBSBAND.BAND_5: "IQX"},
    GSYS.GAL: {GOBSBAND.BAND_1: "ABCXZ", GOBSBAND.BAND_5: "IQX", GOBSBAND.BAND_7: "IQX",
               GOBSBAND.BAND_8: "IQX", GOBSBAND.BAND_6: "ABCXZ"},
    GSYS.BDS: {GOBSBAND.BAND_2: "XIQ", GOBSBAND.BAND_7: "IQX", GOBSBAND.BAND_6: "IQX",
               GOBSBAND.BAND_5: "DPX", GOBSBAND.BAND_9: "DPZ", GOBSBAND.BAND_8: "DPX",
               GOBSBAND.BAND_1: "DPX"},
    GSYS.GLO: {GOBSBAND.BAND_1: "PC", GOBSBAND.BAND_2: "CP"},
    GSYS.QZS: {GOBSBAND.BAND_1: "CSLX", GOBSBAND.BAND_2: "LX", GOBSBAND.BAND_5: "IQX"},
}


def select_attr(gs: GSYS, band: GOBSBAND, candidates: list[str],
                phase: bool = False) -> str | None:
    """LibGnut select_range/select_phase 的单频点属性选择。

    在 ``candidates``（同一 (系统, 波段, 类型) 的观测码列表）中，取属性
    字符在 LibGnut 优先级字符串中位置最大者；均不在字符串中时返回首个候选。
    """
    gs = GSYS(gs)
    table = PHASE_ORDER_ATTR_RAW if phase else RANGE_ORDER_ATTR_RAW
    order = table.get(gs, {}).get(GOBSBAND(band), "")
    best, best_loc = None, -1
    for code in candidates:
        loc = order.find(code[2]) if len(code) >= 3 else -1
        if loc > best_loc:
            best, best_loc = code, loc
    return best


def freq_priority(gs: GSYS, iseq: int) -> int:
    """FREQ_SEQ 序号 → 该系统的 GFRQ 标识（gsys.cpp freq_priority，下标 1 起）。"""
    seq = GNSS_FREQ_PRIORITY[GSYS(gs)]
    if iseq < 1 or iseq >= len(seq):
        return FREQ_PLACEHOLDER
    return seq[iseq]


def band2freq_seq(gs: GSYS, band: GOBSBAND) -> int:
    """GOBSBAND → FREQ_SEQ 序号（gsys.cpp band2freq；未声明返回 999）。"""
    gs = GSYS(gs)
    seq = GNSS_BAND_PRIORITY.get(gs, [])
    for i, b in enumerate(seq):
        if int(b) == int(band):
            return i
    return 999

# GNSS_SATS（gnss.cpp GNSS_SATS）：每系统的默认卫星集合（PRN 范围）
GNSS_SATS: dict[GSYS, list[str]] = {
    GSYS.GPS: [f"G{i:02d}" for i in range(1, 33)],
    GSYS.GLO: [f"R{i:02d}" for i in range(1, 25)],
    GSYS.GAL: [f"E{i:02d}" for i in range(1, 37)],
    GSYS.BDS: [f"C{i:02d}" for i in range(1, 19)]
              + [f"C{i:02d}" for i in range(19, 47)]
              + [f"C{i:02d}" for i in range(58, 61)],
    GSYS.QZS: [f"J{i:02d}" for i in range(1, 8)],
    GSYS.SBS: [f"S{i:02d}" for i in range(101, 133)],
    GSYS.IRN: [f"I{i:02d}" for i in range(1, 8)],
}


def gnss_supported() -> list[GSYS]:
    """支持的系统列表（gnss.cpp GNSS_SUPPORTED）。"""
    return [GSYS.GPS, GSYS.GAL, GSYS.GLO, GSYS.BDS, GSYS.QZS, GSYS.SBS, GSYS.IRN]
