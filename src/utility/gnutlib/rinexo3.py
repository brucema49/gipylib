"""LibGnut RINEX 3 观测读取移植（GREAT-MSF/src/LibGnut/gcoders/rinexo3.cpp +
rinexo2.cpp + gcoder 缓冲语义）。

覆盖 C++ 读取器的完整行为：

- 头部：RINEX VERSION / TYPE、SYS / # / OBS TYPES（每行 13 个信号、多续行）、
  SYS / SCALE FACTOR、SYS / PHASE SHIFT、GLONASS SLOT / FRQ #、
  GLONASS COD/PHS/BIS；
- BDS 频带归一化 ``_fix_band``（rinexo3.cpp 698-731）：版本 <= 3.03 时
  C1x→C2x（B1I）、C3x→C6x（B3I）；版本 >= 3.04 时 C7D/P/Z→C9D/P/Z（B2b）；
- 数据行：每观测 16 列，数值域 14 列（3+16i 起），LLI 在 +14，信号强度位在
  +15（映射为近似 dBHz）；0 值观测丢弃；比例因子乘入；
- pyrinrx（rtklib-py）风格输出：历元对象提供 ``P/L/D/S/lli`` 槽位数组视图。

纯读取工具，不做解算；槽位映射由调用方（rinex_improve）以 LibGnut
波段优先级给出。
"""

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterator

import numpy as np

from .gnss import GOBSBAND, GSYS, gobs2str, pha2snr, str2gobs

# SYS / # / OBS TYPES 每行最多 13 个信号（rinexo3.cpp 38-95）
_SIGS_PER_LINE = 13
# 观测字段宽度（rinexo2.cpp _read_obs：idx = 3 + 16*ii）
_FIELD_WIDTH = 16
# 数值域宽度（14 列）与 LLI / 信号强度位偏移
_VALUE_WIDTH = 14
_LLI_OFFSET = _VALUE_WIDTH
_SSI_OFFSET = _VALUE_WIDTH + 1

# 相位观测 SNR 强度位 → 近似 dBHz（rinexo2.cpp 931-1010 的映射）
_SSI_TO_DBHZ = {1: 6.0, 2: 15.0, 3: 20.0, 4: 27.0, 5: 33.0, 6: 39.0,
                7: 45.0, 8: 50.0, 9: 60.0}

_OBS_CODE_RE = re.compile(r"[CLDS]\d[A-Z]")


def fix_band(sys_char: str, code: str, version: float) -> str:
    """BDS 观测码频带归一化（rinexo3.cpp _fix_band 的移植）。

    - 版本 <= 3.03：BDS C1x → C2x（B1I 以 band 1 记录时归一到 BAND_2）；
      C3x → C6x（B3I 归一到 BAND_6）；
    - 版本 >= 3.04：BDS C7D/P/Z → C9D/P/Z（B2b 归一到 BAND_9）。
    非 BDS 或非 RINEX3 观测码原样返回。
    """
    if sys_char != "C" or len(code) != 3:
        return code
    out = code
    if version <= 3.03:
        if out[1] == "1":
            out = out[0] + "2" + out[2]
        if out[1] == "3":
            out = out[0] + "6" + out[2]
    if version >= 3.04 and out[1] == "7" and out[2] in "DPZ":
        out = out[0] + "9" + out[2]
    return out


@dataclass
class ScaleFactor:
    """SYS / SCALE FACTOR 头解析结果。"""
    factor: int = 1
    sats: list[str] = field(default_factory=list)  # 空 = 应用于整个系统


@dataclass
class RinexObsHeader:
    """RINEX 3 观测文件头部解析结果（对应 t_rnxhdr）。"""
    version: float = 0.0
    types: dict[str, list[str]] = field(default_factory=dict)   # sys → 观测码
    scale_factors: dict[str, ScaleFactor] = field(default_factory=dict)
    phase_shifts: dict[str, dict[str, float]] = field(default_factory=dict)
    glo_slot_freq: dict[str, int] = field(default_factory=dict)  # Rxx → 通道号
    glo_code_phase_bias: dict[str, float] = field(default_factory=dict)
    marker_name: str = ""
    observer: str = ""
    rec_type: str = ""
    ant_type: str = ""
    position: tuple[float, float, float] | None = None
    first_obs: str | None = None
    last_obs: str | None = None

    def normalized_types(self) -> dict[str, list[str]]:
        """应用 ``fix_band`` 后的观测码表（与 LibGnut 读取行为一致）。"""
        return {
            sys: [fix_band(sys, code, self.version) for code in codes]
            for sys, codes in self.types.items()
        }

    def scale_for(self, sys_char: str, sat: str) -> float:
        """卫星观测值的比例因子（rinexo3.cpp 99-166）。"""
        sf = self.scale_factors.get(sys_char)
        if sf is None:
            return 1.0
        if not sf.sats or sat in sf.sats:
            return float(sf.factor)
        return 1.0


def _parse_float(text: str) -> float:
    text = text.strip()
    if not text:
        return 0.0
    try:
        return float(text.replace("D", "E").replace("d", "e"))
    except ValueError:
        return 0.0


def parse_obs_header(path: str) -> RinexObsHeader:
    """解析 RINEX 3 观测文件头部（含多续行 OBS TYPES 与比例因子）。"""
    header = RinexObsHeader()
    pending_sys = None       # SCALE FACTOR 续行归属
    pending_nsat = 0
    pending_sats: list[str] = []

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if len(line) < 60:
                if line.strip() == "":
                    continue
                label = ""
            else:
                label = line[60:].rstrip("\r\n")
            if label.startswith("RINEX VERSION / TYPE"):
                header.version = _parse_float(line[4:12])
                continue
            if label.startswith("END OF HEADER"):
                break
            if label.startswith("MARKER NAME"):
                header.marker_name = line[0:60].strip()
                continue
            if label.startswith("OBSERVER / AGENCY"):
                header.observer = line[0:60].strip()
                continue
            if label.startswith("REC # / TYPE / VERS"):
                header.rec_type = line[0:20].strip()
                continue
            if label.startswith("ANT # / TYPE"):
                header.ant_type = line[20:40].strip()
                continue
            if label.startswith("APPROX POSITION XYZ"):
                header.position = (
                    _parse_float(line[0:14]), _parse_float(line[14:28]),
                    _parse_float(line[28:42]))
                continue
            if label.startswith("TIME OF FIRST OBS"):
                header.first_obs = line[0:43].strip()
                continue
            if label.startswith("TIME OF LAST OBS"):
                header.last_obs = line[0:43].strip()
                continue
            if label.startswith("SYS / # / OBS TYPES"):
                if not line or line[0] not in "GRECJSI":
                    continue
                sys_char = line[0]
                nsig = int(line[3:6])
                codes: list[str] = []
                # 首行信号自第 7 列起，每 4 列一个 3 字符码；续行一次读入
                chunk = line[7:7 + 4 * _SIGS_PER_LINE]
                codes.extend(m.group(0) for m in
                             _OBS_CODE_RE.finditer(chunk))
                while len(codes) < nsig:
                    cont = f.readline()
                    if not cont:
                        break
                    codes.extend(m.group(0) for m in
                                 _OBS_CODE_RE.finditer(cont[7:7 + 4 * _SIGS_PER_LINE]))
                header.types[sys_char] = codes[:nsig]
                continue
            if label.startswith("SYS / SCALE FACTOR"):
                factor = int(line[2:6] or 1)
                nsat = int(line[8:10] or 0)
                sats: list[str] = []
                for i in range(min(nsat, 12)):
                    sat = line[11 + 4 * i:14 + 4 * i].strip()
                    if sat:
                        sats.append(sat)
                if nsat == 0:
                    header.scale_factors[line[0]] = ScaleFactor(factor, [])
                    pending_sys, pending_nsat, pending_sats = None, 0, []
                else:
                    pending_sys, pending_nsat, pending_sats = line[0], nsat, sats
                    if len(sats) >= nsat:
                        header.scale_factors[pending_sys] = ScaleFactor(
                            factor, list(pending_sats))
                        pending_sys, pending_nsat, pending_sats = None, 0, []
                continue
            if label.startswith("SYS / PHASE SHIFT"):
                if line[0] in "GRECJSI":
                    shifts = header.phase_shifts.setdefault(line[0], {})
                    for i in range(12):
                        sat = line[8 + 7 * i:11 + 7 * i].strip()
                        val = line[11 + 7 * i:22 + 7 * i].strip()
                        if sat and val:
                            shifts[sat] = _parse_float(val) * math.pi
                continue
            if label.startswith("GLONASS SLOT / FRQ #"):
                for i in range(8):
                    sat = line[4 + 7 * i:7 + 7 * i].strip()
                    chn = line[7 + 7 * i:10 + 7 * i].strip()
                    if sat.startswith("R") and chn:
                        header.glo_slot_freq[sat] = int(_parse_float(chn))
                continue
            if label.startswith("GLONASS COD/PHS/BIS"):
                header.glo_code_phase_bias["P1"] = _parse_float(line[5:14])
                header.glo_code_phase_bias["P2"] = _parse_float(line[19:28])
                continue
            # SCALE FACTOR 卫星列表续行（无 60 列标签）
            if pending_sys is not None and line.startswith("      "):
                for i in range(12):
                    sat = line[11 + 4 * i:14 + 4 * i].strip()
                    if sat:
                        pending_sats.append(sat)
                if len(pending_sats) >= pending_nsat:
                    sf = header.scale_factors.get(pending_sys, ScaleFactor(1, []))
                    header.scale_factors[pending_sys] = ScaleFactor(
                        sf.factor, pending_sats[:pending_nsat])
                    pending_sys, pending_nsat, pending_sats = None, 0, []
                continue

    return header


@dataclass
class SatelliteObs:
    """单卫星原始观测（rinexo3 _read_obstypes 输出；pyrinrx 兼容旁路视图）。"""
    sat: str                                    # 卫星号（如 "C01"）
    sys_char: str                               # RINEX 系统字符
    values: dict[str, float] = field(default_factory=dict)  # 观测码 → 数值
    lli: dict[str, int] = field(default_factory=dict)       # 观测码 → LLI
    snr: dict[str, float] = field(default_factory=dict)     # 观测码 → dBHz
    scale: float = 1.0

    def code(self, band: int, otype: str = "C") -> float | None:
        """按 (频带数字, 类型) 取第一个可用观测值（LibGnut _id_range 语义）。"""
        for obs_code, value in self.values.items():
            if (len(obs_code) == 3 and obs_code[0] == otype
                    and obs_code[1].isdigit() and int(obs_code[1]) == band):
                return value
        return None


@dataclass
class EpochObs:
    """单历元观测（rinexo3 _decode_data 输出）。"""
    time: tuple[int, int, int, int, int, float]  # (y, m, d, h, mi, s)
    flag: int = 0
    sats: list[SatelliteObs] = field(default_factory=list)
    slot_arrays: dict | None = None  # pyrinrx 风格槽位数组（可选附加）

    @property
    def epoch_seconds(self) -> float:
        y, m, d, h, mi, s = self.time
        # 简化儒略日 → 自 1980-01-06 GPST 起的秒（仅用于排序/展示）
        from datetime import datetime
        dt = datetime(y, m, d, h, mi) - datetime(1980, 1, 6)
        return dt.total_seconds() + s


def iter_obs_epochs(path: str, header: RinexObsHeader,
                    systems: list[str] | None = None,
                    sat_filter: Callable[[str], bool] | None = None,
                    field_offset: int = 3,
                    ) -> Iterator[EpochObs]:
    """逐历元流式读取观测数据（rinexo3.cpp _decode_data / _read_epoch 语义）。

    Args:
        path: RINEX 3 观测文件路径
        header: :func:`parse_obs_header` 的结果
        systems: 保留的系统字符列表；None 表示全部
        sat_filter: 可选卫星过滤器（返回 False 的卫星跳过）
        field_offset: 观测值域起始列偏移。LibGnut/RINEX 规范为 3
            （卫星号 3 列后紧跟 16 列观测域，域内 14 列数值 + LLI + 强度）。
            仓库内的 pyrinrx 解码器与 rinex_improve 改写器历史上使用 4，
            调用方可显式对齐该约定（见 rinex_improve.code_availability）。

    Yields:
        :class:`EpochObs`
    """
    wanted = set(systems) if systems else None
    codes = header.normalized_types()
    scale_cache: dict[tuple[str, str], float] = {}

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        # 跳过头部（rinexo3.cpp 以第 60 列起的标签判定；标签后可能有尾随空格）
        for line in f:
            label = line[60:].rstrip("\r\n") if len(line) >= 60 else ""
            if label.startswith("END OF HEADER"):
                break
        for line in f:
            if not line.startswith(">"):
                continue
            year = int(line[2:6])
            month = int(line[7:9])
            day = int(line[10:12])
            hour = int(line[13:15])
            minute = int(line[16:18])
            sec = float(line[19:29] or 0)
            flag = int(line[31] or 0)
            nsat = int(line[32:35] or 0)
            # 事件历元（flag != 0/1）跳过其后 nsat 行（rinexo3.cpp 441-463）
            skip_data_lines = 0 if flag in (0, 1) else nsat
            if skip_data_lines:
                for _ in range(skip_data_lines):
                    f.readline()
                continue
            epoch = EpochObs(time=(year, month, day, hour, minute, sec),
                             flag=flag)
            for _ in range(nsat):
                line = f.readline()
                if not line:
                    break
                sys_char = line[0]
                if sys_char not in codes:
                    continue
                if wanted is not None and sys_char not in wanted:
                    continue
                sat = f"{sys_char}{line[1:3]}"
                if sat_filter is not None and not sat_filter(sat):
                    continue
                key = (sys_char, sat)
                if key not in scale_cache:
                    scale_cache[key] = header.scale_for(sys_char, sat)
                scale = scale_cache[key]
                satobs = SatelliteObs(sat=sat, sys_char=sys_char, scale=scale)
                decl = codes[sys_char]
                for i, obs_code in enumerate(decl):
                    start = int(field_offset) + _FIELD_WIDTH * i
                    value_field = line[start:start + _VALUE_WIDTH]
                    if len(value_field) < _VALUE_WIDTH or not value_field.strip():
                        continue  # 缺测（rinexo2.cpp _read_obs：len < idx+14）
                    value = _parse_float(value_field)
                    if value == 0.0:
                        continue  # 0 值观测直接丢弃
                    value *= scale
                    satobs.values[obs_code] = value
                    if obs_code[0] == "L":
                        lli_field = line[start + _LLI_OFFSET:start + _LLI_OFFSET + 1]
                        if lli_field and lli_field != " ":
                            lli = int(lli_field)
                            if lli > 3:
                                lli -= 4  # 兼容（rinexo2.cpp）
                            satobs.lli[obs_code] = lli
                        ssi_field = line[start + _SSI_OFFSET:start + _SSI_OFFSET + 1]
                        if ssi_field and ssi_field != " ":
                            snr_code_str = gobs2str(pha2snr(str2gobs(obs_code)))
                            if snr_code_str:
                                satobs.snr[snr_code_str] = _SSI_TO_DBHZ.get(
                                    int(ssi_field), 0.0)
                    elif obs_code[0] == "C":
                        # 伪距标准差位（pyrinrx Pstd）
                        std_field = line[start + _LLI_OFFSET + 1:
                                         start + _LLI_OFFSET + 2]
                        if std_field and std_field != " ":
                            satobs.snr.setdefault(
                                f"_{obs_code}_std", float(int(std_field)))
                epoch.sats.append(satobs)
            yield epoch


def _reverse_lookup(gobs: int) -> str:
    from .gnss import gobs2str
    return gobs2str(gobs)


def band_slot_view(epoch: EpochObs, band_slot: dict[str, dict[int, int]],
                   max_freq: int = 2):
    """把历元原始观测折叠为 pyrinrx（rtklib-py Obs）风格的槽位数组。

    Args:
        epoch: :func:`iter_obs_epochs` 产生的历元
        band_slot: {系统字符: {raw band 数字: 槽位}}（LibGnut 波段优先级映射）
        max_freq: 槽位数（pyrinrx MAX_NFREQ）

    Returns:
        dict(P=, L=, D=, S=, lli=, sat=) numpy 数组（sat 为 RINEX 卫星号列表）。
        槽位与 pyrinrx ``Obs`` 的 ``P/L/D/S/lli`` 数组同构，可直接桥接。
    """
    n = len(epoch.sats)
    P = np.zeros((n, max_freq))
    L = np.zeros((n, max_freq))
    D = np.zeros((n, max_freq))
    S = np.zeros((n, max_freq))
    lli = np.zeros((n, max_freq), dtype=int)
    sats: list[str] = []
    for i, satobs in enumerate(epoch.sats):
        sats.append(satobs.sat)
        slot_map = band_slot.get(satobs.sys_char, {})
        for obs_code, value in satobs.values.items():
            if len(obs_code) != 3 or not obs_code[1].isdigit():
                continue
            band = int(obs_code[1])
            slot = slot_map.get(band)
            if slot is None or slot >= max_freq:
                continue
            col = slot
            tchar, attr = obs_code[0], obs_code[2]
            if tchar == "C":
                P[i, col] = value
            elif tchar == "L":
                L[i, col] = value
                lli[i, col] = satobs.lli.get(obs_code, 0)
            elif tchar == "D":
                D[i, col] = value
            elif tchar == "S":
                S[i, col] = value
    return {"P": P, "L": L, "D": D, "S": S, "lli": lli, "sat": sats}


def declared_bands(header: RinexObsHeader,
                   sys_char: str) -> list[int]:
    """头部声明（经 fix_band 归一化）的 raw 频带数字列表（保持声明顺序）。"""
    bands: list[int] = []
    for code in header.normalized_types().get(sys_char, []):
        if len(code) == 3 and code[1].isdigit():
            band = int(code[1])
            if band not in bands:
                bands.append(band)
    return bands


def band_frequency_hz(sys_char: str, band: int, channel: int = 0) -> float:
    """(RINEX 系统字符, raw 频带数字) → 载波频率 [Hz]（gnutlib.gsys 便捷桥）。"""
    from .gsys import frequency
    sys_map = {"G": GSYS.GPS, "R": GSYS.GLO, "E": GSYS.GAL,
               "C": GSYS.BDS, "J": GSYS.QZS, "S": GSYS.SBS, "I": GSYS.IRN}
    gs = sys_map.get(sys_char)
    if gs is None:
        return 0.0
    return frequency(gs, GOBSBAND(band), channel)
