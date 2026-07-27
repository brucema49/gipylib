"""RINEX 3.02 观测文件简化器。

rtklib-py 的 rinex.decode_obs() 假设信号按频点分组交织排列
（如 C1C L1C D1C S1C C2W L2W D2W S2W），且每系统不超过 2 个频点
（MAX_NFREQ=2）。但许多 RINEX 文件的信号按类型分组
（如 C1C C2W C2X C5X D1C D2W ... L1C L2W ... S1C S2W ...），且含 3+ 频点，
会触发 "Obs file too complex" 错误或信号值落入错误频点槽位。

本模块把任意 RINEX 3.02 观测文件简化为 rtklib-py 友好的格式：
1. 每系统最多保留 max_freqs 个频点（默认 2）
2. 信号按 C-L-D-S 顺序交织排列
3. 保留原始时间戳与卫星观测值，只重排字段顺序
"""
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# 频点编号映射：信号代码后两位 → 频点序号（越小优先级越高）
# L1/E1/B1 频点: 0
# L2 频点: 1
# L5/E5a 频点: 2
# E5b/B2I 频点: 3
# E6/B3 频点: 4
_FREQ_ORDER: Dict[str, int] = {
    "1C": 0, "1X": 0, "1W": 0, "1P": 0, "1I": 0, "1M": 0, "1S": 0,
    "2C": 1, "2X": 1, "2W": 1, "2L": 1, "2P": 1, "2S": 1, "2I": 3, "2M": 1,
    "5Q": 2, "5X": 2, "5P": 2,
    "6C": 4, "6I": 4, "6X": 4,
    "7Q": 3, "7X": 3, "7I": 3, "7P": 3,
    "8X": 4, "8Q": 4, "8P": 4,
}

# 信号类型优先级：C(伪距) > L(载波) > D(多普勒) > S(信噪比)
_TYPE_ORDER = {"C": 0, "L": 1, "D": 2, "S": 3}

# 每系统最多每频点保留 4 种类型（C/L/D/S）
_TYPES_PER_FREQ = 4

# RINEX 3.02 观测字段宽度（每个信号 16 字符）
_FIELD_WIDTH = 16

# 观测行首部偏移：1(系统) + 2(PRN) + 1(空格) = 4
_LINE_HEADER_LEN = 4

# SYS / # / OBS TYPES 行每行最多 13 个信号
_SIGS_PER_HEADER_LINE = 13


def _freq_band(sig: str) -> int:
    """信号代码（如 'C1C'）→ 频点序号。未知返回 99。"""
    code = sig[1:3]
    return _FREQ_ORDER.get(code, 99)


def _type_rank(sig: str) -> int:
    """信号代码（如 'C1C'）→ 类型序号（C=0, L=1, D=2, S=3）。"""
    return _TYPE_ORDER.get(sig[0], 99)


def _parse_obs_types_lines(lines: List[str], start_idx: int) -> Tuple[List[str], int]:
    """解析 SYS / # / OBS TYPES 行（含续行），返回 (信号列表, 下一行索引)。

    输入 lines[start_idx] 是首行，形如：
        "G    16 C1C C2W C2X C5X D1C D2W D2X D5X L1C L2W L2X L5X S1C  SYS / # / OBS TYPES"
    """
    line = lines[start_idx]
    nsig = int(line[3:6])
    # 首行信号从位置 7 开始，每个 4 字符
    sig_str = line[6:60]
    sigs = re.findall(r"[A-Z]\d[A-Z]", sig_str)
    idx = start_idx + 1
    # 续行：位置 7 开始，每 4 字符
    while len(sigs) < nsig and idx < len(lines):
        cont = lines[idx]
        if "SYS / # / OBS TYPES" not in cont:
            break
        sig_str = cont[6:60]
        sigs.extend(re.findall(r"[A-Z]\d[A-Z]", sig_str))
        idx += 1
    return sigs[:nsig], idx


def _select_signals(sigs: List[str], max_freqs: int,
                    preferred_freqs: List[int] = None) -> Tuple[List[str], List[Optional[int]]]:
    """从原始信号列表中选择最多 max_freqs 个频点的信号。

    Args:
        sigs: 原始信号列表
        max_freqs: 最多保留频点数
        preferred_freqs: 优先保留的频点列表（如 [3, 0] 表示优先保留 B2I, 其次 L1）。
                     若为 None 或优先频点不足 max_freqs 个，则按频点序号补齐。

    返回 (新信号列表, 旧索引→新索引映射，None 表示丢弃)。
    新信号列表按 C-L-D-S 顺序交织排列。
    """
    # 按频点分组
    freq_groups: Dict[int, Dict[str, str]] = {}  # freq_band → {type_char: sig}
    for i, sig in enumerate(sigs):
        fb = _freq_band(sig)
        if fb == 99:
            continue
        tc = sig[0]
        if tc not in _TYPE_ORDER:
            continue
        if fb not in freq_groups:
            freq_groups[fb] = {}
        # 同频点同类型只保留第一个
        if tc not in freq_groups[fb]:
            freq_groups[fb][tc] = sig

    # 选择频点：优先保留 preferred_freqs 中存在的频点
    available_freqs = set(freq_groups.keys())
    selected_freqs = []
    if preferred_freqs:
        for pf in preferred_freqs:
            if pf in available_freqs and pf not in selected_freqs:
                selected_freqs.append(pf)
                if len(selected_freqs) >= max_freqs:
                    break
        # preferred_freqs 指定时只保留指定的频点, 不补齐
        # (避免基站/流动站频点不一致: 如 BDS preferred=[3] 时
        #  流动站只有 B2I, 基站补齐 L1 后频点槽位错位)
    else:
        # 无 preferred_freqs 时按频点序号补齐到 max_freqs
        for fb in sorted(available_freqs):
            if fb not in selected_freqs:
                selected_freqs.append(fb)
                if len(selected_freqs) >= max_freqs:
                    break
    selected_freqs = sorted(selected_freqs)

    # 构建新信号列表（C-L-D-S 顺序）与索引映射
    new_sigs: List[str] = []
    old_to_new: List[Optional[int]] = [None] * len(sigs)
    for fb in selected_freqs:
        for tc in ["C", "L", "D", "S"]:
            if tc in freq_groups[fb]:
                sig = freq_groups[fb][tc]
                new_idx = len(new_sigs)
                new_sigs.append(sig)
                # 找到原始索引（同频点同类型的第一个）
                for i, s in enumerate(sigs):
                    if old_to_new[i] is None and s == sig and _freq_band(s) == fb:
                        old_to_new[i] = new_idx
                        break
    return new_sigs, old_to_new


def _format_obs_types_line(sys_char: str, sigs: List[str]) -> List[str]:
    """生成 SYS / # / OBS TYPES 行（含续行）。"""
    nsig = len(sigs)
    lines = []
    # 首行：13 个信号
    first_batch = sigs[:_SIGS_PER_HEADER_LINE]
    sig_str = " ".join(f"{s:<3}" for s in first_batch)
    line1 = f"{sys_char}{nsig:5d} {sig_str}"
    # 填充到 60 字符，加标签
    line1 = line1.ljust(60) + "SYS / # / OBS TYPES"
    lines.append(line1)
    # 续行
    rest = sigs[_SIGS_PER_HEADER_LINE:]
    while rest:
        batch = rest[:_SIGS_PER_HEADER_LINE]
        rest = rest[_SIGS_PER_HEADER_LINE:]
        sig_str = " ".join(f"{s:<3}" for s in batch)
        cont = "      " + sig_str
        cont = cont.ljust(60) + "SYS / # / OBS TYPES"
        lines.append(cont)
    return lines


def _reorder_obs_line(line: str, old_to_new: List[Optional[int]]) -> str:
    """重排观测行字段。old_to_new[i] = j 表示原字段 i → 新位置 j。"""
    if len(line) < _LINE_HEADER_LEN:
        return line
    header = line[:_LINE_HEADER_LEN]
    # 提取原始字段（每 16 字符）
    body = line[_LINE_HEADER_LEN:].rstrip("\n").rstrip()
    old_fields = []
    for i in range(0, len(body), _FIELD_WIDTH):
        old_fields.append(body[i:i + _FIELD_WIDTH])

    new_fields = [" " * _FIELD_WIDTH] * len([x for x in old_to_new if x is not None])
    for old_idx, new_idx in enumerate(old_to_new):
        if new_idx is not None and old_idx < len(old_fields):
            new_fields[new_idx] = old_fields[old_idx].ljust(_FIELD_WIDTH)

    return header + "".join(new_fields) + "\n"


def simplify_rinex(input_path: str, output_path: str, max_freqs: int = 2,
                   freq_priority: Dict[str, List[int]] = None) -> str:
    """简化 RINEX 3.02 观测文件。

    Args:
        input_path: 输入 RINEX 文件路径
        output_path: 输出简化后 RINEX 文件路径
        max_freqs: 每系统最多保留频点数（默认 2，匹配 rtklib-py MAX_NFREQ）
        freq_priority: 各系统优先保留的频点列表，键为系统字符（'G'/'C'/'E'等），
                      值为频点序号列表（如 [3, 0] 表示优先保留 B2I, 其次 L1）。
                      用于确保基站与流动站频点匹配。若为 None 则按频点序号选择。

    Returns:
        输出文件路径
    """
    in_path = Path(input_path)
    out_path = Path(output_path)

    with open(in_path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()

    # 1. 解析 header，构建每系统的信号映射
    # sys_char → (信号列表, 旧→新映射)
    sys_sigs: Dict[str, Tuple[List[str], List[Optional[int]]]] = {}
    block_first_indices: Dict[int, str] = {}  # 首行索引 → sys_char
    skip_indices = set()  # 需要跳过的原始行索引（含续行）
    header_end_idx = 0

    i = 0
    while i < len(lines):
        line = lines[i]
        if "END OF HEADER" in line:
            header_end_idx = i
            break
        if "SYS / # / OBS TYPES" in line and line[0] in "GREJC":
            sys_char = line[0]
            sigs, next_i = _parse_obs_types_lines(lines, i)
            # 标记首行和续行为跳过
            block_first_indices[i] = sys_char
            skip_indices.add(i)
            for j in range(i + 1, next_i):
                skip_indices.add(j)
            preferred = freq_priority.get(sys_char) if freq_priority else None
            new_sigs, old_to_new = _select_signals(sigs, max_freqs, preferred)
            sys_sigs[sys_char] = (new_sigs, old_to_new)
            i = next_i
            continue
        i += 1

    # 2. 写输出文件
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        # 写 header（替换 SYS / # / OBS TYPES 行）
        i = 0
        while i <= header_end_idx:
            if i == header_end_idx:
                f.write(lines[i])
                break
            if i in skip_indices:
                # 仅首行写简化后的 SYS / # / OBS TYPES，续行直接跳过
                if i in block_first_indices:
                    sys_char = block_first_indices[i]
                    if sys_char in sys_sigs:
                        new_sigs, _ = sys_sigs[sys_char]
                        if new_sigs:
                            for nl in _format_obs_types_line(sys_char, new_sigs):
                                f.write(nl + "\n")
                i += 1
                continue
            f.write(lines[i])
            i += 1

        # 3. 写观测数据（重排字段）
        for line in lines[header_end_idx + 1:]:
            if not line.strip() or line[0] == ">":
                f.write(line)
                continue
            sys_char = line[0] if line else " "
            if sys_char in sys_sigs:
                _, old_to_new = sys_sigs[sys_char]
                f.write(_reorder_obs_line(line, old_to_new))
            else:
                f.write(line)

    return str(out_path)


def needs_simplification(input_path: str, max_freqs: int = 2) -> bool:
    """检查 RINEX 文件是否需要简化（任一系统频点数 > max_freqs 或信号未按交织顺序）。"""
    in_path = Path(input_path)
    if not in_path.exists():
        return False

    with open(in_path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i]
        if "END OF HEADER" in line:
            break
        if "SYS / # / OBS TYPES" in line and line[0] in "GREJC":
            sigs, next_i = _parse_obs_types_lines(lines, i)
            # 检查频点数
            freqs = set()
            for s in sigs:
                fb = _freq_band(s)
                if fb != 99:
                    freqs.add(fb)
            if len(freqs) > max_freqs:
                return True
            # 检查是否已按交织顺序（同频点信号连续）
            last_freq = -1
            for s in sigs:
                fb = _freq_band(s)
                if fb == 99:
                    continue
                if fb < last_freq:
                    return True  # 频点回退，未按交织顺序
                last_freq = fb
            i = next_i
            continue
        i += 1
    return False
