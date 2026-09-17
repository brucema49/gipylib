"""rinex_improve — RINEX 观测读取/信号选择的升级版（取代 rinex_simplifier）。

设计原则（issue/9-15 北斗频点映射后续轮）：

1. **LibGnut 处理逻辑第一位**：观测码归一化（BDS ``_fix_band``）、波段优先级
   （``gnss.GNSS_BAND_PRIORITY``）、同频跟踪属性优先级
   （``gnss.GNSS_DATA_PRIORITY``）全部来自 :mod:`src.utility.gnutlib` 的
   LibGnut 移植，不再维护 gipylib 私有默认表。
2. **pyrinrx（rtklib-py）风格兼容**：输出仍是 rtklib-py 解码器
   （``rnx_decode``）可直接消费的 RINEX 3 文件——每系统 ≤ nf 个频点、
   C-L-D-S 交织排列、原始时间戳与卫星观测值不变。
3. **用户不需要配置信号映射**：流动站/基站的波段方案由本模块解析
   RINEX 头自动规划（共同波段 ∩ LibGnut 波段优先级），
   ``freq_table/freq_ix0/freq_ix1/dfreq_glo`` 等频率映射键全部自动派生。

旧接口 ``rinex_simplifier.py`` 保留兼容；新代码一律走本模块。
"""

from pathlib import Path


from src.stream.gnss_band_mapping import (
    RINEX_TO_SYSTEM,
    SYSTEM_TO_RINEX,
    resolve_raw_band_priority,
)
from src.utility.gnutlib import (
    CLIGHT,
    GOBSBAND,
    GNSS_BAND_PRIORITY,
    GSYS,
    band_frequency_hz,
    declared_bands,
    fix_band,
    frequency as gnut_frequency,
    glo_l1_frequency,
    glo_l2_frequency,
    iter_obs_epochs,
    parse_obs_header,
    select_attr,
)

# ---------------------------------------------------------------------------
# LibGnut 派生的默认频率表与 GLONASS 通道间隔
# ---------------------------------------------------------------------------

# 规范频率表 [Hz]。前 7 项与历史配置 freq_table 完全一致（0=L1/E1, 1=L2,
# 2=L5/E5a, 3=E5b/B2I, 4=GLO L1, 5=GLO L2, 6=B1I），第 7 项起为 LibGnut
# 扩展（7=B3I, 8=E6），自动派生的 freq_ix 引用本表。
DEFAULT_FREQ_TABLE: list[float] = [
    gnut_frequency(GSYS.GPS, GOBSBAND.BAND_1),    # 0: L1 / E1 / B1C / J L1
    gnut_frequency(GSYS.GPS, GOBSBAND.BAND_2),    # 1: L2
    gnut_frequency(GSYS.GPS, GOBSBAND.BAND_5),    # 2: L5 / E5a / B2a
    gnut_frequency(GSYS.GAL, GOBSBAND.BAND_7),    # 3: E5b / B2I / B2b
    glo_l1_frequency(0),                          # 4: GLO L1 (k=0)
    glo_l2_frequency(0),                          # 5: GLO L2 (k=0)
    gnut_frequency(GSYS.BDS, GOBSBAND.BAND_2),    # 6: B1I
    gnut_frequency(GSYS.BDS, GOBSBAND.BAND_6),    # 7: B3I
    gnut_frequency(GSYS.GAL, GOBSBAND.BAND_6),    # 8: E6 / QZS L6
    gnut_frequency(GSYS.GAL, GOBSBAND.BAND_8),    # 9: E5b+E5a (B2) / E8
]

# LibGnut gsys.h R01_STEP / R02_STEP
DEFAULT_DFREQ_GLO: list[float] = [0.5625e6, 0.4375e6]

# 观测类型输出顺序（pyrinrx 交织风格）
_TYPE_ORDER = ("C", "L", "D", "S")

_FREQ_MATCH_TOL = 1.0  # Hz，频率表查重容差

# ---------------------------------------------------------------------------
# 波段规划：解析 RINEX 头 → LibGnut 优先级 → 流级波段方案
# ---------------------------------------------------------------------------

def _system_chars(gnss_t) -> list[str] | None:
    """gnss_t 系统名列表 → RINEX 系统字符列表；None/空 表示全部。"""
    if not gnss_t:
        return None
    chars = []
    for name in gnss_t:
        key = str(name).strip().upper()
        if key in SYSTEM_TO_RINEX:
            chars.append(SYSTEM_TO_RINEX[key])
        elif key in RINEX_TO_SYSTEM:
            chars.append(key)
        else:
            raise ValueError(f"gnss_t has unknown system '{name}'")
    return chars


def _paths(path) -> list[str]:
    if path is None:
        return []
    if isinstance(path, (list, tuple)):
        return [str(p) for p in path if p]
    return [str(path)]


def _gnut_sys(sys_char: str) -> GSYS | None:
    return {
        "G": GSYS.GPS, "R": GSYS.GLO, "E": GSYS.GAL,
        "C": GSYS.BDS, "J": GSYS.QZS, "S": GSYS.SBS, "I": GSYS.IRN,
    }.get(sys_char)


def plan_stream_signals(rover_path, base_path=None,
                        gnss_t=None, nf: int = 2,
                        verify_consistency: bool = True
                        ) -> dict[str, list[int]]:
    """自动规划流级 raw 波段方案（用户无需配置信号映射）。

    规则（LibGnut 逻辑第一位 + 物理自检）：

    1. 逐文件解析 ``SYS / # / OBS TYPES`` 头，观测码先经 LibGnut
       ``fix_band`` 归一化（BDS C1I→C2I 等）；
    2. 取流动站与基站声明波段的**交集**（双差只在共同波段成立）；
    3. （默认开启）用「码-相一致性」自检波段假设的载波波长：同一波段
       正确波长下 ``Δ(C - λL)`` 的跨星中位数为 cm 级，波长错误时为 m 级；
       只在候选多于一个时剔除不一致的波段（fail-open，避免稀疏数据误杀）；
    4. 按 LibGnut ``GNSS_BAND_PRIORITY``（B1I 优先于 B3I 等波长序）排序；
    5. 截断到 ``nf`` 个频点。

    Returns:
        ``{RINEX 系统字符: [raw band, ...]}``（波段数 ≤ nf）
    """
    rover_paths = _paths(rover_path)
    base_paths = _paths(base_path)
    if not rover_paths:
        raise ValueError("plan_stream_signals requires a rover RINEX path")

    rover_header = parse_obs_header(rover_paths[0])
    rover_bands = {
        sys: declared_bands(rover_header, sys)
        for sys in (rover_header.types or {})
    }
    base_bands: dict[str, list[int]] = {}
    for path in base_paths:
        header = parse_obs_header(path)
        for sys in header.types:
            bands = declared_bands(header, sys)
            base_bands.setdefault(sys, [])
            for band in bands:
                if band not in base_bands[sys]:
                    base_bands[sys].append(band)

    wanted = _system_chars(gnss_t)
    plan: dict[str, list[int]] = {}
    for sys, rover_sys_bands in rover_bands.items():
        if wanted is not None and sys not in wanted:
            continue
        if not rover_sys_bands:
            continue
        if base_paths:
            allowed = set(base_bands.get(sys, []))
            common = [b for b in rover_sys_bands if b in allowed]
        else:
            common = list(rover_sys_bands)
        if not common:
            continue
        if verify_consistency and len(common) > 1:
            measured = {b: band_consistency(rover_paths[0], sys, b)
                        for b in common}
            known = [v for v in measured.values() if v is not None]
            if known:
                # 相对判据：手机等噪声较大的接收机即使波长正确也可能有
                # 0.7-1.0 m 的指标，故只在另一候选明显更优（比值）且超过
                # 绝对下限时才剔除，避免误杀。无样本(None)视为通过。
                limit = max(BAND_CONSISTENCY_LIMIT_M,
                            BAND_CONSISTENCY_RELATIVE_FACTOR * min(known))
                consistent = [b for b in common
                              if measured[b] is None or measured[b] <= limit]
                if consistent:
                    common = consistent
        plan[sys] = _order_by_gnut_priority(sys, common)[:max(1, int(nf))]
    return plan

# 码-相一致性判据：绝对下限 [m] 与相对倍数。正确波长下 Δ(C-λL) 跨星
# 中位数为 cm 级（低噪声接收机）到 ~1 m（手机码噪声）；用错波长（例如把
# B1C 当 B1I）时指标约为同系统最优候选的 10 倍以上。仅在候选波段多于
# 一个时生效，且剔除后必须仍有余量（fail-open）。
BAND_CONSISTENCY_LIMIT_M = 0.6
BAND_CONSISTENCY_RELATIVE_FACTOR = 8.0


def band_consistency(path: str, sys_char: str, band: int,
                     max_epochs: int = 60) -> float | None:
    """测量 (系统, 波段) 的码-相一致性（验证该波段载波波长假设）。

    对每个历元的每颗卫星计算 ``C - λ(band)·L``，取相邻历元之差并消去
    历元公共项（接收机钟差/几何公共部分）后的跨星绝对值中位数：

    - 波长正确 → 仅剩电离层/多路径/噪声，量级 cm；
    - 波长错误 → 残留与几何距离变化成正比，量级 m。

    Returns:
        一致性指标 [m]；样本不足（<4 颗共同星或 <2 对历元）时返回 None。
    """
    gs = _gnut_sys(sys_char)
    if gs is None:
        return None
    header = parse_obs_header(path)
    code_codes = _band_codes_for(path, sys_char, band, "C", header)
    phase_codes = _band_codes_for(path, sys_char, band, "L", header)
    if not code_codes or not phase_codes:
        return None
    freq = band_frequency_hz(sys_char, band)
    if freq <= 0:
        return None
    lam = CLIGHT / freq
    code_code, phase_code = code_codes[0], phase_codes[0]
    prev: dict[str, float] | None = None
    samples: list[float] = []
    for epoch in iter_obs_epochs(path, header, systems=[sys_char]):
        current = {}
        for sat in epoch.sats:
            code = sat.values.get(code_code)
            phase = sat.values.get(phase_code)
            if code and phase:
                current[sat.sat] = code - lam * phase
        if prev:
            common = set(current) & set(prev)
            if len(common) >= 4:
                diffs = [current[s] - prev[s] for s in common]
                median = sorted(diffs)[len(diffs) // 2]
                samples.extend(abs(d - median) for d in diffs)
        prev = current
        if len(samples) >= 2000:
            break
    if len(samples) < 8:
        return None
    samples.sort()
    return samples[len(samples) // 2]


AVAILABILITY_MIN_FRACTION = 0.5


def code_availability(path: str, max_epochs: int = 40
                      ) -> dict[tuple[str, str], float]:
    """抽样统计每个 (系统, 归一化观测码) 的非空比例。

    LibGnut 属性表（select_range/select_phase 取末位最优）在某些文件上会
    指向稀疏跟踪码（例如手机基站的 GPS L1M 仅约 25% 卫星有值），静态选择
    该码会丢掉大部分同频数据。返回值用于在属性排序前剔除明显稀疏的候选。

    Returns:
        ``{(系统字符, 观测码): 非空比例}``
    """
    header = parse_obs_header(path)
    total: dict[str, int] = {}
    counts: dict[tuple[str, str], int] = {}
    epochs = 0
    # 与 rinex_improve 改写器/仓库 pyrinrx 解码器的列约定保持一致
    # (观测域起于第 4 列), 使可用性统计与解算实际看到的数据一致。
    for epoch in iter_obs_epochs(path, header, field_offset=_LINE_HEADER_LEN):
        epochs += 1
        for sat in epoch.sats:
            total[sat.sys_char] = total.get(sat.sys_char, 0) + 1
            for code in sat.values:
                counts[(sat.sys_char, code)] = counts.get(
                    (sat.sys_char, code), 0) + 1
        if epochs >= max_epochs:
            break
    availability: dict[tuple[str, str], float] = {}
    for (sys_char, code), count in counts.items():
        base = total.get(sys_char, 0)
        availability[(sys_char, code)] = count / base if base else 0.0
    return availability


def _drop_sparse_codes(sys_char: str, codes: list[str],
                       availability: dict[tuple[str, str], float] | None,
                       ) -> list[str]:
    """按同 (波段, 类型) 分组剔除可用性明显偏低的跟踪码。"""
    if not availability:
        return codes
    groups: dict[tuple[int, str], list[str]] = {}
    for code in codes:
        if len(code) == 3 and code[1].isdigit():
            groups.setdefault((int(code[1]), code[0]), []).append(code)
    drop = set()
    for group in groups.values():
        if len(group) < 2:
            continue
        best = max(availability.get((sys_char, c), 0.0) for c in group)
        for code in group:
            value = availability.get((sys_char, code), 0.0)
            if value < AVAILABILITY_MIN_FRACTION * best and value < 0.5:
                drop.add(code)
    kept = [c for c in codes if c not in drop]
    return kept if kept else codes


def _band_codes_for(path: str, sys_char: str, band: int, tchar: str,
                    header=None) -> list[str]:
    """头部声明中属于该 (系统, 波段, 类型) 的归一化观测码。"""
    if header is None:
        header = parse_obs_header(path)
    codes = []
    for code in header.normalized_types().get(sys_char, []):
        if (len(code) == 3 and code[0] == tchar and code[1].isdigit()
                and int(code[1]) == band and code not in codes):
            codes.append(code)
    return codes


def _order_by_gnut_priority(sys_char: str, bands: list[int]) -> list[int]:
    """按 LibGnut GNSS_BAND_PRIORITY 的波长序排列波段，未声明的排最后。"""
    gs = _gnut_sys(sys_char)
    if gs is None:
        return sorted(bands)
    order = [int(b) for b in GNSS_BAND_PRIORITY.get(gs, []) if int(b) != int(GOBSBAND.BAND)]
    ranked = [b for b in order if b in set(bands)]
    ranked += sorted(b for b in bands if b not in set(ranked))
    return ranked


def default_band_plan(gnss_t=None, nf: int = 2) -> dict[str, list[int]]:
    """无 RINEX 文件信息时的默认波段方案：每系统取 LibGnut 前 nf 个优先波段。

    供 build_params 直接消费最小配置（无 freq_table/freq_ix）时兜底。
    """
    plan: dict[str, list[int]] = {}
    for sys_char in (_system_chars(gnss_t)
                     or ["G", "R", "E", "C", "J", "S"]):
        gs = _gnut_sys(sys_char)
        if gs is None:
            continue
        order = [int(b) for b in GNSS_BAND_PRIORITY.get(gs, [])
                 if int(b) != int(GOBSBAND.BAND)]
        if order:
            plan[sys_char] = order[:max(1, int(nf))]
    return plan

# ---------------------------------------------------------------------------
# 频率映射自动派生（freq_table / freq_ix0 / freq_ix1 / dfreq_glo）
# ---------------------------------------------------------------------------

def auto_freq_plan(band_plan: dict[str, list[int]],
                   freq_table: list[float] | None = None,
                   dfreq_glo: list[float] | None = None,
                   max_freq: int = 2) -> dict:
    """从波段方案派生解算器频率映射（用户无需配置 freq_ix/freq_table）。

    - ``freq_table``：未提供时用 :data:`DEFAULT_FREQ_TABLE`（LibGnut 频率），
      规划波段频率不在表中时按 LibGnut 频率值追加；
    - ``freq_ix0``/``freq_ix1``：``{系统名: freq_table 索引}``。为避免
      ``zdres_sat`` 对 ``nav.obs_idx[f][sys]`` 的无条件访问触发 KeyError，
      即使流中该系统没有第二频点，也以 LibGnut 该系统次优先波段的频率
      兜底填入 slot 1；
    - ``dfreq_glo``：LibGnut R01_STEP/R02_STEP。
    """
    table = [float(f) for f in (freq_table if freq_table else DEFAULT_FREQ_TABLE)]

    def _index_of(freq: float) -> int:
        for i, f in enumerate(table):
            if abs(f - freq) <= _FREQ_MATCH_TOL:
                return i
        table.append(float(freq))
        return len(table) - 1

    freq_ix: list[dict[str, int]] = [{}, {}]
    for sys_char, bands in band_plan.items():
        gs = _gnut_sys(sys_char)
        if gs is None or sys_char not in RINEX_TO_SYSTEM:
            continue
        name = RINEX_TO_SYSTEM[sys_char]
        # LibGnut 波段优先级作为槽位兜底序列（slot f 缺失时取 f+1 优先波段）
        fallback = _order_by_gnut_priority(
            sys_char, [b for b in
                       [int(x) for x in GNSS_BAND_PRIORITY.get(gs, [])
                        if int(x) != int(GOBSBAND.BAND)]])
        for slot in range(max_freq):
            if slot < len(bands):
                band = bands[slot]
            elif slot < len(fallback):
                band = fallback[slot]
            else:
                band = fallback[-1] if fallback else None
            if band is None:
                continue
            freq = band_frequency_hz(sys_char, band)
            if freq > 0:
                freq_ix[slot][name] = _index_of(freq)
    return {
        "freq_table": table,
        "freq_ix0": freq_ix[0],
        "freq_ix1": freq_ix[1],
        "dfreq_glo": [float(x) for x in (dfreq_glo if dfreq_glo
                                         else DEFAULT_DFREQ_GLO)],
    }

# ---------------------------------------------------------------------------
# 配置边界：显式映射优先，否则全自动规划
# ---------------------------------------------------------------------------

def resolve_stream_plan(gnss_cfg: dict, rover_path=None,
                        base_path=None) -> tuple[dict, dict[str, list[int]]]:
    """解析一条流（流动站+基站）的信号方案与频率映射。

    优先级：

    1. 配置显式 ``raw_band_priority``（高级用户出口，仍受波段域校验）；
    2. 遗留 ``freq_ix0``/``freq_ix1`` 元数据；
    3. 都没有 → 由 RINEX 文件头自动规划（LibGnut 优先级）。

    同时把缺失的 ``freq_table``/``freq_ix0``/``freq_ix1``/``dfreq_glo``
    自动补进配置副本（不改传入配置）。返回 ``(cfg_copy, raw_band_priority)``。
    """
    cfg = dict(gnss_cfg)
    nf = int(cfg.get("nf", 2) or 2)
    explicit = cfg.get("raw_band_priority")
    has_legacy = bool(cfg.get("freq_ix0")) or bool(cfg.get("freq_ix1"))
    if explicit is not None or has_legacy:
        priority = resolve_raw_band_priority(cfg)
        if not priority:
            priority = plan_stream_signals(
                rover_path, base_path, cfg.get("gnss_t"), nf)
    else:
        priority = plan_stream_signals(
            rover_path, base_path, cfg.get("gnss_t"), nf)

    if any(key not in cfg for key in
           ("freq_table", "freq_ix0", "freq_ix1", "dfreq_glo")):
        freq_plan = _frequency_plan_scope(priority, rover_path, cfg.get("gnss_t"), nf)
        derived = auto_freq_plan(freq_plan, freq_table=cfg.get("freq_table"),
                                 dfreq_glo=cfg.get("dfreq_glo"), max_freq=max(nf, 2))
        cfg.setdefault("freq_table", derived["freq_table"])
        cfg.setdefault("freq_ix0", derived["freq_ix0"])
        cfg.setdefault("freq_ix1", derived["freq_ix1"])
        cfg.setdefault("dfreq_glo", derived["dfreq_glo"])
    return cfg, priority


def _frequency_plan_scope(plan: dict[str, list[int]],
                          rover_path, gnss_t, nf: int) -> dict[str, list[int]]:
    """频率派生的系统范围：解码到的系统都要有 freq_ix。

    ``zdres_sat`` 对每颗被解码卫星无条件访问 ``nav.obs_idx[f][sys]``；若某系统
    在流动站有观测却没有与基站的共同波段（例如 base 无 GLONASS），自动方案不会
    给它波段，此时若 ``freq_ix`` 也缺该系统就会 KeyError。这里对「流动站声明 ∩
    gnss_t」中缺席的系统补上 LibGnut 默认波段，仅用于频率派生（不改变解码槽位）。
    """
    scope = {sys: list(bands) for sys, bands in plan.items()}
    if not rover_path:
        return scope
    try:
        header = parse_obs_header(str(rover_path))
    except OSError:
        return scope
    wanted = _system_chars(gnss_t)
    for sys_char in header.types:
        if sys_char in scope:
            continue
        if wanted is not None and sys_char not in wanted:
            continue
        gs = _gnut_sys(sys_char)
        if gs is None or sys_char not in RINEX_TO_SYSTEM:
            continue
        bands = [int(b) for b in GNSS_BAND_PRIORITY.get(gs, [])
                 if int(b) != int(GOBSBAND.BAND)][:max(1, nf)]
        if bands:
            scope[sys_char] = bands
    return scope

# ---------------------------------------------------------------------------
# 信号选择（LibGnut GNSS_DATA_PRIORITY 属性优先 + pyrinrx 交织输出）
# ---------------------------------------------------------------------------

def select_codes(sys_char: str, codes: list[str],
                 plan_bands: list[int] | None = None,
                 raw_signal_priority: dict[int, dict[str, str]] | None = None,
                 max_freqs: int = 2,
                 availability: dict[tuple[str, str], float] | None = None,
                 ) -> list[str]:
    """从归一化观测码中选择输出码（LibGnut 逻辑第一位）。

    Args:
        sys_char: RINEX 系统字符（已是 fix_band 归一化后的码表）
        codes: 声明观测码列表（可含重复 band/attr）
        plan_bands: 保留的 raw 波段（LibGnut 优先级序）；None 时按
            LibGnut 波段优先级自动取前 ``max_freqs`` 个
        raw_signal_priority: 可选 GREAT RAW_MIX 风格同频属性覆盖
            ``{raw_band: {code: "CPW", phase: "..."}}``
        max_freqs: plan_bands 缺省时的最大频点数
        availability: 可选 ``{(系统, 观测码): 非空比例}``；提供时先剔除
            明显稀疏的跟踪码（见 :func:`code_availability`），避免静态
            属性选择丢掉大部分同频数据。

    Returns:
        选中的观测码（按波段方案序 + C-L-D-S 交织）。
    """
    gs = _gnut_sys(sys_char)
    if plan_bands is None:
        declared = []
        for code in codes:
            if len(code) == 3 and code[1].isdigit():
                band = int(code[1])
                if band not in declared:
                    declared.append(band)
        plan_bands = _order_by_gnut_priority(sys_char, declared)[:max_freqs]

    selected: list[str] = []
    for band in plan_bands:
        for tchar in _TYPE_ORDER:
            candidates = [c for c in codes
                          if len(c) == 3 and c[0] == tchar
                          and c[1].isdigit() and int(c[1]) == band]
            if not candidates:
                continue
            if availability:
                candidates = _drop_sparse_codes(sys_char, candidates,
                                                availability)
            chosen = candidates[0]
            order = None
            if raw_signal_priority:
                entry = raw_signal_priority.get(band) or {}
                order = entry.get({"C": "code", "L": "phase",
                                   "D": "doppler", "S": "snr"}[tchar])
            if order:
                ranked = [(order.find(c[2]), i, c) for i, c in
                          enumerate(candidates) if c[2] in order]
                # LibGnut select_range 语义：属性优先串中位置最大者胜出
                if ranked:
                    chosen = max(ranked)[2]
            elif gs is not None:
                # LibGnut 路径：select_range/select_phase 属性表（末位最优先）
                chosen = select_attr(gs, GOBSBAND(band), candidates,
                                     phase=(tchar == "L")) or candidates[0]
            if chosen not in selected:
                selected.append(chosen)
    return selected

# ---------------------------------------------------------------------------
# 文件改写（输出 pyrinrx 可直接消费的交织 RINEX）
# ---------------------------------------------------------------------------

_FIELD_WIDTH = 16
_LINE_HEADER_LEN = 4
_SIGS_PER_HEADER_LINE = 13


def _format_obs_types_line(sys_char: str, sigs: list[str]) -> list[str]:
    lines = []
    nsig = len(sigs)
    first = sigs[:_SIGS_PER_HEADER_LINE]
    sig_str = " ".join(f"{s:<3}" for s in first)
    line1 = f"{sys_char}{nsig:5d} {sig_str}".ljust(60) + "SYS / # / OBS TYPES"
    lines.append(line1)
    rest = sigs[_SIGS_PER_HEADER_LINE:]
    while rest:
        batch, rest = rest[:_SIGS_PER_HEADER_LINE], rest[_SIGS_PER_HEADER_LINE:]
        cont = ("      " + " ".join(f"{s:<3}" for s in batch)).ljust(60)
        lines.append(cont + "SYS / # / OBS TYPES")
    return lines


def _scale_lines(header) -> bool:
    """是否存在非 1 的系统级比例因子（需要烤入数值并删除头部行）。"""
    for sys_char, sf in header.scale_factors.items():
        if sf.factor != 1 and not sf.sats:
            return True
    return False


def improve_rinex(input_path: str, output_path: str,
                  band_plan: dict[str, list[int]] | None = None,
                  gnss_t=None,
                  raw_signal_priority: dict[str, dict[int, dict[str, str]]] | None = None,
                  max_freqs: int = 2) -> str:
    """按波段方案改写 RINEX 3 观测文件（rinex_simplifier 的升级版）。

    - 观测码先经 LibGnut ``fix_band`` 归一化（BDS 频带修正）；
    - ``band_plan`` 缺省时按 LibGnut 波段优先级自动选取 ``max_freqs`` 个；
    - 同频多跟踪码按 LibGnut ``GNSS_DATA_PRIORITY`` 属性优先选择；
    - 输出 C-L-D-S 交织、仅含保留频点，rtklib-py ``rnx_decode`` 直接可读；
    - 系统级比例因子非 1 时烤入数值并移除 SCALE FACTOR 头行。

    Returns:
        输出文件路径
    """
    in_path = Path(input_path)
    out_path = Path(output_path)
    header = parse_obs_header(str(in_path))
    wanted = _system_chars(gnss_t)
    scale_bake = _scale_lines(header)
    availability = code_availability(str(in_path))

    sys_selected: dict[str, list[str]] = {}
    sys_old_codes: dict[str, list[str]] = {}
    for sys_char, raw_codes in header.types.items():
        if wanted is not None and sys_char not in wanted:
            continue
        codes = [fix_band(sys_char, c, header.version) for c in raw_codes]
        sys_old_codes[sys_char] = codes
        if band_plan is not None:
            plan = [int(b) for b in (band_plan.get(sys_char) or [])]
            if not plan:
                # 波段方案未覆盖该系统 (如基站无 GLONASS 观测): 与解码器的
                # raw_band_to_slot 保持一致, 直接丢弃。若在此处用 LibGnut
                # 默认波段兜底展开, 改写后的文件会保留解码器不支持的系统,
                # 触发 unsupported_raw_band 硬失败 (R1 回归实测)。
                continue
        else:
            plan = None
        signal_priority = None
        if raw_signal_priority:
            entry = raw_signal_priority.get(sys_char) or \
                raw_signal_priority.get(RINEX_TO_SYSTEM.get(sys_char, ""), {})
            signal_priority = {int(k): v for k, v in (entry or {}).items()}
        selected = select_codes(sys_char, codes, plan or None,
                                raw_signal_priority=signal_priority,
                                max_freqs=max_freqs,
                                availability=availability)
        if selected:
            sys_selected[sys_char] = selected

    with open(in_path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()

    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        skip_until_cont = 0
        header_end_idx = len(lines) - 1
        for idx, line in enumerate(lines):
            label = line[60:].rstrip("\r\n") if len(line) >= 60 else ""
            if label.startswith("END OF HEADER"):
                f.write(line)
                header_end_idx = idx
                break
            if label.startswith("SYS / # / OBS TYPES") and line[0] != " ":
                sys_char = line[0]
                try:
                    nsig = int(line[3:6])
                except ValueError:
                    nsig = 0
                n_cont = max(0, -(-max(nsig - _SIGS_PER_HEADER_LINE, 0)
                                  // _SIGS_PER_HEADER_LINE))
                if sys_char in sys_selected:
                    for out_line in _format_obs_types_line(
                            sys_char, sys_selected[sys_char]):
                        f.write(out_line + "\n")
                elif wanted is None:
                    # 未指定 gnss_t: 原样保留（含续行）
                    f.write(line)
                    continue
                # 其余情况（gnss_t 已指定但该系统不在列表内）: 连同续行丢弃。
                # 必须丢弃——pyrinrx 解码器每系统最多解析 26 个观测码, 保留
                # 声明 28 个观测码的系统（如 WUH2 基站的 C）会越界崩溃。
                skip_until_cont = n_cont
                continue
            if skip_until_cont > 0 and label.startswith("SYS / # / OBS TYPES"):
                skip_until_cont -= 1
                continue
            if label.startswith("SYS / SCALE FACTOR") and scale_bake:
                continue  # 比例因子已烤入数值
            f.write(line)

        def _rewrite_row(line: str) -> str:
            """按选择的信号表重写一条卫星观测行。"""
            sys_char = line[0]
            old_codes = sys_old_codes[sys_char]
            new_codes = sys_selected[sys_char]
            old_to_new: list[int | None] = [None] * len(old_codes)
            for i, code in enumerate(old_codes):
                if code in new_codes:
                    slot = new_codes.index(code)
                    # 同一 normalized 码只保留第一个声明位置
                    if slot not in old_to_new:
                        old_to_new[i] = slot
            body = line[_LINE_HEADER_LEN:].rstrip("\r\n")
            n_new = len(new_codes)
            new_fields = [" " * _FIELD_WIDTH] * n_new
            factor = 1.0
            if scale_bake:
                sf = header.scale_factors.get(sys_char)
                if sf is not None and sf.factor != 1 and not sf.sats:
                    factor = float(sf.factor)
            for old_i, new_i in enumerate(old_to_new):
                if new_i is None:
                    continue
                start = _FIELD_WIDTH * old_i
                field = body[start:start + _FIELD_WIDTH]
                if factor != 1.0 and field.strip():
                    try:
                        value = float(field.split()[0]) * factor
                        keep = (field[14:] if len(field) > 14 else "").ljust(2)
                        field = f"{value:14.3f}" + keep
                    except (ValueError, IndexError):
                        pass
                new_fields[new_i] = field.ljust(_FIELD_WIDTH)
            return line[:_LINE_HEADER_LEN] + "".join(new_fields) + "\n"

        # 数据段按历元块处理: 丢弃卫星时同步修正历元行的卫星计数
        idx = header_end_idx + 1
        while idx < len(lines):
            line = lines[idx]
            if not line.strip():
                f.write(line if line.endswith("\n") else line + "\n")
                idx += 1
                continue
            if line[0] != ">":
                idx += 1
                continue
            try:
                nsat = int(line[32:35])
            except ValueError:
                nsat = 0
            block = lines[idx + 1: idx + 1 + nsat]
            if wanted is not None:
                # gnss_t 已指定: 只保留被选中系统的卫星行, 并修正卫星数
                block = [row for row in block
                         if row.strip() and row[0] in sys_selected]
                line = f"{line[:32]}{len(block):3d}{line[35:]}"
            f.write(line if line.endswith("\n") else line + "\n")
            for row in block:
                f.write(_rewrite_row(row))
            idx += 1 + nsat

    return str(out_path)


def needs_improvement(path: str,
                      band_plan: dict[str, list[int]] | None = None,
                      gnss_t=None,
                      raw_signal_priority: dict | None = None,
                      max_freqs: int = 2) -> bool:
    """判断文件是否需要按波段方案改写（rinex_improve 版判定）。

    只要「选中的信号表」与「声明的信号表」不一致（频点超限、含非优先
    波段、同频多跟踪码、LibGnut BDS 频带归一化等任一情形），即需改写。
    """
    if not Path(path).exists():
        return False
    header = parse_obs_header(path)
    wanted = _system_chars(gnss_t)
    if _scale_lines(header):
        return True
    availability = code_availability(path)
    for sys_char, raw_codes in header.types.items():
        if wanted is not None and sys_char not in wanted:
            continue
        codes = [fix_band(sys_char, c, header.version) for c in raw_codes]
        plan = None
        if band_plan is not None:
            plan = [int(b) for b in (band_plan.get(sys_char) or [])] or None
        signal_priority = None
        if raw_signal_priority:
            entry = raw_signal_priority.get(sys_char) or \
                raw_signal_priority.get(RINEX_TO_SYSTEM.get(sys_char, ""), {})
            signal_priority = {int(k): v for k, v in (entry or {}).items()}
        selected = select_codes(sys_char, codes, plan,
                                raw_signal_priority=signal_priority,
                                max_freqs=max_freqs,
                                availability=availability)
        if selected != codes:
            return True
    return False

# ---------------------------------------------------------------------------
# 兼容包装：旧 simplify_rinex 调用风格
# ---------------------------------------------------------------------------

def simplify_rinex(input_path: str, output_path: str, max_freqs: int = 2,
                   freq_priority: dict[str, list[int]] = None,
                   raw_band_priority: dict[str, list[int]] = None,
                   raw_signal_priority: dict[str, dict] = None,
                   band_plan: dict[str, list[int]] = None) -> str:
    """兼容入口：签名与 rinex_simplifier.simplify_rinex 一致。

    ``raw_band_priority``（raw 波段序）直接作为波段方案传给
    :func:`improve_rinex`；``freq_priority``（normalized 序，旧 API）不再
    推荐使用，仅做 Best-effort 转换（按 raw band 数字排序重建方案）。
    """
    if raw_band_priority is not None and band_plan is None:
        band_plan = {sys: [int(b) for b in bands]
                     for sys, bands in raw_band_priority.items()}
    elif freq_priority is not None and band_plan is None:
        # 旧 normalized 语义：无法无损还原 raw band，退化为按声明波段序
        header = parse_obs_header(input_path)
        band_plan = {}
        for sys_char, codes in header.types.items():
            declared = sorted({int(c[1]) for c in codes
                               if len(c) == 3 and c[1].isdigit()})
            band_plan[sys_char] = declared[:max_freqs]
    return improve_rinex(input_path, output_path, band_plan=band_plan,
                         raw_signal_priority=raw_signal_priority,
                         max_freqs=max_freqs)
