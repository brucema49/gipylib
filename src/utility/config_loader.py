"""YAML 配置加载与校验。"""
from pathlib import Path

import numpy as np
import yaml


REQUIRED_DATA_RATE = 100  # external/INS 模式仅支持 100Hz
SUPPORTED_EXTERNAL_FORMATS = {"pos", "awesome_pos"}
SUPPORTED_GNSS_SOURCES = {"external", "awesome_external", "internal"}
SUPPORTED_POSITIONING_MODES = {"spp", "rtd", "rtk"}
SUPPORTED_INS_ENABLED = {"lc", "off", "tc"}
SUPPORTED_IMU_DATA_FORMS = {"rate", "increment"}
SUPPORTED_RB_FORMATS = {"xyz", "llh"}
SUPPORTED_POS_FORMATS = {"llh", "xyz"}
SUPPORTED_TIME_FORMATS = {"gpst", "datetime"}
SUPPORTED_TRACE_LEVELS = {0, 1, 2, 3}
SUPPORTED_MEASUREMENT_TRACE_FORMATS = {"jsonl", "csv"}
SUPPORTED_INIT_PROPAGATION_TRACE_FORMATS = {"jsonl", "csv"}
SUPPORTED_INITIALIZATION_INPUT_TRACE_FORMATS = {"jsonl", "csv"}

# --- 松紧组合差异参数 -------------------------------------------------------
# ``ins`` 段保存松紧组合**共用**的参数 (usually); ``ins_tc`` / ``ins_lc`` 只写
#   该模式专属、或需要覆盖共用值的参数。加载时按 ``ins.enabled`` 把对应段合并进
#   ``cfg["ins"]``, 未激活的段不生效 (允许同一文件同时描述两种模式)。
#
# 语义区分的依据是"是否存在独立 GNSS PVT 观测行":
#   松组合 (LC) 有独立的位置/速度观测, 因此可以 (也必须) 配置观测噪声、
#   位置差分速度、以及为稳定 P 而额外注入的 pos_psd/vel_psd;
#   紧组合 (TC) 只有 DD 伪距/相位观测, 位置与速度不确定度必须完全由
#   Q(IMU 噪声) 与 H/R 传播得到, 额外注入 pos_psd/vel_psd 会破坏与 GREAT
#   的等价性 (见 issue/9-14机械编排发散.md P5)。
INS_MODE_SECTIONS = {"lc": "ins_lc", "tc": "ins_tc"}

# 仅松组合: 依赖独立 GNSS PVT(位置/速度)观测, 紧组合没有这类观测行。
INS_LC_ONLY_KEYS = frozenset({
    # 额外的位置/速度随机游走 (LC 稳定项; TC 必须保持 0, 故直接禁止出现)
    "pos_psd",
    "vel_psd",
    # GNSS PVT 观测噪声与缩放
    "gnss_vel_std",
    "gnss_sd_scale",
    "gnss_sd_axis_scale",
    "gnss_time_sync_noise_s",
    "use_reported_gnss_sd",
    "vertical_sigma_factor",
    # 位置差分速度观测
    "pos_diff_vel_std",
    "position_diff_velocity_update",
    # 位置创新拒绝 (LC 观测级粗差剔除, lc_estimator.py:81-82)
    "innov_reject_threshold",
    "innov_reject_warmup",
})

# 已废弃的死键: 仓库内无任何消费点 (历史上属于 LC 语义)。出现即报错,
# 避免误以为仍在生效; 消费它们的代码如恢复, 应同时从本清单移除。
DEPRECATED_INS_KEYS = frozenset({
    "rtk_float_pos_std",      # 原意: RTK FLOAT 位置 sigma 下限; 无消费点
    "pos_diff_vel_max_std",   # 原意: 直接速度超阈改用位置差分; 无消费点
})

# 仅紧组合
INS_TC_ONLY_KEYS = frozenset({
    "tc_use_doppler",       # 紧组合 Doppler 观测开关 (LC 无此观测)
})

# 共用参数的归属修正记录 (供 manual.md 与配置注释引用):
#   nhc_warmup 同时被 lc_integration.py:108 与 tc_integration.py:143 消费,
#   属于共用参数, 不在本清单; 放在 ins_lc/ins_tc 段的 nhc_warmup 只在对应
#   模式生效, 需要两种模式共用时必须写在共用 ins 段。

_WGS84_A = 6378137.0
_WGS84_F = 1 / 298.257223563
_WGS84_B = _WGS84_A * (1 - _WGS84_F)
_WGS84_E2 = _WGS84_F * (2 - _WGS84_F)


def llh_to_ecef(llh):
    """[lat_deg, lon_deg, h] → ECEF [x, y, z] (m)。"""
    lat = np.radians(float(llh[0]))
    lon = np.radians(float(llh[1]))
    h = float(llh[2])
    sinlat = np.sin(lat)
    coslat = np.cos(lat)
    N = _WGS84_A / np.sqrt(1 - _WGS84_E2 * sinlat * sinlat)
    x = (N + h) * coslat * np.cos(lon)
    y = (N + h) * coslat * np.sin(lon)
    z = (N * (1 - _WGS84_E2) + h) * sinlat
    return np.array([x, y, z], dtype=np.float64)


def _parse_bool(value, field_name: str) -> bool:
    """解析 YAML 布尔字段，接受 bool/int 与常见字符串写法。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        if value in (0, 1):
            return bool(value)
        raise ValueError(f"{field_name} must be a boolean, got {value!r}")
    text = str(value).strip().lower()
    if text in ("true", "1", "yes", "on"):
        return True
    if text in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"{field_name} must be a boolean, got {value!r}")


def _normalize_rb(gnss_cfg: dict) -> None:
    """基站坐标归一化: rb_format=llh 时转 ECEF xyz，原地修改 gnss_cfg['rb']。

    rb 全 0 表示用 RINEX 头，保留不变。
    """
    rb = gnss_cfg.get("rb")
    if rb is None or all(float(v) == 0.0 for v in rb):
        return
    rb_format = gnss_cfg.get("rb_format", "xyz")
    if rb_format not in SUPPORTED_RB_FORMATS:
        raise ValueError(
            f"rb_format must be one of {SUPPORTED_RB_FORMATS}, got '{rb_format}'"
        )
    if rb_format == "llh":
        if len(rb) != 3:
            raise ValueError("rb (llh) must have 3 elements: [lat_deg, lon_deg, h]")
        ecef = llh_to_ecef(rb)
        gnss_cfg["rb"] = [float(v) for v in ecef]


def _validate_output(output_cfg: dict) -> None:
    """校验 output 段格式选项。"""
    pos_fmt = output_cfg.get("position_format", "llh")
    if pos_fmt not in SUPPORTED_POS_FORMATS:
        raise ValueError(
            f"output.position_format must be one of {SUPPORTED_POS_FORMATS}, "
            f"got '{pos_fmt}'"
        )
    time_fmt = output_cfg.get("time_format", "gpst")
    if time_fmt not in SUPPORTED_TIME_FORMATS:
        raise ValueError(
            f"output.time_format must be one of {SUPPORTED_TIME_FORMATS}, "
            f"got '{time_fmt}'"
        )
    trace_level = int(output_cfg.get("trace_level", 0))
    if trace_level not in SUPPORTED_TRACE_LEVELS:
        raise ValueError(
            f"output.trace_level must be one of {SUPPORTED_TRACE_LEVELS}, "
            f"got '{trace_level}'"
        )


def _validate_measurement_trace(tc_cfg: dict) -> None:
    """Normalize the opt-in TC measurement graph trace configuration."""
    trace_cfg = tc_cfg.setdefault("measurement_trace", {})
    if trace_cfg is None:
        trace_cfg = {}
        tc_cfg["measurement_trace"] = trace_cfg
    if not isinstance(trace_cfg, dict):
        raise ValueError("tc.measurement_trace must be a mapping")

    trace_cfg["enabled"] = _parse_bool(
        trace_cfg.get("enabled", False), "tc.measurement_trace.enabled")
    fmt = str(trace_cfg.get("format", "jsonl")).strip().lower()
    if fmt not in SUPPORTED_MEASUREMENT_TRACE_FORMATS:
        raise ValueError(
            "tc.measurement_trace.format must be one of "
            f"{SUPPORTED_MEASUREMENT_TRACE_FORMATS}, got '{fmt}'"
        )
    trace_cfg["format"] = fmt
    default_name = f"tc-measurement-trace.{fmt}"
    filename = str(trace_cfg.get("filename", default_name))
    if (not filename or filename in (".", "..") or
            Path(filename).name != filename or "/" in filename or
            "\\" in filename or Path(filename).is_absolute()):
        raise ValueError("tc.measurement_trace.filename must be a basename")
    trace_cfg["filename"] = filename


def _validate_init_propagation_trace(tc_cfg: dict) -> None:
    """Normalize the opt-in initialization/propagation trace configuration."""
    trace_cfg = tc_cfg.setdefault("init_propagation_trace", {})
    if trace_cfg is None:
        trace_cfg = {}
        tc_cfg["init_propagation_trace"] = trace_cfg
    if not isinstance(trace_cfg, dict):
        raise ValueError("tc.init_propagation_trace must be a mapping")

    trace_cfg["enabled"] = _parse_bool(
        trace_cfg.get("enabled", False),
        "tc.init_propagation_trace.enabled")
    fmt = str(trace_cfg.get("format", "jsonl")).strip().lower()
    if fmt not in SUPPORTED_INIT_PROPAGATION_TRACE_FORMATS:
        raise ValueError(
            "tc.init_propagation_trace.format must be one of "
            f"{SUPPORTED_INIT_PROPAGATION_TRACE_FORMATS}, got '{fmt}'")
    trace_cfg["format"] = fmt
    default_name = f"tc-init-propagation-trace.{fmt}"
    filename = str(trace_cfg.get("filename", default_name))
    if (not filename or filename in (".", "..") or
            Path(filename).name != filename or "/" in filename or
            "\\" in filename or Path(filename).is_absolute()):
        raise ValueError(
            "tc.init_propagation_trace.filename must be a basename")
    trace_cfg["filename"] = filename


def _validate_initialization_input_trace(tc_cfg: dict) -> None:
    """Normalize the opt-in common initialization-input trace configuration."""
    trace_cfg = tc_cfg.setdefault("initialization_input_trace", {})
    if trace_cfg is None:
        trace_cfg = {}
        tc_cfg["initialization_input_trace"] = trace_cfg
    if not isinstance(trace_cfg, dict):
        raise ValueError("tc.initialization_input_trace must be a mapping")

    trace_cfg["enabled"] = _parse_bool(
        trace_cfg.get("enabled", False),
        "tc.initialization_input_trace.enabled")
    fmt = str(trace_cfg.get("format", "jsonl")).strip().lower()
    if fmt not in SUPPORTED_INITIALIZATION_INPUT_TRACE_FORMATS:
        raise ValueError(
            "tc.initialization_input_trace.format must be one of "
            f"{SUPPORTED_INITIALIZATION_INPUT_TRACE_FORMATS}, got '{fmt}'")
    trace_cfg["format"] = fmt
    default_name = f"tc-initialization-input-trace.{fmt}"
    filename = str(trace_cfg.get("filename", default_name))
    if (not filename or filename in (".", "..") or
            Path(filename).name != filename or "/" in filename or
            "\\" in filename or Path(filename).is_absolute()):
        raise ValueError(
            "tc.initialization_input_trace.filename must be a basename")
    trace_cfg["filename"] = filename


def _merge_mode_specific_ins(cfg: dict, ins_enabled: str) -> None:
    """把 ``ins_tc`` / ``ins_lc`` 合并进 ``cfg["ins"]`` 并做跨模式校验。

    三段语义:

    - ``ins``    : 松紧组合**共用**参数 (usually)。
    - ``ins_tc`` : 仅 ``ins.enabled='tc'`` 生效 (专属项或覆盖共用值)。
    - ``ins_lc`` : 仅 ``ins.enabled='lc'`` 生效。

    ``ins.enabled='off'`` 时两段都不生效。

    Raises:
        ValueError: 段不是 mapping；段内写了 ``enabled``；段内出现了另一个
            模式的专属键；共用段出现了当前模式禁止的键；或任何 ins 段出现
            废弃死键。
    """

    if not isinstance(cfg.get("ins", {}), dict):
        raise ValueError("ins must be a mapping")
    active_section = INS_MODE_SECTIONS.get(ins_enabled)
    forbidden_in_common = {
        "lc": INS_TC_ONLY_KEYS,
        "tc": INS_LC_ONLY_KEYS,
    }.get(ins_enabled, frozenset())

    sections_to_check = [("ins", cfg["ins"])] + [
        (section, cfg[section])
        for section in INS_MODE_SECTIONS.values()
        if isinstance(cfg.get(section), dict)
    ]
    for name, block in sections_to_check:
        dead = sorted(DEPRECATED_INS_KEYS.intersection(block))
        if dead:
            raise ValueError(
                f"{name} contains deprecated keys with no consumer in the "
                f"codebase: {', '.join(dead)} (see DEPRECATED_INS_KEYS)"
            )

    for mode, section in INS_MODE_SECTIONS.items():
        block = cfg.get(section)
        if block is None:
            continue
        if not isinstance(block, dict):
            raise ValueError(f"{section} must be a mapping")
        if "enabled" in block:
            raise ValueError(
                f"{section}.enabled is not allowed; ins.enabled selects the mode"
            )
        # 专属段内部也不允许混入另一个模式的参数, 便于提前发现误用。
        wrong = (INS_LC_ONLY_KEYS if mode == "tc" else INS_TC_ONLY_KEYS)
        stray = sorted(wrong.intersection(block))
        if stray:
            raise ValueError(
                f"{section} may not contain "
                f"{'loose-coupling' if mode == 'tc' else 'tight-coupling'}"
                f"-only keys: {', '.join(stray)}"
            )
        if section == active_section:
            cfg["ins"].update(block)

    if forbidden_in_common:
        stray = sorted(forbidden_in_common.intersection(cfg["ins"]))
        if stray:
            other = "紧组合" if ins_enabled == "lc" else "松组合"
            raise ValueError(
                f"ins.enabled='{ins_enabled}': 以下参数只适用于{other}, "
                f"不能出现在共用 ins 段: {', '.join(stray)}"
            )


def load_config(path) -> dict:
    """加载 YAML 配置并做必要校验。

    Raises:
        FileNotFoundError: 文件不存在
        yaml.YAMLError: YAML 解析错误
        ValueError: 配置非法
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    # 禁止使用 coupling_mode (改用 ins.enabled 控制)
    if cfg.get("coupling_mode") is not None:
        raise ValueError(
            "coupling_mode 已废弃, 请删除该字段并使用 ins.enabled 控制: "
            "off=纯GNSS / lc=松组合 / tc=紧组合"
        )

    # ins.enabled 必填
    ins_enabled = cfg.get("ins", {}).get("enabled")
    if ins_enabled is None:
        raise ValueError("ins.enabled is required (must be 'lc', 'off', or 'tc')")
    if ins_enabled not in SUPPORTED_INS_ENABLED:
        raise ValueError(
            f"ins.enabled must be one of {SUPPORTED_INS_ENABLED}, "
            f"got '{ins_enabled}'"
        )

    # 按 ins.enabled 合并模式专属段, 并禁止跨模式误用参数。必须在任何
    # ins_cfg.get(...) 之前执行, 否则消费方会读到未合并的共用值。
    _merge_mode_specific_ins(cfg, ins_enabled)

    # gnss_source 校验
    gnss_source = cfg["gnss"]["gnss_source"]
    if gnss_source not in SUPPORTED_GNSS_SOURCES:
        raise ValueError(
            f"gnss_source must be one of {SUPPORTED_GNSS_SOURCES}, "
            f"got '{gnss_source}'"
        )

    # Validate the reader-facing raw-band domain at the configuration
    # boundary.  This deliberately resolves legacy (system, freq_ix) metadata
    # without looking at frequency values; the solver indices and RINEX bands
    # are separate namespaces.
    from src.stream.gnss_band_mapping import resolve_raw_band_priority
    resolve_raw_band_priority(cfg.get("gnss", {}))

    # 卫星剔除: remove_sat 为规范键（用户面向），excsats 为遗留别名，
    # 两者可共存（合并剔除），消费点在 rtklib_config_adapter.build_params。
    for key in ("remove_sat", "excsats"):
        value = cfg["gnss"].get(key)
        if value is None:
            continue
        if (not isinstance(value, list)
                or not all(isinstance(item, str) for item in value)):
            raise ValueError(
                f"gnss.{key} must be a list of satellite id strings "
                f"(e.g. [\"C01\"]), got {value!r}"
            )

    # 纯 GNSS 模式 (ins.enabled=off) 必须 internal, 不接受外部结果
    if ins_enabled == "off" and gnss_source != "internal":
        raise ValueError(
            "gnss_source must be 'internal' when ins.enabled='off' "
            "(纯 GNSS 模式不支持外部结果输入)"
        )

    # 基站坐标格式归一化 (rb_format=llh → xyz)
    _normalize_rb(cfg["gnss"])

    # output 段格式校验
    output_cfg = cfg.setdefault("output", {})
    _validate_output(output_cfg)
    tc_cfg = cfg.setdefault("tc", {})
    if not isinstance(tc_cfg, dict):
        raise ValueError("tc must be a mapping")
    _validate_measurement_trace(tc_cfg)
    _validate_init_propagation_trace(tc_cfg)
    _validate_initialization_input_trace(tc_cfg)

    # ``imu_data_form`` 描述传感器文件的数值语义，``rate_to_increment`` 描述
    # 是否在机械编排边界把 rate 一次性转成增量。``imu_data_process_form`` 是
    # 派生字段，保留给 LC/TC 既有消费方，避免两处开关各自演化。
    ins_cfg = cfg.setdefault("ins", {})
    imu_format = str(ins_cfg.get("imu_format", "gpst")).lower()
    inferred_form = "increment" if imu_format == "awesome_increment" else "rate"
    imu_data_form = str(ins_cfg.get("imu_data_form", inferred_form)).lower()
    if imu_data_form not in SUPPORTED_IMU_DATA_FORMS:
        raise ValueError(
            f"ins.imu_data_form must be one of {SUPPORTED_IMU_DATA_FORMS}, "
            f"got '{imu_data_form}'"
        )
    if imu_format == "awesome_increment" and imu_data_form != "increment":
        raise ValueError(
            "ins.imu_format='awesome_increment' requires "
            "ins.imu_data_form='increment'"
        )

    legacy_process_form = ins_cfg.get("imu_data_process_form")
    if legacy_process_form is not None:
        legacy_process_form = str(legacy_process_form).lower()
        if legacy_process_form not in SUPPORTED_IMU_DATA_FORMS:
            raise ValueError(
                "ins.imu_data_process_form must be one of "
                f"{SUPPORTED_IMU_DATA_FORMS}, got '{legacy_process_form}'"
            )
        if imu_data_form == "increment" and legacy_process_form == "rate":
            raise ValueError(
                "increment IMU input must use "
                "ins.imu_data_process_form='increment'"
            )

    declared_switch = ins_cfg.get("rate_to_increment")
    if declared_switch is None:
        if imu_data_form == "increment":
            rate_to_increment = False
        else:
            rate_to_increment = (legacy_process_form == "increment")
    else:
        rate_to_increment = _parse_bool(
            declared_switch, "ins.rate_to_increment")
        if imu_data_form == "increment" and rate_to_increment:
            raise ValueError(
                "ins.rate_to_increment requires rate input; increment IMU "
                "payloads must not be converted twice"
            )
        if legacy_process_form is not None:
            expected = "increment" if rate_to_increment else "rate"
            if legacy_process_form != expected:
                raise ValueError(
                    "ins.imu_data_process_form conflicts with "
                    "ins.rate_to_increment"
                )

    if imu_data_form == "increment":
        imu_data_process_form = "increment"
    elif legacy_process_form is not None:
        imu_data_process_form = legacy_process_form
    else:
        imu_data_process_form = "increment" if rate_to_increment else "rate"

    ins_cfg["imu_data_form"] = imu_data_form
    ins_cfg["rate_to_increment"] = bool(rate_to_increment)
    ins_cfg["imu_data_process_form"] = imu_data_process_form

    # external/awesome_external 模式校验
    if gnss_source in ("external", "awesome_external"):
        if ins_enabled != "lc":
            raise ValueError(
                "ins.enabled must be 'lc' when gnss_source='external' "
                "(external GNSS requires integrated navigation path)"
            )
        data_rate = cfg["ins"]["data_rate"]
        if gnss_source == "external" and data_rate != REQUIRED_DATA_RATE:
            raise ValueError(
                f"IMU data_rate must be {REQUIRED_DATA_RATE}, got {data_rate}. "
                f"Current external POS stage only supports 100Hz IMU."
            )
        if gnss_source == "awesome_external" and float(data_rate) <= 0.0:
            raise ValueError(
                f"Awesome IMU data_rate must be positive, got {data_rate}."
            )
        fmt = cfg["gnss"].get("external_sol_format", "pos")
        if fmt not in SUPPORTED_EXTERNAL_FORMATS:
            raise ValueError(
                f"external_sol_format must be one of {SUPPORTED_EXTERNAL_FORMATS}, "
                f"got '{fmt}'"
            )
        if gnss_source == "awesome_external":
            if fmt != "awesome_pos":
                raise ValueError(
                    "awesome_external requires external_sol_format='awesome_pos'"
                )
            if cfg["ins"].get("imu_format") != "awesome_increment":
                raise ValueError(
                    "awesome_external requires ins.imu_format='awesome_increment'"
                )

    # internal 模式校验
    if gnss_source == "internal":
        pos_mode = cfg["gnss"].get("positioning_mode")
        if pos_mode is None:
            raise ValueError(
                "positioning_mode is required when gnss_source='internal'"
            )
        if pos_mode not in SUPPORTED_POSITIONING_MODES:
            raise ValueError(
                f"positioning_mode must be one of {SUPPORTED_POSITIONING_MODES}, "
                f"got '{pos_mode}'"
            )
        if not cfg["gnss"].get("rover_path"):
            raise ValueError("rover_path is required when gnss_source='internal'")
        if not cfg["gnss"].get("eph_path"):
            raise ValueError("eph_path is required when gnss_source='internal'")
        if pos_mode in ("rtk", "rtd") and not cfg["gnss"].get("base_path"):
            raise ValueError(
                f"base_path is required when positioning_mode='{pos_mode}'"
            )
        # INS 模式 (lc/tc) 需要 IMU 数据
        if ins_enabled in ("lc", "tc"):
            if not cfg["ins"].get("imu_data_path"):
                raise ValueError(
                    f"ins.imu_data_path is required when gnss_source='internal' "
                    f"and ins.enabled='{ins_enabled}'"
                )

    return cfg
