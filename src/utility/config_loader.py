"""YAML 配置加载与校验。"""
from pathlib import Path

import numpy as np
import yaml


REQUIRED_DATA_RATE = 100  # external/INS 模式仅支持 100Hz
SUPPORTED_EXTERNAL_FORMATS = {"pos", "awesome_pos"}
SUPPORTED_GNSS_SOURCES = {"external", "awesome_external", "internal"}
SUPPORTED_POSITIONING_MODES = {"spp", "rtd", "rtk"}
SUPPORTED_INS_ENABLED = {"lc", "off", "tc"}
SUPPORTED_RB_FORMATS = {"xyz", "llh"}
SUPPORTED_POS_FORMATS = {"llh", "xyz"}
SUPPORTED_TIME_FORMATS = {"gpst", "datetime"}
SUPPORTED_TRACE_LEVELS = {0, 1, 2, 3}

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

    # gnss_source 校验
    gnss_source = cfg["gnss"]["gnss_source"]
    if gnss_source not in SUPPORTED_GNSS_SOURCES:
        raise ValueError(
            f"gnss_source must be one of {SUPPORTED_GNSS_SOURCES}, "
            f"got '{gnss_source}'"
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
