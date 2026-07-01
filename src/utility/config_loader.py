"""YAML 配置加载与校验。"""
from pathlib import Path

import yaml


REQUIRED_DATA_RATE = 100  # external/INS 模式仅支持 100Hz
SUPPORTED_EXTERNAL_FORMATS = {"pos"}
SUPPORTED_GNSS_SOURCES = {"external", "internal"}
SUPPORTED_POSITIONING_MODES = {"spp", "rtk"}
SUPPORTED_INS_ENABLED = {"on", "off"}


def load_config(path) -> dict:
    """加载 YAML 配置并做必要校验。

    Raises:
        FileNotFoundError: 文件不存在
        yaml.YAMLError: YAML 解析错误
        ValueError: 配置非法
        NotImplementedError: internal + ins.enabled=on（INS 未实现）
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    # ins.enabled 必填
    ins_enabled = cfg.get("ins", {}).get("enabled")
    if ins_enabled is None:
        raise ValueError("ins.enabled is required (must be 'on' or 'off')")
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

    # external 模式校验
    if gnss_source == "external":
        if ins_enabled != "on":
            raise ValueError(
                "ins.enabled must be 'on' when gnss_source='external' "
                "(external GNSS requires integrated navigation path)"
            )
        data_rate = cfg["ins"]["data_rate"]
        if data_rate != REQUIRED_DATA_RATE:
            raise ValueError(
                f"IMU data_rate must be {REQUIRED_DATA_RATE}, got {data_rate}. "
                f"Current stage only supports 100Hz IMU."
            )
        fmt = cfg["gnss"].get("external_sol_format", "pos")
        if fmt not in SUPPORTED_EXTERNAL_FORMATS:
            raise ValueError(
                f"external_sol_format must be one of {SUPPORTED_EXTERNAL_FORMATS}, "
                f"got '{fmt}'"
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
        if pos_mode == "rtk" and not cfg["gnss"].get("base_path"):
            raise ValueError(
                "base_path is required when positioning_mode='rtk'"
            )
        if ins_enabled == "on":
            raise NotImplementedError(
                "INS estimator not implemented: gnss_source='internal' + "
                "ins.enabled='on' is reserved for future INS integration"
            )

    return cfg
