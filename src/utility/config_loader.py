"""YAML 配置加载与校验。"""
from pathlib import Path

import yaml


REQUIRED_DATA_RATE = 100  # 当前阶段仅支持 100Hz
SUPPORTED_EXTERNAL_FORMATS = {"pos"}  # 当前阶段仅支持 POS 格式


def load_config(path) -> dict:
    """加载 YAML 配置并做必要校验。

    Args:
        path: 配置文件路径（str 或 Path）

    Returns:
        配置字典

    Raises:
        FileNotFoundError: 文件不存在
        yaml.YAMLError: YAML 解析错误
        ValueError: data_rate / gnss_source / external_sol_format 不符合要求
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    # IMU 频率检查
    data_rate = cfg["ins"]["data_rate"]
    if data_rate != REQUIRED_DATA_RATE:
        raise ValueError(
            f"IMU data_rate must be {REQUIRED_DATA_RATE}, got {data_rate}. "
            f"Current stage only supports 100Hz IMU."
        )

    # 数据源模式检查
    if cfg["gnss"]["gnss_source"] != "external":
        raise ValueError(
            f"gnss_source must be 'external' in current stage, "
            f"got '{cfg['gnss']['gnss_source']}'"
        )

    # 外部结果格式检查
    fmt = cfg["gnss"].get("external_sol_format", "pos")
    if fmt not in SUPPORTED_EXTERNAL_FORMATS:
        raise ValueError(
            f"external_sol_format must be one of {SUPPORTED_EXTERNAL_FORMATS}, "
            f"got '{fmt}'"
        )

    return cfg
