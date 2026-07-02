"""GInsStream 主入口。

用法:
    python src/main.py [config_path]

默认 config_path = data/config.yaml
"""
import sys
from pathlib import Path
from queue import Queue

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.core.thread_control import ThreadControl
from src.stream.factory import SensorFactory
from src.log.aligned_writer import AlignedWriter
from src.log.aligner import Aligner
from src.log.logger import Logger
from src.log.solution_writer import SolutionWriter
from src.log.solution_logger import SolutionLogger
from src.utility.config_loader import load_config


def main(config_path: str = "data/config.yaml"):
    config = load_config(config_path)

    control = ThreadControl()
    imu_queue = Queue(maxsize=2000)
    gnss_queue = Queue(maxsize=100)

    sensors, logger = _assemble_pipeline(config, control, imu_queue, gnss_queue)

    for s in sensors:
        s.start()
    logger.start()

    logger.join()
    control.shutdown()
    for s in sensors:
        s.join(timeout=2)


def _assemble_pipeline(config, control, imu_queue, gnss_queue):
    """根据配置选择 sensors 与 logger。

    Returns:
        (sensors, logger) 元组
    """
    gnss_source = config["gnss"]["gnss_source"]
    ins_enabled = config["ins"]["enabled"]

    if gnss_source == "external":
        # 路径 A: 现有块状对齐输出
        sensors = SensorFactory.create_sensors(config, imu_queue, gnss_queue, control)
        writer = AlignedWriter(output_dir=config["output"]["output_dir"])
        aligner = Aligner(imu_dt=1.0 / config["ins"]["data_rate"])
        logger = Logger(imu_queue, gnss_queue, writer, aligner, control)
        return sensors, logger

    if gnss_source == "internal" and ins_enabled == "off":
        # 路径 B: 纯 GNSS .pos 输出
        sensors = SensorFactory.create_sensors(config, imu_queue, gnss_queue, control)
        filename = config["output"].get("solution_filename", "solution.pos")
        writer = SolutionWriter(output_dir=config["output"]["output_dir"],
                                filename=filename)
        logger = SolutionLogger(gnss_queue, writer, control)
        return sensors, logger

    # internal + on 已由 config_loader 拦截
    raise NotImplementedError(
        f"Unsupported config: gnss_source={gnss_source}, ins.enabled={ins_enabled}"
    )


if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 else "data/config.yaml"
    main(cfg)
