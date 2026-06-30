"""GInsStream 主入口。

用法:
    python src/main.py [config_path]

默认 config_path = data/config.yaml
"""
import sys
from pathlib import Path
from queue import Queue

# 让 src/main.py 直接运行时也能 import src.*
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.core.thread_control import ThreadControl
from src.stream.factory import SensorFactory
from src.log.aligned_writer import AlignedWriter
from src.log.aligner import Aligner
from src.log.logger import Logger
from src.utility.config_loader import load_config


def main(config_path: str = "data/config.yaml"):
    # 1. 加载配置（含频率与模式校验）
    config = load_config(config_path)

    # 2. 创建共享对象
    control = ThreadControl()
    imu_queue = Queue(maxsize=2000)
    gnss_queue = Queue(maxsize=100)

    # 3. 工厂创建传感器
    sensors = SensorFactory.create_sensors(
        config, imu_queue, gnss_queue, control
    )

    # 4. 装配 Logger
    writer = AlignedWriter(output_dir=config["output"]["output_dir"])
    aligner = Aligner(imu_dt=1.0 / config["ins"]["data_rate"])
    logger = Logger(imu_queue, gnss_queue, writer, aligner, control)

    # 5. 启动所有线程
    for s in sensors:
        s.start()
    logger.start()

    # 6. 等待 Logger 结束（Logger 收到 GNSS EOF 后退出）
    logger.join()
    control.shutdown()
    for s in sensors:
        s.join(timeout=2)


if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 else "data/config.yaml"
    main(cfg)
