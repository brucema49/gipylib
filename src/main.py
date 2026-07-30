"""GInsStream 主入口。

用法:
    python src/main.py [config_path]

默认 config_path = data/config.yaml
"""
import sys
import time
from pathlib import Path
from queue import Queue

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.core.thread_control import ThreadControl
from src.stream.factory import SensorFactory
from src.log.aligned_writer import AlignedWriter
from src.log.aligner import Aligner
from src.log.logger import Logger, TcLogger
from src.log.rslt_writer import RSLTWriter
from src.log.solution_writer import SolutionWriter
from src.log.solution_logger import SolutionLogger
from src.utility.config_loader import load_config


def main(config_path: str = "data/config.yaml"):
    config = load_config(config_path)

    control = ThreadControl()
    imu_queue = Queue(maxsize=200)
    gnss_queue = Queue(maxsize=3)

    sensors, logger = _assemble_pipeline(config, control, imu_queue, gnss_queue)

    t0 = time.monotonic()
    for s in sensors:
        s.start()
    logger.start()

    logger.join()
    elapsed = time.monotonic() - t0
    control.shutdown()
    for s in sensors:
        s.join(timeout=2)

    print(f"运行时长: {elapsed:.1f}s ({int(elapsed // 60)}m {elapsed % 60:.1f}s)")


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
        filename = config["output"].get("gnss_filename", "RTK.pos")
        writer = SolutionWriter(output_dir=config["output"]["output_dir"],
                                filename=filename)
        logger = SolutionLogger(gnss_queue, writer, control)
        return sensors, logger

    if gnss_source == "internal" and ins_enabled == "on":
        # 路径 C: 内部 GNSS 实时解算 + IMU 对齐输出 + 松组合 EKF
        # 输出三个文件：纯 GNSS .pos + 对齐 CSV + 松组合 .rslt (100Hz, ECEF+速度+姿态)
        sensors = SensorFactory.create_sensors(config, imu_queue, gnss_queue, control)
        filename = config["output"].get("aligned_filename", "aligned.csv")
        writer = AlignedWriter(
            output_dir=config["output"]["output_dir"],
            filename=filename,
        )
        # 纯 GNSS 定位结果 .pos 文件（副输出）
        gnss_filename = config["output"].get("gnss_filename", "RTK.pos")
        gnss_writer = SolutionWriter(
            output_dir=config["output"]["output_dir"],
            filename=gnss_filename,
        )
        # 松组合定位结果 .rslt 文件（100Hz, ignav outins 风格 LLH位置+ECEF速度+FRD姿态）
        lc_filename = config["output"].get("rslt_filename", "RTKTC.rslt")
        lc_writer = RSLTWriter(
            output_dir=config["output"]["output_dir"],
            filename=lc_filename,
        )
        from src.core.ins.lc_stream import LcStream
        lc_stream = LcStream(config, lc_writer)
        aligner = Aligner(imu_dt=1.0 / config["ins"]["data_rate"])
        logger = Logger(imu_queue, gnss_queue, writer, aligner, control,
                        gnss_writer=gnss_writer, lc_stream=lc_stream)
        return sensors, logger

    if gnss_source == "internal" and ins_enabled == "tc":
        # 路径 D: 紧组合 GNSS/INS (TcGnssSensor 原始观测 + TcStream + RSLTWriter)
        # 输出一个文件：紧组合 .rslt (100Hz, ignav outins 风格 LLH位置+ECEF速度+FRD姿态)
        # 不输出纯 GNSS .pos (TC 不做独立 GNSS 解算)
        sensors = SensorFactory.create_sensors(config, imu_queue, gnss_queue, control)
        tc_filename = config["output"].get("rslt_filename", "RTKTC.rslt")
        tc_writer = RSLTWriter(
            output_dir=config["output"]["output_dir"],
            filename=tc_filename,
        )
        from src.core.tc.tc_stream import TcStream
        tc_stream = TcStream(config, tc_writer)
        # harvest_window 与 LC 一致 (1.0s), 保证 IMU 覆盖 GNSS 历元窗口
        harvest_window = 1.0
        logger = TcLogger(imu_queue, gnss_queue, tc_stream, control,
                          harvest_window=harvest_window)
        return sensors, logger

    raise NotImplementedError(
        f"Unsupported config: gnss_source={gnss_source}, ins.enabled={ins_enabled}"
    )


if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 else "data/spp-ins-tc.yaml"
    main(cfg)
