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
from src.log.stat_writer import StatWriter
from src.log.solution_writer import SolutionWriter
from src.log.solution_logger import SolutionLogger
from src.log.trace_file_writer import TraceFileWriter
from src.utility.config_loader import load_config


def main(config_path: str = "data/config.yaml"):
    config = load_config(config_path)

    # trace 文件 (level > 0 时启用)
    output_cfg = config.get("output", {})
    trace_level = int(output_cfg.get("trace_level", 0))
    trace_writer = None
    if trace_level > 0:
        # trace 文件名与主输出文件同名, 扩展名 .trace
        ins_enabled = config["ins"]["enabled"]
        if ins_enabled in ("lc", "tc"):
            ref_filename = output_cfg.get("rslt_filename", "RTKLC.rslt")
        else:
            ref_filename = output_cfg.get("gnss_filename", "RTK.pos")
        trace_writer = TraceFileWriter(
            output_dir=output_cfg["output_dir"],
            ref_filename=ref_filename,
            trace_level=trace_level,
        )
        trace_writer.open()
        if trace_writer.enabled:
            import hashlib as _hashlib
            cfg_sum = ""
            try:
                with open(config_path, "rb") as fh:
                    cfg_sum = _hashlib.md5(fh.read()).hexdigest()[:12]
            except OSError:
                pass
            trace_writer.write_event(
                1, "RUN_START",
                f"config={config_path} md5={cfg_sum} "
                f"mode={config['ins']['enabled']} "
                f"stat_level={int(output_cfg.get('stat_level', 0))}",
                mode=config["ins"]["enabled"])

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

    if trace_writer is not None:
        if trace_writer.enabled:
            trace_writer.write_event(1, "RUN_END",
                                     f"elapsed={elapsed:.1f}s",
                                     mode=config["ins"]["enabled"])
        trace_writer.close()

    print(f"运行时长: {elapsed:.1f}s ({int(elapsed // 60)}m {elapsed % 60:.1f}s)")


def _get_output_formats(config: dict):
    """从 config 提取输出格式参数。"""
    output_cfg = config.get("output", {})
    return (
        output_cfg.get("position_format", "llh"),
        output_cfg.get("time_format", "gpst"),
    )


class _GnssStatTee:
    """off 模式复合输出器: SolutionWriter(.pos) + StatWriter(.stat) 同步写入。"""

    def __init__(self, solution_writer, stat_writer):
        self._sol = solution_writer
        self._stat = stat_writer

    def open(self):
        self._sol.open()
        if self._stat is not None:
            self._stat.open()

    def write(self, sol):
        self._sol.write(sol)
        if self._stat is not None:
            self._stat.write_gnss(sol)

    def close(self):
        if self._stat is not None:
            self._stat.close()
        self._sol.close()


def _parse_output_switches(config):
    """解析 output: 段的 stat/trace 开关并校验。

    Returns:
        dict(stat_level, stat_rate, stat_filename, trace_enabled, trace_level)
    """
    out = config.get("output", {})
    stat_level = int(out.get("stat_level", 0))
    if stat_level not in (0, 1, 2, 3):
        raise ValueError(f"output.stat_level 必须是 0/1/2/3, 收到 {stat_level}")
    stat_rate = str(out.get("stat_rate", "update"))
    if stat_rate not in ("update", "second", "imu"):
        raise ValueError(f"output.stat_rate 必须是 update/second/imu, 收到 {stat_rate}")
    stat_filename = str(out.get("stat_filename", ""))
    trace_enabled = bool(out.get("trace_enabled", True))
    trace_level = int(out.get("trace_level", 0))
    if trace_level not in (0, 1, 2, 3):
        raise ValueError(f"output.trace_level 必须是 0/1/2/3, 收到 {trace_level}")
    return {"stat_level": stat_level, "stat_rate": stat_rate,
            "stat_filename": stat_filename, "trace_enabled": trace_enabled,
            "trace_level": trace_level}


def _assemble_pipeline(config, control, imu_queue, gnss_queue):
    """根据配置选择 sensors 与 logger。

    Returns:
        (sensors, logger) 元组
    """
    gnss_source = config["gnss"]["gnss_source"]
    ins_enabled = config["ins"]["enabled"]
    pos_fmt, time_fmt = _get_output_formats(config)

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
        sw = _parse_output_switches(config)
        writer = SolutionWriter(
            output_dir=config["output"]["output_dir"],
            filename=filename,
            position_format=pos_fmt,
            time_format=time_fmt,
        )
        stat_writer = None
        if sw["stat_level"] > 0:
            stat_writer = StatWriter(
                output_dir=config["output"]["output_dir"],
                stat_level=sw["stat_level"],
                stat_rate=sw["stat_rate"],
                filename=sw["stat_filename"],
                ref_filename=filename,
                mode="off",
            )
            writer = _GnssStatTee(writer, stat_writer)
        logger = SolutionLogger(gnss_queue, writer, control)
        return sensors, logger

    if gnss_source in ("internal", "awesome_external") and ins_enabled == "lc":
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
            position_format=pos_fmt,
            time_format=time_fmt,
        )
        # 松组合定位结果 .rslt 文件（100Hz, ignav outins 风格 位置+ECEF速度+FRD姿态）
        lc_filename = config["output"].get("rslt_filename", "RTKLC.rslt")
        lc_writer = RSLTWriter(
            output_dir=config["output"]["output_dir"],
            filename=lc_filename,
            position_format=pos_fmt,
            time_format=time_fmt,
            time_precision=int(config["output"].get("time_precision", 3)),
        )
        sw = _parse_output_switches(config)
        stat_writer = None
        if sw["stat_level"] > 0:
            stat_writer = StatWriter(
                output_dir=config["output"]["output_dir"],
                stat_level=sw["stat_level"],
                stat_rate=sw["stat_rate"],
                filename=sw["stat_filename"],
                ref_filename=lc_filename,
                mode="lc",
            )
        from src.core.ins.lc_stream import LcStream
        lc_stream = LcStream(config, lc_writer, stat_writer=stat_writer)
        aligner = Aligner(imu_dt=1.0 / config["ins"]["data_rate"])
        logger = Logger(imu_queue, gnss_queue, writer, aligner, control,
                        gnss_writer=gnss_writer, lc_stream=lc_stream)
        return sensors, logger

    if gnss_source == "internal" and ins_enabled == "tc":
        # 路径 D: 紧组合 GNSS/INS (TcGnssSensor 原始观测 + TcStream + RSLTWriter)
        # 输出一个文件：紧组合 .rslt (100Hz, ignav outins 风格 位置+ECEF速度+FRD姿态)
        # 不输出纯 GNSS .pos (TC 不做独立 GNSS 解算)
        sensors = SensorFactory.create_sensors(config, imu_queue, gnss_queue, control)
        # campus01 实验契约: output.nav_csv_filename 存在时改写 100 Hz 状态 CSV
        # (固定列 week,sow,x,y,z,vx,vy,vz,roll,pitch,yaw), 否则保持 .rslt。
        nav_csv_filename = str(config["output"].get("nav_csv_filename", ""))
        tc_filename = config["output"].get("rslt_filename", "RTKTC.rslt")
        if nav_csv_filename:
            from src.log.nav_csv_writer import NavCsvWriter
            tc_writer = NavCsvWriter(
                output_dir=config["output"]["output_dir"],
                filename=nav_csv_filename,
            )
        else:
            tc_writer = RSLTWriter(
                output_dir=config["output"]["output_dir"],
                filename=tc_filename,
                position_format=pos_fmt,
                time_format=time_fmt,
                time_precision=int(config["output"].get("time_precision", 3)),
            )
        sw = _parse_output_switches(config)
        stat_writer = None
        if sw["stat_level"] > 0:
            stat_writer = StatWriter(
                output_dir=config["output"]["output_dir"],
                stat_level=sw["stat_level"],
                stat_rate=sw["stat_rate"],
                filename=sw["stat_filename"],
                ref_filename=tc_filename,
                mode="tc",
            )
        from src.core.tc.tc_stream import TcStream
        tc_stream = TcStream(config, tc_writer, stat_writer=stat_writer)
        # harvest_window 与 LC 一致 (1.0s), 保证 IMU 覆盖 GNSS 历元窗口
        harvest_window = 1.0
        logger = TcLogger(imu_queue, gnss_queue, tc_stream, control,
                          harvest_window=harvest_window)
        return sensors, logger

    raise NotImplementedError(
        f"Unsupported config: gnss_source={gnss_source}, ins.enabled={ins_enabled}"
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="GInsStream 主入口")
    parser.add_argument(
        "config", nargs="?", default="data/config.yaml",
        help="YAML 配置文件路径 (位置参数, 兼容旧用法)")
    parser.add_argument(
        "--config", dest="config_option", default=None,
        help="YAML 配置文件路径 (显式选项, 优先于位置参数)")
    args = parser.parse_args()
    main(args.config_option or args.config)
