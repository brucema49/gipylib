"""紧组合 GNSS 原始观测传感器线程。

逐历元从 RINEX 文件读取原始观测 (obsr, obsb, nav)，推入 gnss_queue。
不做任何解算，把原始观测交给 TcStream/TcIntegration 处理。
文件读完后推入 None 作为 EOF sentinel。
"""
import tempfile
from pathlib import Path
from queue import Queue
from threading import Thread

from src.core.thread_control import ThreadControl
from src.core.data_types import SensorData
from src.core.gnss.rtklib_config_adapter import RtklibEnv
from src.stream.gnss_band_mapping import (
    raw_band_priority_to_simplifier_priority,
    resolve_raw_band_priority,
)
from src.utility.rinex_simplifier import needs_simplification, simplify_rinex


class TcGnssSensor(Thread):
    """紧组合 GNSS 原始观测传感器线程。

    加载 RINEX obs + nav，逐历元推送 (obsr, obsb, nav) 到 gnss_queue。
    与 InternalGnssSensor 的区别：不调用 pntpos/relpos，直接输出原始观测。
    """

    def __init__(self, config: dict, output_queue: Queue, control: ThreadControl):
        Thread.__init__(self, name="TcGnssSensor", daemon=True)
        self.config = config
        self.gnss_cfg = config["gnss"]
        self.output_queue = output_queue
        self.control = control
        self._temp_files = []
        # Keep raw RINEX bands separate from solver-facing freq_ix values.
        # The decoder/simplifier wiring will consume this mapping in the next
        # reader slice; resolving it here makes invalid configuration fail
        # before the sensor thread starts.
        self.resolved_raw_band_priority = resolve_raw_band_priority(self.gnss_cfg)

    def run(self):
        try:
            self._run_impl()
        finally:
            for p in self._temp_files:
                try:
                    Path(p).unlink(missing_ok=True)
                except Exception:
                    pass
            self.output_queue.put(None)  # EOF sentinel

    def _build_freq_priority(self) -> dict:
        """Return the legacy simplifier-rank adapter for resolved raw bands."""
        return raw_band_priority_to_simplifier_priority(
            self.resolved_raw_band_priority
        )

    def _prepare_rinex(self, path: str) -> str:
        """如需简化则生成临时简化文件，返回可用路径。"""
        if not needs_simplification(path):
            return path
        suffix = Path(path).suffix
        tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=suffix, delete=False, encoding="utf-8"
        )
        tmp.close()
        freq_priority = self._build_freq_priority()
        simplify_rinex(path, tmp.name, freq_priority=freq_priority)
        self._temp_files.append(tmp.name)
        return tmp.name

    def _run_impl(self):
        # 1. 初始化 rtklib 环境 + nav
        env = RtklibEnv(self.gnss_cfg)
        env.setup()
        nav = env.init_nav()

        # 2. 准备 RINEX 文件
        rover_path = self._prepare_rinex(self.gnss_cfg["rover_path"])

        # 3. 加载流动站观测值 + 星历
        from src.core.gnss.rtklib import rinex as rn
        rov = rn.rnx_decode(env.get_cfg())
        rov.decode_obsfile(nav, rover_path, None)
        rov.decode_nav(self.gnss_cfg["eph_path"], nav)

        # 4. 加载基站 (RTK/RTD 模式, 支持多个连续短时段文件)
        base = None
        mode = self.gnss_cfg.get("positioning_mode", "spp")
        if mode in ("rtk", "rtd"):
            from src.stream.rinex_base import load_base
            base = load_base(env, nav, self.gnss_cfg, self._prepare_rinex)

        # 5. 时间匹配 rover/base，逐历元推送原始观测
        if base is not None:
            base_times = [float(ob.t.time + ob.t.sec) for ob in base.obslist]
            for obsr in rov.obslist:
                if not self.control.is_running():
                    break
                t = float(obsr.t.time + obsr.t.sec)
                # 找时间最接近的 base 历元
                best_j = -1
                best_dt = 1.0
                for j, bt in enumerate(base_times):
                    d = abs(bt - t)
                    if d < best_dt:
                        best_dt = d
                        best_j = j
                obsb = base.obslist[best_j] if best_j >= 0 else None
                self.output_queue.put(
                    SensorData(tag="gnss_raw", gnss_raw=(obsr, obsb, nav)))
        else:
            for obsr in rov.obslist:
                if not self.control.is_running():
                    break
                self.output_queue.put(
                    SensorData(tag="gnss_raw", gnss_raw=(obsr, None, nav)))
