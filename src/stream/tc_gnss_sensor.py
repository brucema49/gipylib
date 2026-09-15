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
from src.stream.gnss_band_mapping import resolve_raw_band_priority
from src.utility.rinex_improve import (
    improve_rinex,
    needs_improvement,
    resolve_stream_plan,
)


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
        # 显式 raw_band_priority 仍被接受（高级出口）；未配置时为 None，
        # 由 _run_impl 内的 resolve_stream_plan 按 RINEX 头自动规划。
        # 这里提前解析一次，使非法的显式配置在线程启动前即报错。
        self.resolved_raw_band_priority = (
            resolve_raw_band_priority(self.gnss_cfg) or None)
        # GREAT RAW_MIX-compatible tracking-attribute order.  Keys are raw
        # RINEX band digits, deliberately separate from legacy ``freq_ix``.
        self.raw_signal_priority = self.gnss_cfg.get("raw_signal_priority", {})

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

    def _prepare_rinex(self, path: str) -> str:
        """如需改写（LibGnut 频带归一化/频点选择）则生成临时文件。"""
        if not needs_improvement(
            path,
            band_plan=self.resolved_raw_band_priority,
            gnss_t=self.gnss_cfg.get("gnss_t"),
            raw_signal_priority=self.raw_signal_priority or None,
            max_freqs=max(int(self.gnss_cfg.get("nf", 2) or 2), 2),
        ):
            return path
        suffix = Path(path).suffix
        tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=suffix, delete=False, encoding="utf-8"
        )
        tmp.close()
        improve_rinex(
            path,
            tmp.name,
            band_plan=self.resolved_raw_band_priority,
            gnss_t=self.gnss_cfg.get("gnss_t"),
            raw_signal_priority=self.raw_signal_priority or None,
            max_freqs=max(int(self.gnss_cfg.get("nf", 2) or 2), 2),
        )
        self._temp_files.append(tmp.name)
        return tmp.name

    def _run_impl(self):
        # 1. 解析流级信号方案（显式映射优先，否则按 RINEX 头自动规划），
        #    并补齐缺失的频率映射键
        cfg, self.resolved_raw_band_priority = resolve_stream_plan(
            self.gnss_cfg,
            self.gnss_cfg.get("rover_path"),
            self.gnss_cfg.get("base_path"),
        )

        # 2. 初始化 rtklib 环境 + nav
        env = RtklibEnv(cfg)
        env.setup()
        nav = env.init_nav()

        # 3. 准备 RINEX 文件
        rover_path = self._prepare_rinex(self.gnss_cfg["rover_path"])

        # 3. 加载流动站观测值 + 星历
        from src.core.gnss.rtklib import rinex as rn
        rov = rn.rnx_decode(
            env.get_cfg(),
            raw_band_priority=self.resolved_raw_band_priority,
        )
        rov.decode_obsfile(nav, rover_path, None)
        rov.decode_nav(self.gnss_cfg["eph_path"], nav)

        # 4. 加载基站 (RTK/RTD 模式, 支持多个连续短时段文件)
        base = None
        mode = self.gnss_cfg.get("positioning_mode", "spp")
        if mode in ("rtk", "rtd"):
            from src.stream.rinex_base import load_base
            base = load_base(
                env,
                nav,
                self.gnss_cfg,
                self._prepare_rinex,
                raw_band_priority=self.resolved_raw_band_priority,
            )

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
