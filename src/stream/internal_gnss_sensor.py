"""内部 GNSS 解算传感器线程。

逐历元调用 rtklib-py 的 pntpos/relpos，把 GnssSolution 推入 gnss_queue。
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


class InternalGnssSensor(Thread):
    """内部 GNSS 解算传感器线程。

    根据 positioning_mode 创建 SppProcessor 或 RtkProcessor，
    逐历元解算并推入 gnss_queue。
    """

    def __init__(self, config: dict, output_queue: Queue, control: ThreadControl):
        Thread.__init__(self, name="InternalGnssSensor", daemon=True)
        self.config = config
        self.gnss_cfg = config["gnss"]
        self.output_queue = output_queue
        self.control = control
        self._temp_files = []  # 临时改写文件，待清理
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
            # 清理临时改写文件
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

        # 3. 准备 RINEX 文件（必要时改写）
        rover_path = self._prepare_rinex(self.gnss_cfg["rover_path"])

        # 3. 加载流动站观测值 + 星历
        from src.core.gnss.rtklib import rinex as rn
        rov = rn.rnx_decode(
            env.get_cfg(),
            raw_band_priority=self.resolved_raw_band_priority,
        )
        rov.decode_obsfile(nav, rover_path, None)
        rov.decode_nav(self.gnss_cfg["eph_path"], nav)

        # 4. RTK 模式加载基站 (支持多个连续短时段文件)
        base = None
        if self.gnss_cfg.get("positioning_mode") in ("rtk", "rtd"):
            from src.stream.rinex_base import load_base
            base = load_base(
                env,
                nav,
                self.gnss_cfg,
                self._prepare_rinex,
                raw_band_priority=self.resolved_raw_band_priority,
            )

        # 5. 创建处理器并运行（延迟导入，需 RtklibEnv.setup() 先完成）
        mode = self.gnss_cfg["positioning_mode"]
        filtertype = self.gnss_cfg.get("filtertype", "forward")
        if mode == "spp":
            from src.core.gnss.spp_processor import SppProcessor
            processor = SppProcessor(nav)
            self._run_spp_loop(processor, rov)
        elif mode == "rtk":
            if filtertype in ("combined", "backward", "combined_noreset"):
                self._run_rtk_batch(nav, rov, base, filtertype)
            else:
                from src.core.gnss.rtk_processor import RtkProcessor
                processor = RtkProcessor(nav)
                self._run_rtk_loop(processor, rov, base, nav, rn)
        elif mode == "rtd":
            from src.core.gnss.rtd_processor import RtdProcessor
            processor = RtdProcessor(nav)
            self._run_rtk_loop(processor, rov, base, nav, rn)
        else:
            raise ValueError(f"Unsupported positioning_mode: {mode}")

    def _run_spp_loop(self, processor, rov):
        """SPP 模式: 直接遍历 rover.obslist。"""
        for obsr in rov.obslist:
            if not self.control.is_running():
                break
            sol = processor.process_epoch(obsr)
            if sol is not None:
                self.output_queue.put(SensorData(tag="gnss_solution", gnss_solution=sol))

    def _run_rtk_loop(self, processor, rov, base, nav, rn):
        """RTK 模式: 用 first_obs/next_obs 做时间同步。"""
        dir = 1  # forward
        obsr, obsb = rn.first_obs(nav, rov, base, dir)
        while True:
            if not self.control.is_running():
                break
            if obsr == []:
                break
            sol = processor.process_epoch(obsr, obsb)
            if sol is not None:
                self.output_queue.put(SensorData(tag="gnss_solution", gnss_solution=sol))
            obsr, obsb = rn.next_obs(nav, rov, base, dir)

    def _run_rtk_batch(self, nav, rov, base, filtertype):
        """RTK 批处理模式: combined/backward 滤波。

        使用 rtklib-py 的 procpos() 完成正向+反向+平滑组合，
        然后将组合解逐历元推入 gnss_queue。
        """
        import os
        from src.core.gnss.rtklib.postpos import procpos
        from src.core.gnss.solution_converter import sol_to_gnss_solution

        nav.filtertype = filtertype
        fp_stat = open(os.devnull, "w")
        try:
            sol_list = procpos(nav, rov, base, fp_stat)
        finally:
            fp_stat.close()

        for sol in sol_list:
            if not self.control.is_running():
                break
            gsol = sol_to_gnss_solution(sol)
            if gsol is not None:
                self.output_queue.put(SensorData(tag="gnss_solution", gnss_solution=gsol))
