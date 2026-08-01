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
from src.utility.rinex_simplifier import needs_simplification, simplify_rinex


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
        self._temp_files = []  # 临时简化文件，待清理

    def run(self):
        try:
            self._run_impl()
        finally:
            # 清理临时简化文件
            for p in self._temp_files:
                try:
                    Path(p).unlink(missing_ok=True)
                except Exception:
                    pass
            self.output_queue.put(None)  # EOF sentinel

    def _build_freq_priority(self) -> dict:
        """从配置的 freq_ix0/freq_ix1 构建各系统的频点优先级。

        配置中 freq_ix0/freq_ix1 的索引与 _FREQ_ORDER 中的频点序号一致:
          0=L1/E1/B1, 1=L2, 2=L5/E5a, 3=E5b/B2I, 4=E6/B3

        Returns:
            {'G': [0, 1], 'C': [3, 3], 'E': [0, 2], ...}
            键为 RINEX 系统字符，值为优先保留的频点列表。
        """
        sys_map = {"GPS": "G", "BDS": "C", "GAL": "E", "GLO": "R", "QZS": "J", "SBS": "S"}
        freq_ix0 = self.gnss_cfg.get("freq_ix0", {})
        freq_ix1 = self.gnss_cfg.get("freq_ix1", {})
        priority = {}
        for sys_name, sys_char in sys_map.items():
            freqs = []
            if sys_name in freq_ix0:
                freqs.append(int(freq_ix0[sys_name]))
            if sys_name in freq_ix1:
                freqs.append(int(freq_ix1[sys_name]))
            if freqs:
                priority[sys_char] = freqs
        return priority

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

        # 2. 准备 RINEX 文件（必要时简化）
        rover_path = self._prepare_rinex(self.gnss_cfg["rover_path"])

        # 3. 加载流动站观测值 + 星历
        from src.core.gnss.rtklib import rinex as rn
        rov = rn.rnx_decode(env.get_cfg())
        rov.decode_obsfile(nav, rover_path, None)
        rov.decode_nav(self.gnss_cfg["eph_path"], nav)

        # 4. RTK 模式加载基站
        base = None
        if self.gnss_cfg.get("positioning_mode") == "rtk":
            base_path = self._prepare_rinex(self.gnss_cfg["base_path"])
            base = rn.rnx_decode(env.get_cfg())
            base.decode_obsfile(nav, base_path, None)
            if nav.rb[0] == 0:
                nav.rb = base.pos

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
