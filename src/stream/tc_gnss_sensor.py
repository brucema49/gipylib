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

        # 2. 准备 RINEX 文件
        rover_path = self._prepare_rinex(self.gnss_cfg["rover_path"])

        # 3. 加载流动站观测值 + 星历
        from src.core.gnss.rtklib import rinex as rn
        rov = rn.rnx_decode(env.get_cfg())
        rov.decode_obsfile(nav, rover_path, None)
        rov.decode_nav(self.gnss_cfg["eph_path"], nav)

        # 4. 加载基站 (RTK/RTD 模式)
        base = None
        mode = self.gnss_cfg.get("positioning_mode", "spp")
        if mode in ("rtk", "rtd"):
            base_path = self._prepare_rinex(self.gnss_cfg["base_path"])
            base = rn.rnx_decode(env.get_cfg())
            base.decode_obsfile(nav, base_path, None)
            if nav.rb[0] == 0:
                nav.rb = base.pos

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
