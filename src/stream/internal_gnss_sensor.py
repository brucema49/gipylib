"""内部 GNSS 解算传感器线程。

逐历元调用 rtklib-py 的 pntpos/relpos，把 GnssSolution 推入 gnss_queue。
文件读完后推入 None 作为 EOF sentinel。
"""
from queue import Queue
from threading import Thread

from src.core.thread_control import ThreadControl
from src.core.data_types import SensorData
from src.core.gnss.rtklib_config_adapter import RtklibEnv


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

    def run(self):
        try:
            self._run_impl()
        except Exception:
            pass
        finally:
            self.output_queue.put(None)  # EOF sentinel

    def _run_impl(self):
        # 1. 初始化 rtklib 环境 + nav
        env = RtklibEnv(self.gnss_cfg, library_path="library/rtklib-py/src")
        env.setup()
        nav = env.init_nav()

        # 2. 加载流动站观测值 + 星历（延迟导入 rinex）
        import rinex as rn
        rov = rn.rnx_decode(env.get_cfg())
        rov.decode_obsfile(nav, self.gnss_cfg["rover_path"], None)
        rov.decode_nav(self.gnss_cfg["eph_path"], nav)

        # 3. RTK 模式加载基站
        base = None
        if self.gnss_cfg.get("positioning_mode") == "rtk":
            base = rn.rnx_decode(env.get_cfg())
            base.decode_obsfile(nav, self.gnss_cfg["base_path"], None)
            if nav.rb[0] == 0:
                nav.rb = base.pos

        # 4. 创建处理器并运行（延迟导入，需 RtklibEnv.setup() 先完成）
        mode = self.gnss_cfg["positioning_mode"]
        if mode == "spp":
            from src.core.gnss.spp_processor import SppProcessor
            processor = SppProcessor(nav)
            self._run_spp_loop(processor, rov)
        elif mode == "rtk":
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
