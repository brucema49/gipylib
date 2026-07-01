"""SPP 单点定位处理器，薄封装 rtklib-py 的 pntpos。"""
from typing import Optional

from src.core.data_types import GnssSolution
from src.core.gnss.gnss_processor import GnssProcessor
from src.core.gnss.solution_converter import sol_to_gnss_solution


class SppProcessor(GnssProcessor):
    """SPP 单点定位处理器。

    每历元独立调用 pntpos(obsr, nav)，无跨历元模糊度状态。
    rtklib-py 的 pntpos 在 __init__ 时延迟导入（需 RtklibEnv.setup() 先完成）。
    """

    def __init__(self, nav):
        self.nav = nav
        # 延迟导入：需先由 RtklibEnv.setup() 注入 __ppk_config 到 sys.modules
        from pntpos import pntpos
        self._pntpos = pntpos

    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        """调用 pntpos 解算单历元 SPP。"""
        sol = self._pntpos(obsr, self.nav)
        return sol_to_gnss_solution(sol)
