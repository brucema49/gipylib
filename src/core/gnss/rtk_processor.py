"""RTK 相对定位处理器，薄封装 rtklib-py 的 relpos。"""
from typing import Optional

from src.core.data_types import GnssSolution
from src.core.gnss.gnss_processor import GnssProcessor
from src.core.gnss.solution_converter import sol_to_gnss_solution, SOLQ_NONE


class RtkProcessor(GnssProcessor):
    """RTK 相对定位处理器。

    参考 rtklib-py 的 rtkpos() 循环（rtkpos.py:1073），拆分为逐历元接口。
    跨历元状态:
    - self.sol: 上历元解算结果（用于判断是否需要重新 SPP 取初值）
    - nav: 由调用方管理，持有 x/P/azel/lock 等状态

    rtklib-py 函数在 __init__ 时延迟导入（需 RtklibEnv.setup() 先完成）。
    """

    def __init__(self, nav):
        self.nav = nav
        # 延迟导入
        from pntpos import pntpos
        from rtkpos import relpos
        from rtkcmn import Sol
        self._pntpos = pntpos
        self._relpos = relpos
        self._Sol = Sol
        self.sol = self._Sol()  # 初始 sol，rr[0]==0 触发首历元 pntpos

    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        """调用 relpos 解算单历元 RTK。"""
        # 首历元或 sol.rr[0]==0 时先 pntpos 取初值
        if self.nav.use_sing_pos or self.sol.stat == SOLQ_NONE or self.sol.rr[0] == 0.0:
            self.sol = self._pntpos(obsr, self.nav)
            # 用 SPP 解初始化 nav.x，供 relpos 的 zdres 计算流动站位置。
            # rtklib-py 的 rtkpos() 用 cfg.rr_f 初始化 nav.x；当 rr_f=0 时
            # nav.x 保持 [0,0,0]，导致 relpos 残差异常全部被剔除。
            self.nav.x[0:6] = self.sol.rr[0:6]
            self.nav.x[6:9] = 1e-6  # match RTKLIB
        else:
            self.sol = self._Sol()

        # 确保时间戳正确
        if self.sol.t.time == 0:
            self.sol.t = obsr.t

        # 相对定位（修改 self.sol 与 self.nav 状态）
        self._relpos(self.nav, obsr, obsb, self.sol)

        return sol_to_gnss_solution(self.sol)

    def reset(self) -> None:
        """重置处理器状态（不重置 nav）。"""
        self.sol = self._Sol()
