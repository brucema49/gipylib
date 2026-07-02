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
    - self._prev_t: 上历元解算时间（用于计算 nav.tt，TimeDiff）
    - nav: 由调用方管理，持有 x/P/azel/lock 等状态

    rtklib-py 函数在 __init__ 时延迟导入（需 RtklibEnv.setup() 先完成）。
    """

    def __init__(self, nav):
        self.nav = nav
        # 延迟导入
        from pntpos import pntpos
        from rtkpos import relpos, timediff
        from rtkcmn import Sol, gtime_t
        self._pntpos = pntpos
        self._relpos = relpos
        self._timediff = timediff
        self._Sol = Sol
        self._gtime_t = gtime_t
        self.sol = self._Sol()  # 初始 sol，rr[0]==0 触发首历元 pntpos
        self._prev_t = self._gtime_t()  # 初始为 0，首历元不计算 nav.tt

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

        # 计算 nav.tt（当前历元与上历元的时间差），对应 rtkpos.py:1106-1107。
        # relpos 内的 udpos/udbias 依赖 nav.tt 做状态传播与过程噪声注入；
        # 若 nav.tt 恒为 0，位置不预测、相位偏差过程噪声不加，会导致级联剔除。
        if self._prev_t.time != 0:
            self.nav.tt = self._timediff(self.sol.t, self._prev_t)

        # 相对定位（修改 self.sol 与 self.nav 状态）
        self._relpos(self.nav, obsr, obsb, self.sol)

        # 记录本历元时间，供下一历元计算 nav.tt
        self._prev_t.time = self.sol.t.time
        self._prev_t.sec = self.sol.t.sec

        # rtklib-py 的 relpos 不写 sol.ns，用 nav.ns (L1 频点 vsat>0 卫星数)
        # 当 nav.ns==0 (DGPS 降级且 vsat 全 0) 时，回退到本历元观测卫星数
        if self.sol.stat != SOLQ_NONE and self.sol.ns == 0:
            self.sol.ns = self.nav.ns if self.nav.ns > 0 else len(obsr.sat)

        return sol_to_gnss_solution(self.sol)

    def reset(self) -> None:
        """重置处理器状态（不重置 nav）。"""
        self.sol = self._Sol()
        self._prev_t = self._gtime_t()
