"""紧组合状态向量索引 (扩展 StateIndex 加 GNSS 参数块)。

状态布局:
  δx = [δr^e, δv^e, δψ^e, δb_g, δb_a, 可选块, clk_bias(3)?, ambiguity(N)?]
  - SPP: + clk_bias(3)  [dtr, dtr_glo, dtr_gal]
  - RTK: + ambiguity(N)  N 运行时动态
  - RTD: 无 GNSS 参数块 (双差消除钟差, 仅伪距无模糊度)

继承 StateIndex (dataclass), 复用 INS 基础 15 维 + 可选块索引,
扩展 GNSS 参数块 (clk_bias / ambiguity)。
"""
from src.core.ins.state_index import StateIndex


class TcStateIndex(StateIndex):
    """紧组合状态索引。

    继承 StateIndex 的 INS 基础 15 维 + 可选块,
    扩展 GNSS 参数块 (clk_bias / ambiguity)。
    """

    def __init__(self):
        # 调用 dataclass 默认初始化 (所有字段取默认值)
        super().__init__()
        self.mode = "spp"
        self._gnss_base = self.dim          # INS + 可选块结束位置
        self.clk_bias = -1                  # SPP 钟差块起始 (-1=未启用)
        self.amb_start = -1                 # RTK 模糊度块起始 (-1=未启用)
        self.n_amb = 0                      # 模糊度数量
        self.nf = 2                         # 频率数 (影响 amb_idx 索引公式, 默认双频)

    @classmethod
    def from_config(cls, config: dict, mode: str) -> "TcStateIndex":
        """根据配置 + 模式构建索引。

        Args:
            config: 配置字典 (读 ins.estimate_leverarm 等)
            mode: "spp" / "rtd" / "rtk"
        """
        # 先用基类 from_config 构建 INS 部分
        base = StateIndex.from_config(config)
        si = cls()
        # 复制基类字段
        si.pos = base.pos
        si.vel = base.vel
        si.att = base.att
        si.gyro_bias = base.gyro_bias
        si.accel_bias = base.accel_bias
        si.lever_arm = base.lever_arm
        si.imu_angle = base.imu_angle
        si.imu_leverarm = base.imu_leverarm
        si.time_sync = base.time_sync
        si.dim = base.dim
        # 扩展 GNSS 参数块
        si.mode = mode
        si.nf = int(config.get("gnss", {}).get("nf", 2)) if config else 2
        si._gnss_base = si.dim
        si._init_gnss_blocks()
        return si

    def _init_gnss_blocks(self):
        """根据模式初始化 GNSS 参数块索引。"""
        cur = self._gnss_base
        self.clk_bias = -1
        self.amb_start = -1
        self.n_amb = 0
        if self.mode == "spp":
            self.clk_bias = cur
            cur += 3
        elif self.mode == "rtk":
            self.amb_start = cur
            # ambiguity 数量运行时设置
        # rtd: 无 GNSS 块
        self.dim = cur + (self.n_amb if self.mode == "rtk" else 0)

    def set_ambiguity_count(self, n: int):
        """RTK 模式运行时设置模糊度数量 (影响 dim)。"""
        if self.mode != "rtk":
            return
        self.n_amb = n
        self.dim = self.amb_start + n

    def has_ambiguity(self) -> bool:
        return self.mode == "rtk" and self.n_amb > 0

    def amb_idx(self, sat: int, freq: int) -> int:
        """模糊度索引 (sat-major 布局, 与 GINav 兼容)。

        idx = amb_start + (sat-1)*nf + freq
        对于 nf=1: idx = amb_start + sat - 1, 范围 [amb_start, amb_start+MAXSAT-1]
        对于 nf=2: idx = amb_start + (sat-1)*2 + freq, 范围 [amb_start, amb_start+2*MAXSAT-1]
        总槽位 = MAXSAT * nf (由 set_ambiguity_count 分配)
        """
        if self.amb_start < 0:
            return -1
        return self.amb_start + (sat - 1) * self.nf + freq

    def reset_gnss_blocks(self):
        """降级重整时重置 GNSS 参数块 (保留 INS+可选块)。"""
        self.n_amb = 0
        self._init_gnss_blocks()
