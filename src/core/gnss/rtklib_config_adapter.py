"""rtklib-py 配置适配器：YAML → cfg 模块对象 → sys.modules 注入。

rtklib-py 的 rtkpos.py / postpos.py 在模块顶层 `import __ppk_config as cfg`，
且 rtkinit(cfg) 从 cfg 读取所有参数。本模块把 YAML 的 gnss: 段翻译成 cfg 模块
对象，注入 sys.modules['__ppk_config']，并把 library/rtklib-py/src 加入
sys.path，使 rtklib-py 的各模块可被正常 import。

不修改 library/rtklib-py/ 任何文件，不写 __ppk_config.py 文件。
"""
import sys
import types
from pathlib import Path

_CONSTELLATION_MAP = None  # 懒加载


def _get_constellation_map():
    """懒加载 uGNSS 枚举映射表。"""
    global _CONSTELLATION_MAP
    if _CONSTELLATION_MAP is None:
        from rtkcmn import uGNSS
        _CONSTELLATION_MAP = {
            "GPS": uGNSS.GPS,
            "GLO": uGNSS.GLO,
            "GAL": uGNSS.GAL,
            "QZS": uGNSS.QZS,
            "BDS": uGNSS.BDS,
        }
    return _CONSTELLATION_MAP


def build_cfg_module(gnss_cfg: dict) -> types.ModuleType:
    """把 YAML gnss: 段翻译成 rtklib-py 期望的 cfg 模块对象。"""
    cfg = types.ModuleType("__ppk_config")

    cfg.nf = gnss_cfg["nf"]
    cfg.pmode = gnss_cfg["pmode"]
    cfg.filtertype = gnss_cfg["filtertype"]
    cfg.use_sing_pos = gnss_cfg["use_sing_pos"]
    cfg.elmin = gnss_cfg["elmin"]
    cfg.cnr_min = gnss_cfg["cnr_min"]
    cfg.excsats = gnss_cfg["excsats"]
    cfg.maxinno = gnss_cfg["maxinno"]
    cfg.maxcode = gnss_cfg["maxcode"]
    cfg.maxage = gnss_cfg["maxage"]
    cfg.maxout = gnss_cfg["maxout"]
    cfg.thresdop = gnss_cfg["thresdop"]
    cfg.thresslip = gnss_cfg["thresslip"]
    cfg.interp_base = gnss_cfg["interp_base"]
    cfg.eratio = gnss_cfg["eratio"]
    cfg.snrmax = gnss_cfg["snrmax"]
    cfg.accelh = gnss_cfg["accelh"]
    cfg.accelv = gnss_cfg["accelv"]
    cfg.prnbias = gnss_cfg["prnbias"]
    cfg.sig_p0 = gnss_cfg["sig_p0"]
    cfg.sig_v0 = gnss_cfg["sig_v0"]
    cfg.sig_n0 = gnss_cfg["sig_n0"]
    cfg.armode = gnss_cfg["armode"]
    cfg.thresar = gnss_cfg["thresar"]
    cfg.thresar1 = gnss_cfg["thresar1"]
    cfg.minlock = gnss_cfg.get("minlock", 0)
    cfg.glo_hwbias = gnss_cfg["glo_hwbias"]
    cfg.elmaskar = gnss_cfg["elmaskar"]
    cfg.var_holdamb = gnss_cfg["var_holdamb"]
    cfg.minfix = gnss_cfg["minfix"]
    cfg.minfixsats = gnss_cfg["minfixsats"]
    cfg.minholdsats = gnss_cfg["minholdsats"]
    cfg.mindropsats = gnss_cfg["mindropsats"]
    cfg.sing_p0 = gnss_cfg["sing_p0"]
    cfg.sing_v0 = gnss_cfg["sing_v0"]
    cfg.sing_elmin = gnss_cfg["sing_elmin"]
    cfg.freq = gnss_cfg["freq_table"]
    cfg.dfreq_glo = gnss_cfg["dfreq_glo"]
    cfg.rb = gnss_cfg["rb"]
    cfg.rr_f = gnss_cfg["rr_f"]
    cfg.rr_b = gnss_cfg["rr_b"]

    # err 数组: [_, base, el, bl, snr, rcvstd, satclk]
    cfg.err = [
        0,
        gnss_cfg["err_base"],
        gnss_cfg["err_el"],
        0.0,
        0,
        0,
        gnss_cfg["err_satclk"],
    ]

    cmap = _get_constellation_map()
    cfg.efact = {}
    for sat_str, val in [
        ("GPS", gnss_cfg["efact_gps"]),
        ("GLO", gnss_cfg["efact_glo"]),
        ("GAL", gnss_cfg["efact_gal"]),
    ]:
        if sat_str in cmap:
            cfg.efact[cmap[sat_str]] = val

    cfg.gnss_t = [cmap[s] for s in gnss_cfg["gnss_t"] if s in cmap]

    cfg.freq_ix0 = {cmap[k]: v for k, v in gnss_cfg["freq_ix0"].items() if k in cmap}
    cfg.freq_ix1 = {cmap[k]: v for k, v in gnss_cfg["freq_ix1"].items() if k in cmap}

    return cfg


class RtklibEnv:
    """rtklib-py 运行环境管理器。

    用法:
        env = RtklibEnv(config["gnss"], "library/rtklib-py/src")
        env.setup()
        nav = env.init_nav()
        ...
        env.cleanup()
    """

    def __init__(self, gnss_cfg: dict, library_path: str = "library/rtklib-py/src"):
        self.gnss_cfg = gnss_cfg
        self.library_path = str(Path(library_path).resolve())
        self._cfg_module = None
        self._path_added = False
        self._injected = False

    def setup(self):
        """加入 sys.path + 构建 cfg + 注入 sys.modules。"""
        if self.library_path not in sys.path:
            sys.path.insert(0, self.library_path)
            self._path_added = True

        self._cfg_module = build_cfg_module(self.gnss_cfg)
        sys.modules["__ppk_config"] = self._cfg_module
        self._injected = True

    def get_cfg(self):
        """返回 cfg 模块对象（setup 后可用）。"""
        if self._cfg_module is None:
            self.setup()
        return self._cfg_module

    def init_nav(self):
        """调用 rtkinit(cfg) 返回 nav 对象。"""
        cfg = self.get_cfg()
        from rtkpos import rtkinit
        return rtkinit(cfg)

    def cleanup(self):
        """移除 sys.path 与 sys.modules 注入。"""
        if self._injected and "__ppk_config" in sys.modules:
            del sys.modules["__ppk_config"]
            self._injected = False
        if self._path_added and self.library_path in sys.path:
            sys.path.remove(self.library_path)
            self._path_added = False
