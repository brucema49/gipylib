"""rtklib-py 配置适配器：YAML → params dict → config.set_params()。

rtklib-py 的 rtkpos.py / postpos.py 通过 `from . import config as cfg` 引用配置。
本模块把 YAML 的 gnss: 段翻译成 params dict，调用 config.set_params() 注入。
"""
import warnings

_CONSTELLATION_MAP = None  # 懒加载


def _get_constellation_map():
    """懒加载 uGNSS 枚举映射表。"""
    global _CONSTELLATION_MAP
    if _CONSTELLATION_MAP is None:
        from .rtklib.rtkcmn import uGNSS
        _CONSTELLATION_MAP = {
            "GPS": uGNSS.GPS,
            "GLO": uGNSS.GLO,
            "GAL": uGNSS.GAL,
            "QZS": uGNSS.QZS,
            "BDS": uGNSS.BDS,
        }
    return _CONSTELLATION_MAP


def _f(v):
    """强制转 float（YAML 可能将科学计数法解析为 str）。"""
    return float(v)


def _i(v):
    """强制转 int。"""
    return int(v)


def _fv(lst):
    """列表 → float 列表。"""
    return [float(x) for x in lst]


def build_params(gnss_cfg: dict) -> dict:
    """把 YAML gnss: 段翻译成 rtklib-py 配置参数字典。

    返回的 dict 的 key 与原 __ppk_config.py 的模块级变量名一致，
    由 config.set_params() 注入到 _CfgProxy。
    """
    params = {}

    params["nf"] = _i(gnss_cfg["nf"])
    params["pmode"] = gnss_cfg["pmode"]
    params["filtertype"] = gnss_cfg["filtertype"]
    params["use_sing_pos"] = gnss_cfg["use_sing_pos"]
    params["elmin"] = _f(gnss_cfg["elmin"])
    params["cnr_min"] = _fv(gnss_cfg["cnr_min"])
    # excsats 在下方与 remove_sat 合并并转换为卫星号（见 id2sat 处）
    params["maxinno"] = _f(gnss_cfg["maxinno"])
    params["maxcode"] = _f(gnss_cfg["maxcode"])
    params["maxage"] = _f(gnss_cfg["maxage"])
    params["maxout"] = _i(gnss_cfg["maxout"])
    params["thresdop"] = _f(gnss_cfg["thresdop"])
    params["thresslip"] = _f(gnss_cfg["thresslip"])
    params["interp_base"] = gnss_cfg["interp_base"]
    params["eratio"] = _fv(gnss_cfg["eratio"])
    params["snrmax"] = _f(gnss_cfg["snrmax"])
    params["accelh"] = _f(gnss_cfg["accelh"])
    params["accelv"] = _f(gnss_cfg["accelv"])
    params["pos_psd"] = _f(gnss_cfg.get("pos_psd", 0.0))
    params["prnbias"] = _f(gnss_cfg["prnbias"])
    params["sig_p0"] = _f(gnss_cfg["sig_p0"])
    params["sig_v0"] = _f(gnss_cfg["sig_v0"])
    params["sig_n0"] = _f(gnss_cfg["sig_n0"])
    # RTKLIB ambiguity states are cycles.  Keep that legacy default explicit,
    # while allowing GREAT-compatible configurations to declare the initial
    # sigma in metres and convert per satellite at the RTK boundary.
    sig_n0_units = str(gnss_cfg.get("sig_n0_units", "cycles")).strip().lower()
    if sig_n0_units not in {"cycles", "m", "meter", "metre", "meters", "metres"}:
        raise ValueError(
            "gnss.sig_n0_units must be 'cycles' or a metre alias, "
            f"got '{sig_n0_units}'")
    params["sig_n0_units"] = sig_n0_units
    params["armode"] = _i(gnss_cfg["armode"])
    params["thresar"] = _f(gnss_cfg["thresar"])
    params["thresar1"] = _f(gnss_cfg["thresar1"])
    params["minlock"] = _i(gnss_cfg.get("minlock", 0))
    params["glo_hwbias"] = _f(gnss_cfg["glo_hwbias"])
    params["elmaskar"] = _f(gnss_cfg["elmaskar"])
    params["var_holdamb"] = _f(gnss_cfg["var_holdamb"])
    params["minfix"] = _i(gnss_cfg["minfix"])
    params["minfixsats"] = _i(gnss_cfg["minfixsats"])
    params["minholdsats"] = _i(gnss_cfg["minholdsats"])
    params["mindropsats"] = _i(gnss_cfg["mindropsats"])
    params["sing_p0"] = _f(gnss_cfg["sing_p0"])
    params["sing_v0"] = _f(gnss_cfg["sing_v0"])
    params["sing_elmin"] = _f(gnss_cfg["sing_elmin"])

    # 频率映射（freq_table/freq_ix0/freq_ix1/dfreq_glo）已不再是用户必填项:
    # 缺失时由 gnutlib 的 LibGnut 频率表自动派生（见 rinex_improve）。
    freq_keys_missing = any(
        gnss_cfg.get(key) is None
        for key in ("freq_table", "freq_ix0", "freq_ix1", "dfreq_glo"))
    if freq_keys_missing:
        from src.utility.rinex_improve import auto_freq_plan, default_band_plan
        nf = int(gnss_cfg.get("nf", 2) or 2)
        band_plan = gnss_cfg.get("band_plan")
        if not band_plan:
            band_plan = default_band_plan(gnss_cfg.get("gnss_t"), nf)
        derived = auto_freq_plan(band_plan, freq_table=gnss_cfg.get("freq_table"),
                                 dfreq_glo=gnss_cfg.get("dfreq_glo"),
                                 max_freq=max(nf, 2))
    params["freq"] = _fv(gnss_cfg["freq_table"] if gnss_cfg.get("freq_table") is not None
                         else derived["freq_table"])
    params["dfreq_glo"] = _fv(gnss_cfg["dfreq_glo"] if gnss_cfg.get("dfreq_glo") is not None
                              else derived["dfreq_glo"])
    params["rb"] = _fv(gnss_cfg["rb"])
    params["rr_f"] = _fv(gnss_cfg["rr_f"])
    params["rr_b"] = _fv(gnss_cfg["rr_b"])

    # err 数组: [_, base, el, bl, snr, rcvstd, satclk]
    params["err"] = [
        0,
        _f(gnss_cfg["err_base"]),
        _f(gnss_cfg["err_el"]),
        0.0,
        0,
        0,
        _f(gnss_cfg["err_satclk"]),
    ]

    cmap = _get_constellation_map()
    params["efact"] = {}
    for sat_str, val in [
        ("GPS", gnss_cfg["efact_gps"]),
        ("GLO", gnss_cfg["efact_glo"]),
        ("GAL", gnss_cfg["efact_gal"]),
        ("BDS", gnss_cfg.get("efact_bds", 1.0)),
        ("QZS", gnss_cfg.get("efact_qzs", 1.0)),
    ]:
        if sat_str in cmap:
            params["efact"][cmap[sat_str]] = _f(val)

    params["gnss_t"] = [cmap[s] for s in gnss_cfg["gnss_t"] if s in cmap]

    # 卫星剔除: remove_sat 为新的规范键（用户面向），excsats 为遗留别名。
    # rtklib-py 的 satexclude 以卫星号比较, 这里统一转换为卫星号,
    # 并保留无法解析的原始项以便诊断。
    remove_entries = list(gnss_cfg.get("remove_sat") or []) + \
        list(gnss_cfg.get("excsats") or [])
    from .rtklib.rtkcmn import id2sat
    excsat_nos = []
    for entry in remove_entries:
        sat_no = id2sat(str(entry).strip().upper())
        if sat_no > 0 and sat_no not in excsat_nos:
            excsat_nos.append(sat_no)
    params["excsats"] = excsat_nos

    if freq_keys_missing:
        params["freq_ix0"] = {
            cmap[k]: _i(v) for k, v in derived["freq_ix0"].items() if k in cmap}
        params["freq_ix1"] = {
            cmap[k]: _i(v) for k, v in derived["freq_ix1"].items() if k in cmap}
    else:
        params["freq_ix0"] = {cmap[k]: _i(v) for k, v in gnss_cfg["freq_ix0"].items() if k in cmap}
        params["freq_ix1"] = {cmap[k]: _i(v) for k, v in gnss_cfg["freq_ix1"].items() if k in cmap}

    # rtklib-py 的 rnx_decode 需要信号查找表与跳过表。
    # sig_tbl: rtklib-py 标准信号查找表。
    # skip_sig_tbl: 由 RINEX 简化器（rinex_simplifier）在解码前限制为 2 频点，
    #               因此这里默认不跳过任何信号。
    from .rtklib.rtkcmn import rSIG
    params["sig_tbl"] = {
        "1C": rSIG.L1C, "1X": rSIG.L1X, "1W": rSIG.L1W, "1P": rSIG.L1C, "1I": rSIG.L1C, "1M": rSIG.L1C, "1S": rSIG.L1C,
        "2W": rSIG.L2W, "2C": rSIG.L2C, "2X": rSIG.L2X, "2L": rSIG.L2L, "2P": rSIG.L2C, "2S": rSIG.L2C, "2I": rSIG.L7X, "2M": rSIG.L2C,
        "5Q": rSIG.L5Q, "5X": rSIG.L5X, "5P": rSIG.L5Q,
        "6C": rSIG.L7X, "6I": rSIG.L7X, "6X": rSIG.L7X,
        "7Q": rSIG.L7Q, "7X": rSIG.L7X, "7I": rSIG.L7X, "7P": rSIG.L7X,
        "8X": rSIG.L7X, "8Q": rSIG.L7X, "8P": rSIG.L7X,
    }
    params["skip_sig_tbl"] = {sat_enum: [] for sat_enum in cmap.values()}

    return params


class RtklibEnv:
    """rtklib-py 运行环境管理器。

    用法:
        env = RtklibEnv(config["gnss"])
        env.setup()
        nav = env.init_nav()
        ...
        env.cleanup()
    """

    def __init__(self, gnss_cfg: dict, library_path: str = None):
        if library_path is not None:
            warnings.warn(
                "library_path parameter is deprecated and ignored; "
                "rtklib-py is now absorbed into src/core/gnss/rtklib/",
                DeprecationWarning,
                stacklevel=2,
            )
        self.gnss_cfg = gnss_cfg
        self._setup_done = False

    def setup(self):
        """构建 params 并注入 config 单例，初始化 tracelevel。"""
        from .rtklib import config
        config.set_params(build_params(self.gnss_cfg))

        # rtklib-py 的 trace() 引用模块级 trace_level 变量，但该变量仅在
        # 调用 tracelevel() 后才存在。这里调用一次以初始化（设为 0 关闭日志），
        # 否则 pntpos/relpos 首次调用会抛 NameError。
        from .rtklib import rtkcmn as _gn
        if not hasattr(_gn, "trace_level"):
            _gn.tracelevel(0)
        self._setup_done = True

    def get_cfg(self):
        """返回 config 代理对象（setup 后可用）。

        代理对象通过 __getattr__ 访问 config._params 中的参数，
        用法与原 __ppk_config 模块对象一致：cfg.nf, cfg.elmin 等。
        """
        if not self._setup_done:
            self.setup()
        from .rtklib import config
        return config.cfg

    def init_nav(self):
        """调用 rtkinit(cfg) 返回 nav 对象。"""
        if not self._setup_done:
            self.setup()
        from .rtklib.rtkpos import rtkinit
        from .rtklib import config
        return rtkinit(config.cfg)

    def cleanup(self):
        """重置 config 单例（测试隔离用）。"""
        from .rtklib import config
        config.reset()
        self._setup_done = False
