"""共享的基站 RINEX 加载 (支持多个连续短时段文件)。

campus01 的基站观测被切成 15 min 一个文件, 而运行窗口跨越文件边界, 因此
``base_path`` 允许是字符串或字符串列表。每个文件用独立解码器读取后把历元
并入同一列表, 避免重复读头污染 ``sig``/``nobs`` 表。
"""
from typing import Callable, List


def base_paths(gnss_cfg: dict) -> List[str]:
    """把 ``gnss.base_path`` 归一化为路径列表。"""
    raw = gnss_cfg.get("base_path")
    if raw is None:
        return []
    if isinstance(raw, (list, tuple)):
        return [str(item) for item in raw if str(item)]
    return [str(raw)]


def load_base(
    env,
    nav,
    gnss_cfg: dict,
    prepare_rinex: Callable[[str], str],
    raw_band_priority=None,
):
    """读取全部基站文件并返回合并后的基站解码器。

    Args:
        env: ``RtklibEnv`` (需已 ``setup()``)
        nav: 已加载星历的导航对象 (``nav.rb`` 为 0 时用基站头文件坐标回填)
        gnss_cfg: ``gnss`` 配置段
        prepare_rinex: 单文件预处理回调 (CRX 解压/裁剪), 返回可读路径
    """
    from src.core.gnss.rtklib import rinex as rn

    paths = base_paths(gnss_cfg)
    if not paths:
        raise ValueError("base_path is required for rtk/rtd positioning")

    base = rn.rnx_decode(
        env.get_cfg(), raw_band_priority=raw_band_priority
    )
    for index, path in enumerate(paths):
        prepared = prepare_rinex(path)
        if index == 0:
            base.decode_obsfile(nav, prepared, None)
            continue
        more = rn.rnx_decode(
            env.get_cfg(), raw_band_priority=raw_band_priority
        )
        more.decode_obsfile(nav, prepared, None)
        base.obslist.extend(more.obslist)

    if len(paths) > 1:
        base.obslist.sort(key=lambda ob: float(ob.t.time + ob.t.sec))
        base.index = 0
    if nav.rb[0] == 0:
        nav.rb = base.pos
    return base
