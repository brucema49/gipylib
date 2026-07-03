"""rtklib-py 配置单例。RtklibEnv.setup() 调用 set_params() 注入参数。

替代原 rtklib-py 的 __ppk_config 动态模块机制。
rtkpos.py / postpos.py 通过 `from . import config as cfg` 引用，
用法不变：cfg.nf, cfg.pos_elmin, cfg.cnr_min 等。
"""
_params = {}


class _CfgProxy:
    """属性代理，从 _params 字典取值。"""

    def __getattr__(self, name):
        try:
            return _params[name]
        except KeyError:
            raise AttributeError(
                f"rtklib config has no attribute '{name}'. "
                f"Did you call RtklibEnv.setup()?"
            )


cfg = _CfgProxy()


def set_params(params: dict) -> None:
    """RtklibEnv 调用此函数注入配置参数。

    params 字典的 key 应与原 __ppk_config.py 的模块级变量名一致，
    如 nf, elmin, cnr_min, eratio, armode 等。
    """
    _params.update(params)


def reset() -> None:
    """清空配置（测试隔离用）。"""
    _params.clear()
