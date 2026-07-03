"""rtklib-py 核心算法子包。

从 library/rtklib-py/src/ 吸收，改为相对导入。
公共 API 通过此 __init__.py 导出，方便外部引用。
"""
from . import config  # 确保配置单例可用
from .rtkcmn import Sol, gtime_t, uGNSS, rSIG, rCST
from .rinex import rnx_decode
from .pntpos import pntpos
from .rtkpos import relpos, timediff, rtkinit, rtkpos
from .postpos import procpos, savesol
