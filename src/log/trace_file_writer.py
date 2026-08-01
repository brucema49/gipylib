"""trace 文件输出器: 捕获 rtklib-py trace() 输出到 .trace 文件。

trace_level:
  0 = off (不生成 .trace 文件)
  1 = info (基本运行信息)
  2 = detail (详细的解算过程)
  3 = debug (含状态向量等调试信息)

trace 文件与定位结果文件同名, 扩展名 .trace, 自动生成到输出目录:
  RTK.pos  → RTK.trace
  RTKLC.rslt → RTKLC.trace

时间格式统一为 GPS 周 + 周内秒 (week sow), 与 .pos/.rslt 一致。
未初始化时的无效数据 (pos=[0,0,0], x[clk]=N/A, clk_stored=[0,0,0]) 不输出。
"""
import os
import re
from pathlib import Path
from typing import Optional

from src.core.time_utils import unix_to_gpst


# 匹配 "t=<unix_ts>" 形式的时间戳 (如 t=1553744090.000)
_UNIX_TS_PATTERN = re.compile(r't=(\d{10}\.\d+)')


def _is_invalid_state_line(msg: str) -> bool:
    """检测无效状态行: 含 pos=[0. 0. 0.] / x[clk]=N/A / clk_stored=[0. 0. 0.]。

    这些字段在滤波器未初始化时无意义, 不写入 trace 文件。
    """
    invalid_markers = (
        "pos=[0. 0. 0.]",
        "x[clk]=N/A",
        "clk_stored=[0. 0. 0.]",
        "x[pos]=[0. 0. 0.]",
    )
    return any(m in msg for m in invalid_markers)


def _convert_timestamp(msg: str) -> str:
    """把消息中的 Unix 时间戳 t=XXX 转为 GPS 周+周内秒格式。

    例: 't=1553744090.000' → 'week=2035 sow=454690.000'
    """
    def _replace(match):
        unix_ts = float(match.group(1))
        week, sow = unix_to_gpst(unix_ts)
        return "week=%d sow=%.3f" % (week, sow)

    return _UNIX_TS_PATTERN.sub(_replace, msg)


class TraceFileWriter:
    """trace 文件输出器: 重定向 rtklib-py trace() 到 .trace 文件。

    用法:
        writer = TraceFileWriter(output_dir, "RTK.pos", trace_level=3)
        writer.open()   # 自动 patch rtklib-py trace()
        ... 运行解算 ...
        writer.close()  # 恢复原 trace() 并关闭文件
    """

    def __init__(self, output_dir: str, ref_filename: str,
                 trace_level: int = 0):
        self.output_dir = output_dir
        # RTK.pos → RTK.trace, RTKLC.rslt → RTKLC.trace
        stem = Path(ref_filename).stem
        self.filename = stem + ".trace"
        self.trace_level = int(trace_level)
        self._fp = None
        self._closed = False
        self._original_trace = None
        self._original_tracelevel = None

    @property
    def enabled(self) -> bool:
        return self.trace_level > 0

    def open(self) -> None:
        """打开 trace 文件并 patch rtklib-py 的 trace() 函数。"""
        if not self.enabled:
            return
        os.makedirs(self.output_dir, exist_ok=True)
        path = Path(self.output_dir) / self.filename
        self._fp = open(path, "w", encoding="utf-8")
        self._fp.write(
            "# GInsStream trace file (level=%d)\n" % self.trace_level
        )
        self._fp.write("# 时间格式: GPS 周 + 周内秒 (week sow)\n\n")
        self._closed = False
        self._patch_rtklib_trace()

    def _patch_rtklib_trace(self) -> None:
        """替换 rtklib-py 的 trace() 和 tracelevel() 函数。

        把原本输出到 stderr 的 trace 重定向到 .trace 文件,
        并将时间戳转为 GPS 周+周内秒, 过滤无效状态数据。
        """
        try:
            from src.core.gnss.rtklib import rtkcmn as _gn
        except ImportError:
            return

        # 保存原始函数
        self._original_trace = _gn.trace
        self._original_tracelevel = _gn.tracelevel
        fp = self._fp
        level = self.trace_level

        def _patched_trace(lv, msg):
            if lv > level:
                return
            # 过滤无效状态数据
            if _is_invalid_state_line(msg):
                return
            # 转换时间戳格式
            converted = _convert_timestamp(msg)
            fp.write("%d %s" % (lv, converted))
            fp.flush()

        def _patched_tracelevel(lv):
            # 忽略低于配置等级的设置 (防止 RtklibEnv.setup() 重置为 0)
            if lv < level:
                return
            _gn.trace_level = lv

        _gn.trace = _patched_trace
        _gn.tracelevel = _patched_tracelevel
        _gn.trace_level = level

    def _restore_rtklib_trace(self) -> None:
        """恢复 rtklib-py 原始 trace() 函数。"""
        if self._original_trace is None:
            return
        try:
            from src.core.gnss.rtklib import rtkcmn as _gn
            _gn.trace = self._original_trace
            _gn.tracelevel = self._original_tracelevel
            _gn.trace_level = 0
        except ImportError:
            pass
        self._original_trace = None
        self._original_tracelevel = None

    def write(self, level: int, msg: str) -> None:
        """直接写入一条 trace 消息 (供非 rtklib-py 模块使用)。"""
        if not self.enabled or self._fp is None:
            return
        if level > self.trace_level:
            return
        if _is_invalid_state_line(msg):
            return
        converted = _convert_timestamp(msg)
        self._fp.write("%d %s" % (level, converted))
        self._fp.flush()

    def close(self) -> None:
        if self._closed:
            return
        self._restore_rtklib_trace()
        if self._fp is not None:
            self._fp.close()
            self._fp = None
        self._closed = True
