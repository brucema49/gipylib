"""分析 18 个失败历元的具体原因。

开启 trace level 2，逐历元跑 RTK，对失败历元打印关键诊断信息。
"""
import sys
import io
from pathlib import Path
from datetime import datetime, timedelta

# 18 个失败历元的 sow（来自 _debug_missing.py）
FAIL_SOWS = {
    358183, 358366, 358422, 358493, 358650, 358703, 358763,
    358924, 359020, 359112, 359173, 359272, 359393, 359461,
    359516, 359579, 359628, 359709,
}

# 项目根
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import yaml

with open("data/cfg_test_rtk.yaml") as f:
    config = yaml.safe_load(f)

# 1. 初始化 rtklib 环境
from src.core.gnss.rtklib_config_adapter import RtklibEnv
from src.utility.rinex_simplifier import needs_simplification, simplify_rinex
import tempfile

env = RtklibEnv(config["gnss"], library_path="library/rtklib-py/src")
env.setup()
nav = env.init_nav()

# 2. 开启 trace level 3（捕获 relpos 关键决策: no common sats / not enough ddres / Age of）
import rtkcmn as gn
gn.tracelevel(3)

# 用 StringIO 捕获 stderr（rtklib-py 的 trace 写到 sys.stderr）
trace_buffer = io.StringIO()
_original_stderr = sys.stderr
sys.stderr = trace_buffer

# 3. 准备 RINEX（简化）
def prepare_rinex(path):
    if not needs_simplification(path):
        return path
    suffix = Path(path).suffix
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=suffix, delete=False, encoding="utf-8")
    tmp.close()
    simplify_rinex(path, tmp.name)
    return tmp.name

rover_path = prepare_rinex(config["gnss"]["rover_path"])
base_path = prepare_rinex(config["gnss"]["base_path"])

import rinex as rn
rov = rn.rnx_decode(env.get_cfg())
rov.decode_obsfile(nav, rover_path, None)
rov.decode_nav(config["gnss"]["eph_path"], nav)

base = rn.rnx_decode(env.get_cfg())
base.decode_obsfile(nav, base_path, None)
if nav.rb[0] == 0:
    nav.rb = base.pos

# 4. 创建 RtkProcessor
from src.core.gnss.rtk_processor import RtkProcessor
processor = RtkProcessor(nav)

# 5. 遍历历元，捕获失败历元 trace
dir = 1
obsr, obsb = rn.first_obs(nav, rov, base, dir)
fail_idx = 0
total_idx = 0
GPST_EPOCH = datetime(1980, 1, 6)

PRINT_LIMIT = 2  # 只详细打印前 2 个失败历元
printed = 0

while True:
    if obsr == []:
        break

    # 计算当前历元的 sow
    week, sow = gn.time2gpst(obsr.t)
    is_fail_epoch = sow in FAIL_SOWS
    if is_fail_epoch and printed >= PRINT_LIMIT:
        is_fail_epoch = False  # 不再详细打印

    # 对失败历元，记录 trace 起始位置
    trace_start = trace_buffer.tell() if is_fail_epoch else 0

    # 调用解算
    sol = processor.process_epoch(obsr, obsb)

    if is_fail_epoch:
        # 读取本次历元的 trace
        trace_buffer.seek(trace_start)
        epoch_trace = trace_buffer.read()
        trace_buffer.seek(0, 2)  # 回到末尾
        printed += 1

        # 转换时间戳为可读格式
        total_sec = week * 604800 + int(sow)
        gpst_dt = GPST_EPOCH + timedelta(seconds=total_sec)
        gpst_str = gpst_dt.strftime("%H:%M:%S")

        print(f"\n{'='*70}")
        print(f"失败历元 idx={total_idx} week={week} sow={sow} GPST={gpst_str}")
        print(f"  obsr 卫星数: {len(obsr.sat)}, obsb 卫星数: {len(obsb.sat)}")
        print(f"  nav.dt (差分龄期): {nav.dt:.3f}s (maxage={config['gnss']['maxage']}s)")
        print(f"  sol.stat: {sol.quality if sol else 'None (SOLQ_NONE)'}")
        if sol:
            print(f"  sol.ns: {sol.num_sv}, position: {sol.position}")
        print(f"  trace 关键行:")
        for line in epoch_trace.split('\n'):
            # 只显示关键决策行
            if any(kw in line for kw in ['relpos', 'no common', 'not enough', 'Age of',
                                          'ddres', 'ns=', 'stat=', 'ratio', 'valpos',
                                          'exclude', 'sats=', 'common sats',
                                          'outlier', 'slip', 'sat=', 'udstate', 'udbias']):
                print(f"    {line.strip()}")

    total_idx += 1
    obsr, obsb = rn.next_obs(nav, rov, base, dir)

# 恢复 stderr
sys.stderr = _original_stderr

print(f"\n{'='*70}")
print(f"总历元数: {total_idx}")
