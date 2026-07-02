"""用 rtklib-py 自己的 time2gpst 算 rover 历元的 GPST。"""
import sys
sys.path.insert(0, "library/rtklib-py/src")

# 注入 __ppk_config
import types
cfg = types.ModuleType("__ppk_config")
cfg.nf = 2
cfg.gnss_t = []
sys.modules["__ppk_config"] = cfg

import rtkcmn as gn

# 读 rover RINEX 首历元
with open('data/cpt0870.19o') as f:
    while True:
        line = f.readline()
        if 'END OF HEADER' in line:
            break
    first_epoch_line = None
    for line in f:
        if line.startswith('>'):
            first_epoch_line = line
            break

print(f"首历元行: {first_epoch_line.strip()}")
parts = first_epoch_line.split()
yyyy, mm, dd = int(parts[1]), int(parts[2]), int(parts[3])
hh, mi = int(parts[4]), int(parts[5])
ss = float(parts[6])
print(f"历元字段: {yyyy}-{mm}-{dd} {hh}:{mi}:{ss}")

# 用 rtklib-py 的 epoch2time
t = gn.epoch2time([yyyy, mm, dd, hh, mi, ss])
week, tow = gn.time2gpst(t)
print(f"rtklib-py 转换: week={week} tow={tow:.3f}")

# 对比 test_rtk.pos 首行
print(f"test_rtk.pos 首行: week=2569 tow=11853.000")

# 检查 RINEX 头的时间系统标识
with open('data/cpt0870.19o') as f:
    for _ in range(30):
        line = f.readline()
        if 'TIME OF FIRST OBS' in line:
            print(f"RINEX 头: {line.strip()}")
            break
