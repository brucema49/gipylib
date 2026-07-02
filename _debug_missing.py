"""找出 test_rtk.pos 中缺失的 18 个历元并分析失败原因。"""
from datetime import datetime, timedelta

# RINEX 头标识 "GPS TIME"，历元本身就是 GPST，无需闰秒转换
GPST_EPOCH = datetime(1980, 1, 6)


def gpst_components_to_sow(yyyy, mm, dd, hh, mi, ss):
    """GPST 历元分量 → (week, sow)。"""
    gpst_dt = datetime(yyyy, mm, dd, hh, mi) + timedelta(seconds=ss)
    delta = gpst_dt - GPST_EPOCH
    total_sec = int(delta.total_seconds())
    week = total_sec // 604800
    sow = total_sec - week * 604800
    return week, sow


# 1. 读 rover RINEX 所有历元
rover_epochs = []
with open('data/cpt0870.19o') as f:
    while True:
        line = f.readline()
        if 'END OF HEADER' in line:
            break
    for line in f:
        if line.startswith('>'):
            parts = line.split()
            yyyy, mm, dd = int(parts[1]), int(parts[2]), int(parts[3])
            hh, mi = int(parts[4]), int(parts[5])
            ss = float(parts[6])
            rover_epochs.append(gpst_components_to_sow(yyyy, mm, dd, hh, mi, ss))

# 2. 读 test_rtk.pos 时间戳
rtk_sows = set()
with open('output/test_rtk.pos') as f:
    next(f)
    for line in f:
        parts = line.split()
        rtk_sows.add((int(parts[0]), float(parts[1])))

# 3. 找出缺失历元
missing = [(w, s) for (w, s) in rover_epochs if (w, s) not in rtk_sows]
print(f"rover 历元数: {len(rover_epochs)}")
print(f"rtk 输出行数: {len(rtk_sows)}")
print(f"缺失历元数: {len(missing)}")
print("\n缺失历元 (week, sow) 及在 rover 中的索引:")
for ms in missing:
    if ms in rover_epochs:
        idx = rover_epochs.index(ms)
        prev_ms = rover_epochs[idx - 1] if idx > 0 else None
        next_ms = rover_epochs[idx + 1] if idx < len(rover_epochs) - 1 else None
        gap_prev = (ms[1] - prev_ms[1]) if prev_ms else 0
        gap_next = (next_ms[1] - ms[1]) if next_ms else 0
        # 转回 GPST 日期
        total = ms[0] * 604800 + int(ms[1])
        gpst_dt = GPST_EPOCH + timedelta(seconds=total)
        gpst_str = gpst_dt.strftime("%Y-%m-%d %H:%M:%S")
        print(f"  idx={idx:4d}  week={ms[0]} sow={ms[1]:.3f}  GPST={gpst_str}  prev_gap={gap_prev:.1f}s next_gap={gap_next:.1f}s")
