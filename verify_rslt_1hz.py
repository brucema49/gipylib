"""1Hz 整数秒对比验证 (与之前 RTKLC.pos 验证一致)。

只对比整数秒历元（容差 0.005s），等价于之前的 .pos 文件对比。
"""
import sys
from pathlib import Path
import numpy as np


def load_rslt(path):
    data = {}
    with open(path, "r") as f:
        for line in f:
            if line.startswith("%") or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 17:
                continue
            week = int(parts[0])
            sow = float(parts[1])
            x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
            key = (week, round(sow, 3))
            data[key] = (x, y, z, sow)
    return data


def main():
    ours = load_rslt("data/output/RTKLC.rslt")
    ref = load_rslt("data/rtktcgps.rslt")

    # 只取整数秒历元（sow 接近整数）
    tol = 0.005
    ours_int = {k: v for k, v in ours.items() if abs(v[3] - round(v[3])) < tol}
    ref_int = {k: v for k, v in ref.items() if abs(v[3] - round(v[3])) < tol}

    common = sorted(set(ours_int.keys()) & set(ref_int.keys()))
    print(f"1Hz 整数秒对比:")
    print(f"  ours 整数秒历元: {len(ours_int)}")
    print(f"  ref  整数秒历元: {len(ref_int)}")
    print(f"  重叠: {len(common)}")

    dx, dy, dz = [], [], []
    for k in common:
        x_o, y_o, z_o, _ = ours_int[k]
        x_r, y_r, z_r, _ = ref_int[k]
        dx.append(x_o - x_r)
        dy.append(y_o - y_r)
        dz.append(z_o - z_r)

    dx = np.array(dx)
    dy = np.array(dy)
    dz = np.array(dz)
    planar = np.sqrt(dx ** 2 + dy ** 2)

    print(f"\n=== 1Hz 位置误差统计 (m) ===")
    print(f"  平面: max={planar.max():.4f}, mean={planar.mean():.4f}, p99={np.percentile(planar, 99):.4f}")
    print(f"  高程: max={np.abs(dz).max():.4f}, mean={np.abs(dz).mean():.4f}, p99={np.percentile(np.abs(dz), 99):.4f}")

    planar_fail = (planar > 0.5).sum()
    elev_fail = (np.abs(dz) > 1.0).sum()
    total = len(common)
    print(f"\n=== 1Hz 硬约束 ===")
    print(f"  平面 ≤ 0.5m: {planar_fail}/{total} 失败 ({100 * planar_fail / total:.2f}%)")
    print(f"  高程 ≤ 1.0m: {elev_fail}/{total} 失败 ({100 * elev_fail / total:.2f}%)")
    pass_rate = 100 * (1 - (planar_fail + elev_fail) / total)
    print(f"  通过率: {pass_rate:.2f}%")

    # 同时段 100Hz 对比（相同时间窗口）
    t0 = ref_int[common[0]][3]
    t_end = ref_int[common[-1]][3]
    print(f"\n=== 同时段 100Hz 对比 (sow {t0:.3f} ~ {t_end:.3f}) ===")
    ours_window = {k: v for k, v in ours.items() if t0 <= v[3] <= t_end}
    ref_window = {k: v for k, v in ref.items() if t0 <= v[3] <= t_end}
    common_100 = sorted(set(ours_window.keys()) & set(ref_window.keys()))
    dx2, dy2, dz2 = [], [], []
    for k in common_100:
        x_o, y_o, z_o, _ = ours_window[k]
        x_r, y_r, z_r, _ = ref_window[k]
        dx2.append(x_o - x_r)
        dy2.append(y_o - y_r)
        dz2.append(z_o - z_r)
    dx2 = np.array(dx2)
    dy2 = np.array(dy2)
    dz2 = np.array(dz2)
    planar2 = np.sqrt(dx2 ** 2 + dy2 ** 2)
    p2_fail = (planar2 > 0.5).sum()
    e2_fail = (np.abs(dz2) > 1.0).sum()
    print(f"  重叠历元: {len(common_100)}")
    print(f"  平面: max={planar2.max():.4f}, mean={planar2.mean():.4f}")
    print(f"  高程: max={np.abs(dz2).max():.4f}, mean={np.abs(dz2).mean():.4f}")
    print(f"  平面失败: {p2_fail}/{len(common_100)} ({100 * p2_fail / len(common_100):.2f}%)")
    print(f"  高程失败: {e2_fail}/{len(common_100)} ({100 * e2_fail / len(common_100):.2f}%)")

    # 排除前 180s 收敛期后的 100Hz 对比
    t_steady = t0 + 180.0
    print(f"\n=== 排除前 180s 收敛期后 100Hz 对比 (sow {t_steady:.3f} ~ {t_end:.3f}) ===")
    ours_steady = {k: v for k, v in ours.items() if t_steady <= v[3] <= t_end}
    ref_steady = {k: v for k, v in ref.items() if t_steady <= v[3] <= t_end}
    common_s = sorted(set(ours_steady.keys()) & set(ref_steady.keys()))
    dx3, dy3, dz3 = [], [], []
    for k in common_s:
        x_o, y_o, z_o, _ = ours_steady[k]
        x_r, y_r, z_r, _ = ref_steady[k]
        dx3.append(x_o - x_r)
        dy3.append(y_o - y_r)
        dz3.append(z_o - z_r)
    dx3 = np.array(dx3)
    dy3 = np.array(dy3)
    dz3 = np.array(dz3)
    planar3 = np.sqrt(dx3 ** 2 + dy3 ** 2)
    p3_fail = (planar3 > 0.5).sum()
    e3_fail = (np.abs(dz3) > 1.0).sum()
    total3 = len(common_s)
    print(f"  重叠历元: {total3}")
    print(f"  平面: max={planar3.max():.4f}, mean={planar3.mean():.4f}, p99={np.percentile(planar3, 99):.4f}")
    print(f"  高程: max={np.abs(dz3).max():.4f}, mean={np.abs(dz3).mean():.4f}, p99={np.percentile(np.abs(dz3), 99):.4f}")
    print(f"  平面失败: {p3_fail}/{total3} ({100 * p3_fail / total3:.2f}%)")
    print(f"  高程失败: {e3_fail}/{total3} ({100 * e3_fail / total3:.2f}%)")
    print(f"  通过率: {100 * (1 - (p3_fail + e3_fail) / total3):.2f}%")


if __name__ == "__main__":
    main()
