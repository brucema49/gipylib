"""分析位置/速度/姿态误差随时间的分布，定位失败时段。

RTKLC.rslt (LLH) vs rtktcgps.rslt (ECEF): ours LLH → ECEF → 与 ref 求差 → 转 ENU。
"""
import math
from pathlib import Path
import numpy as np


# LLH 字段索引 (ours)
LAT, LON, H = 2, 3, 4
VX, VY, VZ = 16, 17, 18
ROLL, PITCH, YAW = 25, 26, 27

# ECEF 字段索引 (ref)
X_ECEF, Y_ECEF, Z_ECEF = 2, 3, 4


def llh2ecef(lat_deg, lon_deg, h):
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    N = a / math.sqrt(1 - e2 * sl * sl)
    x = (N + h) * cl * math.cos(lon)
    y = (N + h) * cl * math.sin(lon)
    z = (N * (1 - e2) + h) * sl
    return x, y, z


def ecef2llh(x, y, z):
    a = 6378137.0
    e2 = 6.69437999014e-3
    b = a * math.sqrt(1 - e2)
    ep2 = (a * a - b * b) / (b * b)
    p = math.sqrt(x * x + y * y)
    theta = math.atan2(z * a, p * b)
    lon = math.atan2(y, x)
    lat = math.atan2(z + ep2 * b * math.sin(theta) ** 3,
                     p - e2 * a * math.cos(theta) ** 3)
    return lat, lon


def ecef_diff_to_enu(dx, dy, dz, lat, lon):
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    dE = -so * dx + co * dy
    dN = -sl * co * dx - sl * so * dy + cl * dz
    dU = cl * co * dx + cl * so * dy + sl * dz
    return dE, dN, dU


def load_ours(path):
    """加载 LLH 格式 RSLT (ours)。"""
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
            rec = {
                "lat": float(parts[LAT]), "lon": float(parts[LON]),
                "h": float(parts[H]),
                "qins": int(parts[6]),
                "vx": float(parts[VX]), "vy": float(parts[VY]), "vz": float(parts[VZ]),
                "sow": sow,
            }
            if len(parts) >= 28:
                rec["roll"] = float(parts[ROLL])
                rec["pitch"] = float(parts[PITCH])
                rec["yaw"] = float(parts[YAW])
            key = (week, round(sow, 3))
            data[key] = rec
    return data


def load_ref(path):
    """加载 ECEF 格式 RSLT (ref)。"""
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
            rec = {
                "x": float(parts[X_ECEF]), "y": float(parts[Y_ECEF]),
                "z": float(parts[Z_ECEF]),
                "vx": float(parts[VX]), "vy": float(parts[VY]), "vz": float(parts[VZ]),
                "sow": sow,
            }
            key = (week, round(sow, 3))
            data[key] = rec
    return data


def main():
    ours = load_ours("data/output/RTKLC.rslt")
    ref = load_ref("data/rtktcgps.rslt")
    common = sorted(set(ours.keys()) & set(ref.keys()))

    times = []
    planar = []
    dU_arr = []
    dv_abs = []
    qins_ours = []
    for k in common:
        o = ours[k]
        r = ref[k]
        ox, oy, oz = llh2ecef(o["lat"], o["lon"], o["h"])
        dx = ox - r["x"]
        dy = oy - r["y"]
        dz = oz - r["z"]
        lat_r, lon_r = ecef2llh(r["x"], r["y"], r["z"])
        dE, dN, dU = ecef_diff_to_enu(dx, dy, dz, lat_r, lon_r)
        times.append(r["sow"])
        planar.append(math.sqrt(dE * dE + dN * dN))
        dU_arr.append(dU)
        dvx = o["vx"] - r["vx"]
        dvy = o["vy"] - r["vy"]
        dvz = o["vz"] - r["vz"]
        dv_abs.append(math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz))
        qins_ours.append(o["qins"])

    times = np.array(times)
    planar = np.array(planar)
    dU_arr = np.array(dU_arr)
    dv_abs = np.array(dv_abs)
    qins = np.array(qins_ours)

    t0 = times.min()
    t_max = times.max()
    print(f"时间范围: {t0:.3f} ~ {t_max:.3f} ({t_max - t0:.1f}s)")

    # ===== 位置+速度 30s 区间统计 =====
    print(f"\n=== 按 30s 区间统计 (位置 ENU/速度 ECEF) ===")
    print(f"{'时段':>12} {'历元':>5} | "
          f"{'平面max':>9} {'平面mean':>9} {'>0.5m':>5} | "
          f"{'高程max':>9} {'>1m':>5} | "
          f"{'速度max':>9} {'速度mean':>9} {'>0.5':>5}")
    bin_size = 30.0
    bin_start = t0
    while bin_start < t_max:
        bin_end = bin_start + bin_size
        mask = (times >= bin_start) & (times < bin_end)
        if mask.sum() > 0:
            p_bin = planar[mask]
            u_bin = np.abs(dU_arr[mask])
            v_bin = dv_abs[mask]
            pf = int((p_bin > 0.5).sum())
            ef = int((u_bin > 1.0).sum())
            vf = int((v_bin > 0.5).sum())
            print(f"{bin_start-t0:>8.0f}-{bin_end-t0:>3.0f}s {mask.sum():>5} | "
                  f"{p_bin.max():>9.4f} {p_bin.mean():>9.4f} {pf:>5} | "
                  f"{u_bin.max():>9.4f} {ef:>5} | "
                  f"{v_bin.max():>9.4f} {v_bin.mean():>9.4f} {vf:>5}")
        bin_start = bin_end

    # ===== 姿态 30s 区间统计 (仅 ours) =====
    att_keys = sorted(ours.keys())
    att_keys_with_att = [k for k in att_keys if "roll" in ours[k]]
    if att_keys_with_att:
        print(f"\n=== 按 30s 区间统计 (姿态, deg) ===")
        print(f"{'时段':>12} {'历元':>5} | "
              f"{'roll_mean':>9} {'roll_std':>9} | "
              f"{'pitch_mean':>10} {'pitch_std':>10} | "
              f"{'yaw_mean':>9} {'yaw_std':>9}")
        att_times = np.array([ours[k]["sow"] for k in att_keys_with_att])
        roll_full = np.array([ours[k]["roll"] for k in att_keys_with_att])
        pitch_full = np.array([ours[k]["pitch"] for k in att_keys_with_att])
        yaw_full = np.array([ours[k]["yaw"] for k in att_keys_with_att])
        at0 = att_times.min()
        bin_start = at0
        at_max = att_times.max()
        while bin_start < at_max:
            bin_end = bin_start + bin_size
            mask = (att_times >= bin_start) & (att_times < bin_end)
            if mask.sum() > 0:
                print(f"{bin_start-at0:>8.0f}-{bin_end-at0:>3.0f}s {mask.sum():>5} | "
                      f"{roll_full[mask].mean():>9.3f} {roll_full[mask].std():>9.3f} | "
                      f"{pitch_full[mask].mean():>10.3f} {pitch_full[mask].std():>10.3f} | "
                      f"{yaw_full[mask].mean():>9.3f} {yaw_full[mask].std():>9.3f}")
            bin_start = bin_end

    # ===== 失败历元分布 =====
    print(f"\n=== 平面 > 0.5m 的历元分布 (按 30s bin) ===")
    fail_mask = planar > 0.5
    fail_times = times[fail_mask]
    if len(fail_times) > 0:
        print(f"总失败历元: {len(fail_times)}")
        bin_start = t0
        while bin_start < t_max:
            bin_end = bin_start + bin_size
            n_fail = int(((fail_times >= bin_start) & (fail_times < bin_end)).sum())
            if n_fail > 0:
                print(f"  [{bin_start-t0:.0f}s, {bin_end-t0:.0f}s): {n_fail} 个")
            bin_start = bin_end

    print(f"\n=== 速度 > 0.5m/s 的历元分布 (按 30s bin) ===")
    fail_mask_v = dv_abs > 0.5
    fail_times_v = times[fail_mask_v]
    if len(fail_times_v) > 0:
        print(f"总失败历元: {len(fail_times_v)}")
        bin_start = t0
        while bin_start < t_max:
            bin_end = bin_start + bin_size
            n_fail = int(((fail_times_v >= bin_start) & (fail_times_v < bin_end)).sum())
            if n_fail > 0:
                print(f"  [{bin_start-t0:.0f}s, {bin_end-t0:.0f}s): {n_fail} 个")
            bin_start = bin_end

    # ===== 前 10 个失败历元 =====
    print(f"\n=== 前 10 个平面失败历元 ===")
    fail_indices = np.where(fail_mask)[0][:10]
    for i in fail_indices:
        print(f"  sow={times[i]:.3f} (t+{times[i]-t0:.1f}s) "
              f"planar={planar[i]:.4f}m dU={dU_arr[i]:.4f}m "
              f"dv={dv_abs[i]:.4f}m/s qins={qins[i]}")

    print(f"\n=== 前 10 个速度失败历元 ===")
    fail_indices_v = np.where(fail_mask_v)[0][:10]
    for i in fail_indices_v:
        print(f"  sow={times[i]:.3f} (t+{times[i]-t0:.1f}s) "
              f"dv={dv_abs[i]:.4f}m/s planar={planar[i]:.4f}m "
              f"dU={dU_arr[i]:.4f}m qins={qins[i]}")


if __name__ == "__main__":
    main()
