"""对比 spplc.rslt (LLH) 与 truth.csv (ECEF) 的位置/速度/姿态精度。

spplc.rslt: LLH (lat, lon, h) + ECEF 速度 + roll/pitch/yaw, 100Hz
truth.csv:   ECEF (pos_x, pos_y, pos_z) + ECEF 速度 + att_roll/pitch/yaw, 1Hz
按 (week, sow) 匹配, 容差 0.005s (取 1Hz 整数秒)。
"""
import math
import sys
from pathlib import Path

import numpy as np

# spplc.rslt 字段索引 (LLH)
# 0:week 1:sow 2:lat 3:lon 4:h 5:Q 6:Qins 7:ns
# 16:vx 17:vy 18:vz
# 25:roll 26:pitch 27:yaw 28:sdroll 29:sdpitch 30:sdyaw
W, SOW, LAT, LON, H = 0, 1, 2, 3, 4
Q, QINS, NS = 5, 6, 7
VX, VY, VZ = 16, 17, 18
ROLL, PITCH, YAW = 25, 26, 27
SDROLL, SDPITCH, SDYAW = 28, 29, 30


def load_rslt(path):
    """加载 rslt, 对每个整数秒保留最接近的历元 (而非后写覆盖)。

    100Hz IMU 时间戳如 X.006, X.016, ..., X.496 都 round 到 X。
    后写覆盖会取 X.496 (GNSS 更新后 0.496s, INS 漂移最大)。
    正确做法: 取 abs(sow - round(sow)) 最小的历元 (X.006, Qins=3, 位置≈GNSS)。
    """
    data = {}
    with open(path) as f:
        for line in f:
            if line.startswith("%") or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 17:
                continue
            week = int(parts[W])
            sow = float(parts[SOW])
            rec = {
                "lat": float(parts[LAT]), "lon": float(parts[LON]),
                "h": float(parts[H]),
                "vx": float(parts[VX]), "vy": float(parts[VY]),
                "vz": float(parts[VZ]),
                "q": int(parts[Q]), "qins": int(parts[QINS]),
                "ns": int(parts[NS]), "sow": sow,
            }
            if len(parts) > YAW:
                rec["roll"] = float(parts[ROLL])
                rec["pitch"] = float(parts[PITCH])
                rec["yaw"] = float(parts[YAW])
            if len(parts) > SDYAW:
                rec["sdroll"] = float(parts[SDROLL])
                rec["sdpitch"] = float(parts[SDPITCH])
                rec["sdyaw"] = float(parts[SDYAW])
            # 按最近整数秒匹配, 优先取 1Hz 有效输出 (Qins=0 纯GNSS / Qins=3 GNSS更新后)
            # Qins=2 是 100Hz 中间历元 (INS 漂移), 不代表 1Hz 输出精度
            key = (week, int(round(sow)))
            qins = int(parts[QINS])
            dist = abs(sow - round(sow))
            if qins not in (0, 3):
                continue  # 跳过 100Hz 中间历元
            if key not in data or dist < data[key]["_dist"]:
                rec["_dist"] = dist
                data[key] = rec
    return data


def load_truth(path):
    data = {}
    with open(path) as f:
        f.readline()  # header
        for line in f:
            if not line.strip():
                continue
            p = line.split(",")
            week = int(p[0])
            sow = float(p[1])
            data[(week, int(round(sow)))] = {
                "x": float(p[3]), "y": float(p[4]), "z": float(p[5]),
                "vx": float(p[9]), "vy": float(p[10]), "vz": float(p[11]),
                "roll": float(p[12]), "pitch": float(p[13]),
                "yaw": float(p[14]), "sow": sow,
            }
    return data


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


def wrap_deg(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def main():
    ours = load_rslt("data/output/spplc.rslt")
    truth = load_truth("data/truth.csv")
    print(f"spplc.rslt 历元: {len(ours)}")
    print(f"truth.csv 历元: {len(truth)}")

    common = sorted(set(ours.keys()) & set(truth.keys()))
    print(f"重叠 (1Hz 整数秒): {len(common)}")
    if not common:
        print("ERROR: 无重叠历元")
        sys.exit(1)

    dE_a, dN_a, dU_a = [], [], []
    dv_a = []
    roll_e, pitch_e, yaw_e = [], [], []
    qins_arr = []
    times = []
    for k in common:
        o, t = ours[k], truth[k]
        ox, oy, oz = llh2ecef(o["lat"], o["lon"], o["h"])
        dx = ox - t["x"]
        dy = oy - t["y"]
        dz = oz - t["z"]
        lat, lon = ecef2llh(t["x"], t["y"], t["z"])
        dE, dN, dU = ecef_diff_to_enu(dx, dy, dz, lat, lon)
        dE_a.append(dE); dN_a.append(dN); dU_a.append(dU)
        dv = math.sqrt((o["vx"] - t["vx"]) ** 2 +
                       (o["vy"] - t["vy"]) ** 2 +
                       (o["vz"] - t["vz"]) ** 2)
        dv_a.append(dv)
        if "roll" in o:
            roll_e.append(wrap_deg(o["roll"] - t["roll"]))
            pitch_e.append(wrap_deg(o["pitch"] - t["pitch"]))
            yaw_e.append(wrap_deg(o["yaw"] - t["yaw"]))
        qins_arr.append(o["qins"])
        times.append(t["sow"])

    dE = np.array(dE_a); dN = np.array(dN_a); dU = np.array(dU_a)
    planar = np.sqrt(dE ** 2 + dN ** 2)
    pos3d = np.sqrt(dE ** 2 + dN ** 2 + dU ** 2)
    dv = np.array(dv_a)
    times = np.array(times)
    t0 = times.min()
    total = len(common)

    # ===== 位置误差 =====
    print("\n=== 位置误差 (m, ENU) ===")
    print(f"  3D RMSE = {math.sqrt((dE**2+dN**2+dU**2).mean()):.4f}")
    print(f"  平面: max={planar.max():.4f}, mean={planar.mean():.4f}, "
          f"p99={np.percentile(planar, 99):.4f}")
    print(f"  高程: max={np.abs(dU).max():.4f}, mean={np.abs(dU).mean():.4f}, "
          f"p99={np.percentile(np.abs(dU), 99):.4f}")

    # ===== 速度误差 =====
    print("\n=== 速度误差 (m/s) ===")
    print(f"  |dv|: max={dv.max():.4f}, mean={dv.mean():.4f}, "
          f"p99={np.percentile(dv, 99):.4f}")

    # ===== 姿态误差 =====
    if roll_e:
        re = np.array(roll_e); pe = np.array(pitch_e); ye = np.array(yaw_e)
        print("\n=== 姿态误差 (deg) ===")
        print(f"  roll : mean={re.mean():.4f}, std={re.std():.4f}, "
              f"|max|={np.abs(re).max():.4f}")
        print(f"  pitch: mean={pe.mean():.4f}, std={pe.std():.4f}, "
              f"|max|={np.abs(pe).max():.4f}")
        print(f"  yaw  : mean={ye.mean():.4f}, std={ye.std():.4f}, "
              f"|max|={np.abs(ye).max():.4f}")

    # ===== Qins 分布 =====
    print("\n=== Qins 分布 ===")
    qa = np.array(qins_arr)
    for qv in [0, 1, 2, 3]:
        n = int(np.sum(qa == qv))
        print(f"  Qins={qv}: {n} ({100 * n / total:.1f}%)")

    # ===== 时间段分析 (每 60s) =====
    print("\n=== 分时段 3D RMSE (每 120s) ===")
    for t_start in np.arange(0, times.max() - t0 + 1, 120):
        mask = (times - t0 >= t_start) & (times - t0 < t_start + 120)
        if mask.sum() > 0:
            seg3d = pos3d[mask]
            segyaw = np.array(yaw_e)[mask] if yaw_e else None
            ystr = f", yaw_err={segyaw.mean():.2f}" if segyaw is not None else ""
            print(f"  t=[{t_start:.0f},{t_start+120:.0f})s: "
                  f"n={mask.sum()}, 3D RMSE={math.sqrt((seg3d**2).mean()):.3f}{ystr}")

    # ===== 最大误差历元 =====
    print("\n=== 最大误差历元 ===")
    for name, arr in [("平面", planar), ("高程", np.abs(dU)),
                       ("3D", pos3d), ("速度", dv)]:
        idx = int(np.argmax(arr))
        wk, sow = common[idx]
        print(f"  {name} max={arr[idx]:.4f} at sow={sow:.3f} "
              f"(t+{sow-t0:.1f}s, Qins={qins_arr[idx]})")


if __name__ == "__main__":
    main()
