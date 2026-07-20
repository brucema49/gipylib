"""对比 data/output/RTKLC.rslt (LLH) 与 data/rtktcgps.rslt (ECEF) 的位置/速度精度，
并对我们的姿态输出做合理性分析。

RTKLC.rslt 公共参数已与 RTK.pos 一致 (LLH 位置 + ENU sd)；
参考文件 rtktcgps.rslt 仍为 ECEF XYZ 格式，对比时把 ours LLH 转回 ECEF 求差，
再把 ECEF 差转为 ENU 差计算真正的平面/高程误差。

按 (week, sow) 匹配重叠历元（容差 0.005s）。
项目硬约束：平面 ≤ 0.5m, 高程 ≤ 1m, 速度 ≤ 0.5 m/s。
"""
import math
import sys
from pathlib import Path

import numpy as np


# ===== RSLT 字段索引 (LLH 版本) =====
# 0:week 1:sow 2:lat 3:lon 4:h 5:Q 6:Qins 7:ns
# 8:sdn 9:sde 10:sdu 11:sdne 12:sdeu 13:sdun
# 14:age 15:ratio 16:vx 17:vy 18:vz
# 19:sdvx 20:sdvy 21:sdvz 22:sdvxy 23:sdvyz 24:sdvzx
# 25:roll 26:pitch 27:yaw 28:sdroll 29:sdpitch 30:sdyaw (仅 ours)
W, SOW, LAT, LON, H = 0, 1, 2, 3, 4
Q, QINS, NS = 5, 6, 7
AGE, RATIO = 14, 15
VX, VY, VZ = 16, 17, 18
ROLL, PITCH, YAW = 25, 26, 27

# ECEF 字段索引 (ref 文件 25 字段, ECEF XYZ)
X_ECEF, Y_ECEF, Z_ECEF = 2, 3, 4


def load_ours(path):
    """加载 LLH 格式 RSLT (ours)。返回 dict: (week, sow_ms) -> rec。"""
    data = {}
    with open(path, "r") as f:
        for line in f:
            if line.startswith("%") or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 17:
                continue
            rec = {
                "lat": float(parts[LAT]), "lon": float(parts[LON]),
                "h": float(parts[H]),
                "vx": float(parts[VX]), "vy": float(parts[VY]), "vz": float(parts[VZ]),
                "q": int(parts[Q]), "qins": int(parts[QINS]), "ns": int(parts[NS]),
                "sow": float(parts[SOW]),
            }
            if len(parts) >= 28:
                rec["roll"] = float(parts[ROLL])
                rec["pitch"] = float(parts[PITCH])
                rec["yaw"] = float(parts[YAW])
            week = int(parts[W])
            key = (week, round(float(parts[SOW]), 3))
            data[key] = rec
    return data


def load_ref(path):
    """加载 ECEF 格式 RSLT (ref)。返回 dict: (week, sow_ms) -> rec。"""
    data = {}
    with open(path, "r") as f:
        for line in f:
            if line.startswith("%") or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 17:
                continue
            rec = {
                "x": float(parts[X_ECEF]), "y": float(parts[Y_ECEF]),
                "z": float(parts[Z_ECEF]),
                "vx": float(parts[VX]), "vy": float(parts[VY]), "vz": float(parts[VZ]),
                "q": int(parts[Q]), "qins": int(parts[QINS]), "ns": int(parts[NS]),
                "sow": float(parts[SOW]),
            }
            week = int(parts[W])
            key = (week, round(float(parts[SOW]), 3))
            data[key] = rec
    return data


def llh2ecef(lat_deg, lon_deg, h):
    """LLH (deg, deg, m) → ECEF (x, y, z). WGS84."""
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
    """ECEF (x,y,z) → (lat, lon) in radians. WGS84."""
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


def ecef_vel_to_enu(vx, vy, vz, lat, lon):
    """ECEF 速度 → ENU 速度。lat/lon in radians。"""
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    vE = -so * vx + co * vy
    vN = -sl * co * vx - sl * so * vy + cl * vz
    vU = cl * co * vx + cl * so * vy + sl * vz
    return vE, vN, vU


def ecef_diff_to_enu(dx, dy, dz, lat, lon):
    """ECEF 差向量 → ENU 差向量。lat/lon in radians。"""
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    dE = -so * dx + co * dy
    dN = -sl * co * dx - sl * so * dy + cl * dz
    dU = cl * co * dx + cl * so * dy + sl * dz
    return dE, dN, dU


def wrap_angle_deg(a):
    """将角度差归一化到 [-180, 180]。"""
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def main():
    ours_path = Path("data/output/RTKLC.rslt")
    ref_path = Path("data/rtktcgps.rslt")

    print(f"加载 ours (LLH): {ours_path}")
    ours = load_ours(ours_path)
    print(f"  历元数: {len(ours)}")
    print(f"加载 ref (ECEF): {ref_path}")
    ref = load_ref(ref_path)
    print(f"  历元数: {len(ref)}")

    common_keys = sorted(set(ours.keys()) & set(ref.keys()))
    total = len(common_keys)
    print(f"重叠历元: {total}")
    if not common_keys:
        print("ERROR: 无重叠历元")
        sys.exit(1)

    # ===== 收集 (ours LLH → ECEF, 与 ref ECEF 求差, 再转 ENU) =====
    dE_arr, dN_arr, dU_arr = [], [], []
    dvx, dvy, dvz = [], [], []
    qins_ours, qins_ref = [], []
    times = []
    for k in common_keys:
        o, r = ours[k], ref[k]
        # ours LLH → ECEF
        ox, oy, oz = llh2ecef(o["lat"], o["lon"], o["h"])
        # ECEF 差
        dx = ox - r["x"]
        dy = oy - r["y"]
        dz = oz - r["z"]
        # 用 ref 位置的 lat/lon 把 ECEF 差转 ENU
        lat_r, lon_r = ecef2llh(r["x"], r["y"], r["z"])
        dE, dN, dU = ecef_diff_to_enu(dx, dy, dz, lat_r, lon_r)
        dE_arr.append(dE)
        dN_arr.append(dN)
        dU_arr.append(dU)
        # 速度 ECEF 差 (ours vel 仍是 ECEF)
        dvx.append(o["vx"] - r["vx"])
        dvy.append(o["vy"] - r["vy"])
        dvz.append(o["vz"] - r["vz"])
        qins_ours.append(o["qins"])
        qins_ref.append(r["qins"])
        times.append(r["sow"])

    dE = np.array(dE_arr); dN = np.array(dN_arr); dU = np.array(dU_arr)
    dvx = np.array(dvx); dvy = np.array(dvy); dvz = np.array(dvz)
    planar = np.sqrt(dE ** 2 + dN ** 2)  # 真正的水平面误差
    dv_abs = np.sqrt(dvx ** 2 + dvy ** 2 + dvz ** 2)
    times = np.array(times)
    t0 = times.min()

    # ===== 位置误差统计 (ENU) =====
    print("\n=== 位置误差统计 (m, ENU) ===")
    print(f"  平面 (sqrt(dE²+dN²)):")
    print(f"    max  = {planar.max():.4f}")
    print(f"    mean = {planar.mean():.4f}")
    print(f"    p95  = {np.percentile(planar, 95):.4f}")
    print(f"    p99  = {np.percentile(planar, 99):.4f}")
    print(f"  高程 (|dU|):")
    print(f"    max  = {np.abs(dU).max():.4f}")
    print(f"    mean = {np.abs(dU).mean():.4f}")
    print(f"    p95  = {np.percentile(np.abs(dU), 95):.4f}")
    print(f"    p99  = {np.percentile(np.abs(dU), 99):.4f}")
    print(f"  各分量:")
    print(f"    dE: max={np.abs(dE).max():.4f}, mean={np.abs(dE).mean():.4f}")
    print(f"    dN: max={np.abs(dN).max():.4f}, mean={np.abs(dN).mean():.4f}")
    print(f"    dU: max={np.abs(dU).max():.4f}, mean={np.abs(dU).mean():.4f}")

    # ===== 速度误差统计 =====
    print("\n=== 速度误差统计 (m/s, ECEF) ===")
    print(f"  |dv| = sqrt(dvx²+dvy²+dvz²):")
    print(f"    max  = {dv_abs.max():.4f}")
    print(f"    mean = {dv_abs.mean():.4f}")
    print(f"    p95  = {np.percentile(dv_abs, 95):.4f}")
    print(f"    p99  = {np.percentile(dv_abs, 99):.4f}")
    print(f"  各分量:")
    print(f"    dvx: max={np.abs(dvx).max():.4f}, mean={np.abs(dvx).mean():.4f}")
    print(f"    dvy: max={np.abs(dvy).max():.4f}, mean={np.abs(dvy).mean():.4f}")
    print(f"    dvz: max={np.abs(dvz).max():.4f}, mean={np.abs(dvz).mean():.4f}")

    # ===== 硬约束检查 =====
    planar_thr, elev_thr, vel_thr = 0.5, 1.0, 0.5
    planar_fail = int(np.sum(planar > planar_thr))
    elev_fail = int(np.sum(np.abs(dU) > elev_thr))
    vel_fail = int(np.sum(dv_abs > vel_thr))

    print("\n=== 硬约束检查 ===")
    print(f"  平面 ≤ {planar_thr}m: {planar_fail}/{total} 失败 "
          f"({100 * planar_fail / total:.2f}%)")
    print(f"  高程 ≤ {elev_thr}m: {elev_fail}/{total} 失败 "
          f"({100 * elev_fail / total:.2f}%)")
    print(f"  速度 ≤ {vel_thr}m/s: {vel_fail}/{total} 失败 "
          f"({100 * vel_fail / total:.2f}%)")
    total_fail = planar_fail + elev_fail + vel_fail
    print(f"  综合: {total_fail}/{total} 失败 "
          f"({100 * total_fail / total:.2f}%)")

    # ===== Qins 分布 =====
    print("\n=== Qins 分布 ===")
    qins_ours_arr = np.array(qins_ours)
    qins_ref_arr = np.array(qins_ref)
    for qv in [0, 1, 2, 3]:
        n_o = int(np.sum(qins_ours_arr == qv))
        n_r = int(np.sum(qins_ref_arr == qv))
        print(f"  Qins={qv}: ours={n_o} ({100 * n_o / total:.1f}%), "
              f"ref={n_r} ({100 * n_r / total:.1f}%)")

    # ===== 姿态合理性分析 (仅 ours) =====
    print("\n" + "=" * 60)
    print("=== 姿态合理性分析 (仅 ours, ref 无姿态列) ===")
    print("=" * 60)

    att_keys = sorted(ours.keys())
    roll_arr = np.array([ours[k]["roll"] for k in att_keys if "roll" in ours[k]])
    pitch_arr = np.array([ours[k]["pitch"] for k in att_keys if "pitch" in ours[k]])
    yaw_arr = np.array([ours[k]["yaw"] for k in att_keys if "yaw" in ours[k]])
    att_times = np.array([ours[k]["sow"] for k in att_keys if "roll" in ours[k]])

    def stat_arr(name, arr):
        print(f"  {name} (deg): min={arr.min():.3f}, max={arr.max():.3f}, "
              f"mean={arr.mean():.3f}, std={arr.std():.3f}")

    print("\n--- 范围统计 ---")
    stat_arr("roll ", roll_arr)
    stat_arr("pitch", pitch_arr)
    stat_arr("yaw  ", yaw_arr)

    # 变化率 (deg/s) — yaw 用 wrap 处理 ±180 跳变
    print("\n--- 变化率统计 (deg/s) ---")
    dt_arr = np.diff(att_times)
    valid = dt_arr > 1e-6
    def rate(arr, wrap=False):
        d = np.diff(arr)
        if wrap:
            d = np.array([wrap_angle_deg(x) for x in d])
        return d[valid] / dt_arr[valid]
    roll_rate = rate(roll_arr)
    pitch_rate = rate(pitch_arr)
    yaw_rate = rate(yaw_arr, wrap=True)
    def stat_rate(name, arr):
        abs_arr = np.abs(arr)
        print(f"  {name}: |rate| max={abs_arr.max():.3f}, "
              f"mean={abs_arr.mean():.3f}, p95={np.percentile(abs_arr, 95):.3f}, "
              f"p99={np.percentile(abs_arr, 99):.3f}")
    stat_rate("roll ", roll_rate)
    stat_rate("pitch", pitch_rate)
    stat_rate("yaw  ", yaw_rate)

    # 静态段稳定性 (GNSS 速度 < 0.3 m/s 历元)
    print("\n--- 静态段稳定性 (GNSS 速度 < 0.3 m/s 历元) ---")
    static_keys = [k for k in att_keys
                   if k in ref and
                   math.sqrt(ref[k]["vx"] ** 2 + ref[k]["vy"] ** 2 + ref[k]["vz"] ** 2) < 0.3]
    if len(static_keys) > 10:
        roll_s = np.array([ours[k]["roll"] for k in static_keys])
        pitch_s = np.array([ours[k]["pitch"] for k in static_keys])
        yaw_s = np.array([ours[k]["yaw"] for k in static_keys])
        print(f"  静态历元数: {len(static_keys)}")
        print(f"  roll  std: {roll_s.std():.4f} deg")
        print(f"  pitch std: {pitch_s.std():.4f} deg")
        print(f"  yaw   std: {yaw_s.std():.4f} deg")
    else:
        print(f"  静态历元数不足: {len(static_keys)}")

    # 动态段 yaw vs 速度方向一致性
    print("\n--- 动态段 yaw vs GNSS 速度方位角一致性 ---")
    print("  (yaw 与 azimuth_v=atan2(v_E, v_N) 差异, 速度 > 1 m/s)")
    yaw_diffs = []
    speeds_dyn = []
    for k in att_keys:
        if k not in ref or "roll" not in ours[k]:
            continue
        o, r = ours[k], ref[k]
        speed = math.sqrt(r["vx"] ** 2 + r["vy"] ** 2 + r["vz"] ** 2)
        if speed < 1.0:
            continue
        lat, lon = ecef2llh(r["x"], r["y"], r["z"])
        vE, vN, _ = ecef_vel_to_enu(r["vx"], r["vy"], r["vz"], lat, lon)
        azimuth_v = math.degrees(math.atan2(vE, vN))  # 0=N, 90=E
        diff = wrap_angle_deg(o["yaw"] - azimuth_v)
        yaw_diffs.append(diff)
        speeds_dyn.append(speed)
    if yaw_diffs:
        yaw_diffs = np.array(yaw_diffs)
        speeds_dyn = np.array(speeds_dyn)
        abs_d = np.abs(yaw_diffs)
        print(f"  动态历元数: {len(yaw_diffs)}")
        print(f"  速度范围: {speeds_dyn.min():.3f} ~ {speeds_dyn.max():.3f} m/s")
        print(f"  yaw - azimuth_v (deg):")
        print(f"    mean = {yaw_diffs.mean():.3f}")
        print(f"    |diff| max  = {abs_d.max():.3f}")
        print(f"    |diff| mean = {abs_d.mean():.3f}")
        print(f"    |diff| p95  = {np.percentile(abs_d, 95):.3f}")
        print(f"    |diff| p99  = {np.percentile(abs_d, 99):.3f}")
    else:
        print("  无动态历元")

    # ===== 排除前 180s 收敛期后的稳态分析 =====
    print("\n" + "=" * 60)
    print("=== 排除前 180s 收敛期后的稳态统计 ===")
    print("=" * 60)
    steady_mask = times >= (t0 + 180.0)
    if steady_mask.sum() > 0:
        p_s = planar[steady_mask]
        u_s = np.abs(dU[steady_mask])
        v_s = dv_abs[steady_mask]
        n_s = int(steady_mask.sum())
        print(f"  稳态历元: {n_s}")
        print(f"  位置平面: max={p_s.max():.4f}, mean={p_s.mean():.4f}, "
              f"p99={np.percentile(p_s, 99):.4f}")
        print(f"  位置高程: max={u_s.max():.4f}, mean={u_s.mean():.4f}, "
              f"p99={np.percentile(u_s, 99):.4f}")
        print(f"  速度    : max={v_s.max():.4f}, mean={v_s.mean():.4f}, "
              f"p99={np.percentile(v_s, 99):.4f}")
        p_sf = int(np.sum(p_s > planar_thr))
        e_sf = int(np.sum(u_s > elev_thr))
        v_sf = int(np.sum(v_s > vel_thr))
        print(f"  平面失败: {p_sf}/{n_s} ({100 * p_sf / n_s:.2f}%)")
        print(f"  高程失败: {e_sf}/{n_s} ({100 * e_sf / n_s:.2f}%)")
        print(f"  速度失败: {v_sf}/{n_s} ({100 * v_sf / n_s:.2f}%)")
        pass_rate = 100 * (1 - (p_sf + e_sf + v_sf) / n_s)
        print(f"  通过率: {pass_rate:.2f}%")

    # ===== 最大误差历元 =====
    print("\n=== 最大误差历元 ===")
    for name, arr, thr in [("平面", planar, planar_thr),
                            ("高程", np.abs(dU), elev_thr),
                            ("速度", dv_abs, vel_thr)]:
        idx = int(np.argmax(arr))
        wk, sow = common_keys[idx]
        print(f"  {name} max={arr[idx]:.4f} at week={wk} sow={sow:.3f} "
              f"(t+{sow - t0:.1f}s, Qins_ours={qins_ours[idx]}, "
              f"Qins_ref={qins_ref[idx]})")

    # ===== 结论 =====
    print("\n=== 结论 ===")
    if total_fail == 0:
        print(f"  PASS: 所有 {total} 个历元满足位置/速度硬约束")
    else:
        pass_rate = 100 * (1 - total_fail / total)
        print(f"  总通过率: {pass_rate:.2f}% ({total - total_fail}/{total})")
        if pass_rate >= 99.0:
            print(f"  PASS (≥99% 通过率)")
        else:
            print(f"  需关注: 通过率 {pass_rate:.2f}%")


if __name__ == "__main__":
    main()
