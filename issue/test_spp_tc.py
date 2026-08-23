"""SPP-TC 紧组合问题诊断脚本。"""
import sys
import logging
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
logging.basicConfig(level=logging.WARNING, format='%(levelname)s %(name)s: %(message)s')

import tempfile
from src.utility.config_loader import load_config
from src.core.gnss.rtklib_config_adapter import RtklibEnv
from src.core.gnss.rtklib import rinex as rn
from src.core.gnss.rtklib.ephemeris import satposs
from src.core.gnss.rtklib.pntpos import estpos
from src.core.gnss.rtklib.rtkcmn import sat2prn, uGNSS, ecef2pos
from src.core.tc.tc_stream import _filter_gps_svh
from src.utility.rinex_simplifier import needs_simplification, simplify_rinex

config = load_config("data/spp-ins-tc.yaml")
gnss_cfg = config["gnss"]

# 简化 RINEX
def prepare_rinex(path, gnss_cfg):
    if not needs_simplification(path):
        return path
    suffix = Path(path).suffix
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=suffix, delete=False, encoding="utf-8")
    tmp.close()
    sys_map = {"GPS": "G", "BDS": "C", "GAL": "E", "GLO": "R", "QZS": "J", "SBS": "S"}
    freq_ix0 = gnss_cfg.get("freq_ix0", {})
    freq_ix1 = gnss_cfg.get("freq_ix1", {})
    priority = {}
    for sys_name, sys_char in sys_map.items():
        freqs = []
        if sys_name in freq_ix0: freqs.append(int(freq_ix0[sys_name]))
        if sys_name in freq_ix1: freqs.append(int(freq_ix1[sys_name]))
        if freqs: priority[sys_char] = freqs
    simplify_rinex(path, tmp.name, freq_priority=priority)
    return tmp.name

# 初始化 rtklib 环境
env = RtklibEnv(gnss_cfg)
env.setup()
nav = env.init_nav()

# 读取 rover 观测
rover_path = prepare_rinex(gnss_cfg["rover_path"], gnss_cfg)
rov = rn.rnx_decode(env.get_cfg())
rov.decode_obsfile(nav, rover_path, None)
rov.decode_nav(gnss_cfg["eph_path"], nav)
print(f"读入 {len(rov.obslist)} 历元 rover 观测")

# 找到初始化时刻附近的历元 (t=1553743783)
target_t = 1553743783.0
for idx, obsr in enumerate(rov.obslist):
    t = float(obsr.t.time + obsr.t.sec)
    if t >= target_t:
        break
print(f"使用第 {idx} 历元, t={float(rov.obslist[idx].t.time + rov.obslist[idx].t.sec):.3f}")

obsr = rov.obslist[idx]
# SPP 解算
rs, var, dts, svh = satposs(obsr, nav)
svh_gps = _filter_gps_svh(obsr, svh)
print(f"\n=== SPP 解算 (GPS-only) ===")
print(f"卫星数: {len(obsr.sat)}, svh_gps 非零数: {np.sum(svh_gps != 0)}")
if np.any(nav.rb):
    nav.x[0:3] = nav.rb
saved_x = nav.x.copy()
saved_P = nav.P.copy()
sol, x_spp = estpos(obsr, nav, rs[:, :3], dts, svh_gps)
nav.x[:] = saved_x
nav.P[:] = saved_P
print(f"sol.stat={sol.stat}, ns={sol.ns}")
print(f"SPP pos: {x_spp[:3]}")
print(f"SPP clk: {x_spp[3:6]}")

# 看看所有卫星
print(f"\n=== 卫星列表 ===")
for i in range(len(obsr.sat)):
    sys, prn = sat2prn(obsr.sat[i])
    sys_name = {1: "GPS", 2: "GLO", 3: "GAL", 4: "BDS", 5: "QZS", 6: "SBS"}.get(sys, str(sys))
    r, e = None, None
    from src.core.gnss.rtklib.rtkcmn import geodist, satazel
    try:
        r, e = geodist(rs[i, :3], x_spp[:3])
        pos = ecef2pos(x_spp[:3])
        az, el = satazel(pos, e)
        print(f"  sat={obsr.sat[i]:3d} ({sys_name}{prn:02d}) svh={svh[i]} P={obsr.P[i,0]:.1f} el={np.degrees(el):.1f}° e={e}")
    except Exception as ex:
        print(f"  sat={obsr.sat[i]:3d} ({sys_name}{prn:02d}) svh={svh[i]} P={obsr.P[i,0]:.1f} err={ex}")

# 现在测试 TC 量测构造
from src.core.tc.tc_measurement import SppTcMeas
from src.core.tc.tc_state_index import TcStateIndex
from src.core.data_types import InsState

# 构造一个 InsState
from src.core.ins.attitude import dcm2quat
state = InsState(
    timestamp=float(obsr.t.time + obsr.t.sec),
    pos_e=x_spp[:3].copy(),
    vel_e=np.zeros(3, dtype=np.float64),
    C_b_e=np.eye(3, dtype=np.float64),
    q_b_e=dcm2quat(np.eye(3)),
    att_rpy=np.zeros(3, dtype=np.float64),
    gyro_bias=np.zeros(3, dtype=np.float64),
    accel_bias=np.zeros(3, dtype=np.float64),
    imu_angle=np.zeros(2, dtype=np.float64),
    imu_leverarm=np.zeros(3, dtype=np.float64),
    leverarm=np.zeros(3, dtype=np.float64),
)

si = TcStateIndex.from_config(config, "spp")
print(f"\n=== TcStateIndex ===")
print(f"dim={si.dim}, pos={si.pos}, vel={si.vel}, att={si.att}, clk_bias={si.clk_bias}")

# 构造 effective_x (初始 stored=SPP clk, ε=0)
x_eff = np.zeros(si.dim)
x_eff[si.clk_bias + 0] = x_spp[3]   # GPS
x_eff[si.clk_bias + 1] = x_spp[4]   # GLO/BDS
x_eff[si.clk_bias + 2] = x_spp[5]   # GAL

builder = SppTcMeas(config)
v, H, R, info = builder.build(state, obsr, nav, si, x=x_eff)
print(f"\n=== SPP TC 量测 ===")
print(f"量测数 n={info.get('n', 0)}")
print(f"v (residuals): shape={v.shape}")
print(f"  min={v.min():.3f}, max={v.max():.3f}, mean={v.mean():.3f}, std={v.std():.3f}")
print(f"H shape={H.shape}")
print(f"R shape={R.shape}, diag={np.diag(R)[:5]}")

# 看看 H 矩阵的位置列
print(f"\nH[pos] (前5行):")
print(H[:5, si.pos:si.pos+3])
print(f"\nH[clk_bias] (前5行):")
print(H[:5, si.clk_bias:si.clk_bias+3])

# 卫星列表
print(f"\n使用的卫星: {info.get('sats', [])}")

# 模拟 KF 更新
print(f"\n=== KF 更新模拟 ===")
P = np.eye(si.dim) * 100.0 ** 2
P[:3, :3] = 30.0 ** 2
P[3:6, 3:6] = 10.0 ** 2
for k in range(3):
    P[si.clk_bias + k, si.clk_bias + k] = 10.0 ** 2

# 手动 Joseph 更新
x_err = np.zeros(si.dim)
innov = v - H @ x_err
print(f"innov (== v): min={innov.min():.3f}, max={innov.max():.3f}, mean={innov.mean():.3f}")
S = H @ P @ H.T + R
print(f"S diag: {np.diag(S)[:5]}")
K = P @ H.T @ np.linalg.inv(S)
print(f"K shape={K.shape}")
print(f"K[pos, :5] (前5列):")
print(K[si.pos:si.pos+3, :5])
print(f"x_err update = K @ innov:")
delta = K @ innov
print(f"  delta_pos: {delta[si.pos:si.pos+3]}")
print(f"  delta_clk: {delta[si.clk_bias:si.clk_bias+3]}")
print(f"  |delta_pos| = {np.linalg.norm(delta[si.pos:si.pos+3]):.3f} m")

# 如果 delta_pos 很大, 说明 R 太小或 P 太大
print(f"\n=== 诊断 ===")
print(f"P[pos,pos] diag: {np.diag(P)[si.pos:si.pos+3]}")
print(f"R diag: {np.diag(R)[:5]}")
print(f"P/R ratio (pos): {30**2 / 9:.1f} (高 -> K 大, 跟随测量)")
