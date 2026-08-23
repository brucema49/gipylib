"""RTK-TC 紧组合问题诊断脚本。"""
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
from src.core.gnss.rtklib.rtkpos import relpos, zdres, selsat
from src.core.tc.tc_stream import _filter_gps_svh
from src.utility.rinex_simplifier import needs_simplification, simplify_rinex
from src.core.tc.tc_measurement import RtkTcMeas
from src.core.tc.tc_state_index import TcStateIndex
from src.core.data_types import InsState
from src.core.ins.attitude import dcm2quat

config = load_config("data/config.yaml")
gnss_cfg = config["gnss"]

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

env = RtklibEnv(gnss_cfg)
env.setup()
nav = env.init_nav()

rover_path = prepare_rinex(gnss_cfg["rover_path"], gnss_cfg)
base_path = prepare_rinex(gnss_cfg["base_path"], gnss_cfg)
rov = rn.rnx_decode(env.get_cfg())
rov.decode_obsfile(nav, rover_path, None)
rov.decode_nav(gnss_cfg["eph_path"], nav)
base = rn.rnx_decode(env.get_cfg())
base.decode_obsfile(nav, base_path, None)
if nav.rb[0] == 0:
    nav.rb = base.pos
print(f"读入 {len(rov.obslist)} 历元 rover, {len(base.obslist)} 历元 base")

# 找到初始化时刻附近的历元 (t=1553743783)
target_t = 1553743783.0
for idx, obsr in enumerate(rov.obslist):
    t = float(obsr.t.time + obsr.t.sec)
    if t >= target_t:
        break
obsr = rov.obslist[idx]
obsb = base.obslist[idx]
print(f"使用第 {idx} 历元, t={float(obsr.t.time + obsr.t.sec):.3f}")

# SPP for initial position
rs, var, dts, svh = satposs(obsr, nav)
svh_gps = _filter_gps_svh(obsr, svh)
if np.any(nav.rb):
    nav.x[0:3] = nav.rb
saved_x = nav.x.copy()
saved_P = nav.P.copy()
sol, x_spp = estpos(obsr, nav, rs[:, :3], dts, svh_gps)
nav.x[:] = saved_x
nav.P[:] = saved_P
print(f"SPP pos: {x_spp[:3]}")

# RTK for initial position
nav.x[0:6] = sol.rr[0:6]
nav.x[6:9] = 1e-6
from src.core.gnss.rtklib.rtkcmn import Sol, SOLQ_NONE
rtk_sol = Sol()
rtk_sol.t = obsr.t
relpos(nav, obsr, obsb, rtk_sol)
nav.x[:] = saved_x
nav.P[:] = saved_P
print(f"RTK stat={rtk_sol.stat}, ns={rtk_sol.ns}, pos={rtk_sol.rr[:3]}")

# 测试 RTK TC 量测
state = InsState(
    timestamp=float(obsr.t.time + obsr.t.sec),
    pos_e=rtk_sol.rr[:3].copy() if rtk_sol.stat != SOLQ_NONE else x_spp[:3].copy(),
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

si = TcStateIndex.from_config(config, "rtk")
# 设置 ambiguity 数量
from src.core.gnss.rtklib.rtkcmn import uGNSS as U
nf = int(config.get("gnss", {}).get("nf", 2))
si.set_ambiguity_count(U.MAXSAT * nf)
print(f"\n=== TcStateIndex (rtk) ===")
print(f"dim={si.dim}, pos={si.pos}, vel={si.vel}, amb_start={si.amb_start}, n_amb={si.n_amb}")

builder = RtkTcMeas(config)
x_eff = np.zeros(si.dim)
v, H, R, info = builder.build(state, obsr, nav, si, x=x_eff, obsb=obsb, P=np.eye(si.dim)*100**2)
print(f"\n=== RTK TC 量测 ===")
print(f"量测数 n={info.get('n', 0)}")
if len(v) > 0:
    print(f"v: min={v.min():.3f}, max={v.max():.3f}, mean={v.mean():.3f}, std={v.std():.3f}")
    print(f"H shape={H.shape}")
    print(f"R shape={R.shape}")
    print(f"pairs (前10): {info.get('pairs', [])[:10]}")
    print(f"ref_sats: {info.get('ref_sats', [])}")
    
    # 看看 H 的位置列
    print(f"\nH[pos] (前5行, 非零):")
    for i in range(min(5, H.shape[0])):
        h_pos = H[i, si.pos:si.pos+3]
        if np.any(h_pos != 0):
            print(f"  row {i}: {h_pos}")
    
    # 看看 H 的模糊度列
    print(f"\nH[amb] (前5行, 非零):")
    for i in range(min(5, H.shape[0])):
        h_amb = H[i, si.amb_start:si.amb_start+min(10, si.n_amb)]
        nz = np.where(h_amb != 0)[0]
        if len(nz) > 0:
            print(f"  row {i}: nonzero at {nz}, vals={h_amb[nz]}")
    
    # KF 更新模拟
    P = np.eye(si.dim) * 100.0 ** 2
    P[:3, :3] = 30.0 ** 2
    P[3:6, 3:6] = 10.0 ** 2
    # 模糊度初始方差
    for k in range(si.n_amb):
        P[si.amb_start + k, si.amb_start + k] = 30.0 ** 2
    
    innov = v - H @ np.zeros(si.dim)  # x_err = 0
    S = H @ P @ H.T + R
    print(f"\nS diag (前5): {np.diag(S)[:5]}")
    K = P @ H.T @ np.linalg.inv(S)
    delta = K @ innov
    print(f"delta_pos: {delta[si.pos:si.pos+3]}")
    print(f"|delta_pos| = {np.linalg.norm(delta[si.pos:si.pos+3]):.3f} m")
else:
    print("无量测!")
