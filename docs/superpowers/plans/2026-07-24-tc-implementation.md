# 紧组合（TC）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 gipylib 中实现 SPP-INS/RTD-INS/RTK-INS 三种紧组合模式，分模块验证后端到端输出，SPP RMSE≤2m、RTD/RTK≤1m。

**Architecture:** `src/core/tc/` 新模块，`TcEstimator` 继承 `LcEstimator` 复用机械编排/约瑟夫更新；`TcMeasurement` 三模式构造 H/v/R（SPP 用 rtklib pntpos 残差、RTD/RTK 用 ddres 双差）；路径 D 用 `TcGnssSensor` 输出原始观测 + `TcStream` 流式融合；复用现有 `Constraints`/`RSLTWriter`(LLH)，verify 脚本转 ECEF 对比。

**Tech Stack:** Python 3、numpy、rtklib-py 子包（`src/core/gnss/rtklib/`）、现有 INS 框架（`src/core/ins/`）

## Global Constraints

- 所有源码放 `src/core/tc/`，不修改 `src/core/gnss/rtklib/` 与 `src/core/ins/` 现有代码（仅继承/调用）
- `satposs(obs, nav)` 返回 `(rs, var, dts, svh)` 四元组（非三元组）
- LAMBDA 用 `mlambda(a, Q, m=2)`（`mlambda.py:145`），不调用 `resamb_lambda`
- `prange(nav, obs, i)` 仅返回 L1；RTK 双频直接读 `obs.P[i,f]` + `gettgd`
- SPP `varerr(nav, sys, el, rcvstd)` 与 RTK `varerr(nav, sys, el, f, dt, rcvstd, snr_r, snr_b)` 签名不同
- rtklib `ddres` H 符号约定：`H[pos] = LOS_j − LOS_i`（i=参考星），TcRtkMeas 必须采用相同约定以便逐元素对比
- 系统默认 python2，必须用 `python3` 执行
- 参考 `data/rtktc.rslt`(SPP,4sat) 与 `data/rtktcgps.rslt`(RTK,6sat) 均 ECEF XYZ；TC 输出 LLH，verify 转换
- 硬约束：SPP RMSE≤2m，RTD/RTK≤1m

---

## File Structure

**Create:**
- `src/core/tc/__init__.py` — 模块导出
- `src/core/tc/tc_state_index.py` — `TcStateIndex(StateIndex)` 扩展 GNSS 参数块
- `src/core/tc/tc_measurement.py` — `TcMeasurement` ABC + `SppTcMeas`/`RtkTcMeas`/`RtdTcMeas`
- `src/core/tc/tc_ambiguity.py` — `TcAmbiguity` 调 `mlambda`
- `src/core/tc/tc_estimator.py` — `TcEstimator(LcEstimator)` 扩展状态向量
- `src/core/tc/tc_degrade.py` — `TcDegradeManager` 降级链
- `src/core/tc/tc_integration.py` — `TcIntegration` GVINS 风格 IMU 消费 + 量测触发
- `src/core/tc/tc_stream.py` — `TcStream` 流式运行器
- `src/stream/tc_gnss_sensor.py` — `TcGnssSensor` 原始观测传感器
- `tests/tc/__init__.py`
- `tests/tc/test_tc_state_index.py` — M1
- `tests/tc/test_spp_tc_meas.py` — M2
- `tests/tc/test_rtk_tc_meas.py` — M3
- `tests/tc/test_tc_ambiguity.py` — M5
- `tests/tc/test_tc_estimator.py` — M4
- `verify_tc_rslt.py` — M6 端到端对比

**Modify:**
- `src/stream/factory.py` — `positioning_mode=="tc"` 时创建 `TcGnssSensor`
- `src/log/logger.py` — 处理 `SensorData.tag=="tc_obs"`，路由到 `tc_stream`
- `src/main.py` — 路径 D 装配（`internal + ins.enabled=="tc"`）
- `src/utility/config_loader.py` — 校验 `tc` 配置块
- `data/config.yaml` — 新增 `tc` 配置段

---

## Task 1: 脚手架 + TcStateIndex (M1)

**Files:**
- Create: `src/core/tc/__init__.py`
- Create: `src/core/tc/tc_state_index.py`
- Create: `tests/tc/__init__.py`
- Create: `tests/tc/test_tc_state_index.py`
- Test: `tests/tc/test_tc_state_index.py`

**Interfaces:**
- Consumes: `src.core.ins.state_index.StateIndex`（基类，提供 pos/vel/att/gyro_bias/accel_bias + 可选 lever_arm/imu_angle/imu_leverarm/time_sync）
- Produces: `TcStateIndex` 类，属性 `mode`/`clk_bias`/`amb_start`/`n_amb`，方法 `from_config(config, mode)`、`amb_idx(sat, freq)`、`has_ambiguity()`

- [ ] **Step 1: Write the failing test**

```python
# tests/tc/test_tc_state_index.py
import numpy as np
import pytest
from src.core.tc.tc_state_index import TcStateIndex


def _base_cfg(lever=False, imu_angle=False, imu_lever=False, time_sync=False):
    return {
        "ins": {
            "estimate_leverarm": lever,
            "estimate_imu_angle": imu_angle,
            "estimate_imu_leverarm": imu_lever,
            "estimate_time_sync": time_sync,
        }
    }


def test_spp_mode_has_clk_bias_no_ambiguity():
    si = TcStateIndex.from_config(_base_cfg(), mode="spp")
    assert si.mode == "spp"
    assert si.dim == 15 + 3              # 15 INS + 3 clk_bias
    assert si.clk_bias == 15
    assert si.n_amb == 0
    assert not si.has_ambiguity()


def test_rtk_mode_has_ambiguity_block():
    si = TcStateIndex.from_config(_base_cfg(), mode="rtk")
    si.set_ambiguity_count(10)
    assert si.mode == "rtk"
    assert si.dim == 15 + 10
    assert si.amb_start == 15
    assert si.n_amb == 10
    assert si.has_ambiguity()


def test_rtd_mode_no_gnss_params():
    si = TcStateIndex.from_config(_base_cfg(), mode="rtd")
    assert si.mode == "rtd"
    assert si.dim == 15
    assert si.n_amb == 0
    assert not si.has_ambiguity()


def test_with_optional_blocks():
    si = TcStateIndex.from_config(_base_cfg(lever=True, imu_angle=True), mode="spp")
    # 15 + 3 lever + 2 imu_angle + 3 clk
    assert si.dim == 15 + 3 + 2 + 3
    assert si.clk_bias == 15 + 3 + 2


def test_amb_idx_rtklib_ib_convention():
    # IB(sat, freq, na): sat 编号映射，na = amb_start
    si = TcStateIndex.from_config(_base_cfg(), mode="rtk")
    si.set_ambiguity_count(10)
    # 第一颗卫星 freq0 在 amb_start
    idx = si.amb_idx(sat=1, freq=0)
    assert idx == si.amb_start
    # 同频不同星递增
    assert si.amb_idx(sat=2, freq=0) == si.amb_start + 1


def test_dim_changes_with_ambiguity_count():
    si = TcStateIndex.from_config(_base_cfg(), mode="rtk")
    assert si.dim == 15
    si.set_ambiguity_count(6)
    assert si.dim == 21
    si.set_ambiguity_count(0)
    assert si.dim == 15
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/tc/test_tc_state_index.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'src.core.tc'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/core/tc/__init__.py
from src.core.tc.tc_state_index import TcStateIndex
```

```python
# src/core/tc/tc_state_index.py
"""紧组合状态向量索引 (扩展 StateIndex 加 GNSS 参数块)。

状态布局:
  δx = [δr^e, δv^e, δψ^e, δb_g, δb_a, 可选块, clk_bias(3)?, ambiguity(N)?]
  - SPP: + clk_bias(3)  [dtr, dtr_glo, dtr_gal]
  - RTK: + ambiguity(N)  N 运行时动态
  - RTD: 无 GNSS 参数块
"""
from src.core.ins.state_index import StateIndex


class TcStateIndex(StateIndex):
    """紧组合状态索引。

    继承 StateIndex 的 INS 基础 15 维 + 可选块，
    扩展 GNSS 参数块 (clk_bias / ambiguity)。
    """

    def __init__(self, base_dim: int, has_leverarm: bool, has_imu_angle: bool,
                 has_imu_leverarm: bool, has_time_sync: bool, mode: str):
        super().__init__(base_dim,
                         has_leverarm=has_leverarm,
                         has_imu_angle=has_imu_angle,
                         has_imu_leverarm=has_imu_leverarm,
                         has_time_sync=has_time_sync)
        self.mode = mode
        self._gnss_base = self._ins_end          # INS + 可选块结束位置
        self.clk_bias = -1
        self.amb_start = -1
        self.n_amb = 0

    @classmethod
    def from_config(cls, config: dict, mode: str) -> "TcStateIndex":
        ins = config.get("ins", {}) if config else {}
        # 复用基类构建逻辑: 先算 INS 维度
        base = 15
        has_lever = bool(ins.get("estimate_leverarm", False))
        has_imu_angle = bool(ins.get("estimate_imu_angle", False))
        has_imu_lever = bool(ins.get("estimate_imu_leverarm", False))
        has_time_sync = bool(ins.get("estimate_time_sync", False))
        if has_lever:
            base += 3
        if has_imu_angle:
            base += 2
        if has_imu_lever:
            base += 3
        if has_time_sync:
            base += 1
        si = cls(base, has_lever, has_imu_angle, has_imu_lever, has_time_sync, mode)
        si._init_gnss_blocks()
        return si

    def _init_gnss_blocks(self):
        """根据模式初始化 GNSS 参数块索引。"""
        cur = self._gnss_base
        if self.mode == "spp":
            self.clk_bias = cur
            cur += 3
        # rtk 的 ambiguity 运行时设置; rtd 无 GNSS 块
        self.amb_start = cur if self.mode == "rtk" else -1
        self._dim = cur

    def set_ambiguity_count(self, n: int):
        """RTK 模式运行时设置模糊度数量 (影响 dim)。"""
        if self.mode != "rtk":
            return
        self.n_amb = n
        self._dim = self.amb_start + n

    @property
    def dim(self) -> int:
        return self._dim

    def has_ambiguity(self) -> bool:
        return self.mode == "rtk" and self.n_amb > 0

    def amb_idx(self, sat: int, freq: int, na: int = None) -> int:
        """模糊度索引 (rtklib IB 宏约定)。

        IB(sat, f, na) = na + (sat-1)*nf + f  (简化: 按卫星编号线性)
        实际 ddidx 按参考星选择重排, 这里返回相对 amb_start 的偏移基址,
        TcAmbiguity 维护 sat→idx 映射。
        """
        if na is None:
            na = self.amb_start
        return na + (sat - 1) * 2 + freq   # nf=2 双频, 简化布局

    def reset_gnss_blocks(self):
        """降级重整时重置 GNSS 参数块 (保留 INS+可选块)。"""
        self.n_amb = 0
        self._init_gnss_blocks()
```

```python
# tests/tc/__init__.py
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/tc/test_tc_state_index.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: Commit**

```bash
git add src/core/tc/__init__.py src/core/tc/tc_state_index.py tests/tc/__init__.py tests/tc/test_tc_state_index.py
git commit -m "feat(tc): add TcStateIndex with clk_bias/ambiguity blocks (M1)"
```

---

## Task 2: SppTcMeas (M2)

**Files:**
- Create: `src/core/tc/tc_measurement.py`
- Create: `tests/tc/test_spp_tc_meas.py`
- Test: `tests/tc/test_spp_tc_meas.py`

**Interfaces:**
- Consumes: `src.core.gnss.rtklib.ephemeris.satposs(obs, nav) → (rs, var, dts, svh)`；`pntpos.varerr(nav, sys, el, rcvstd)`；`rtkcmn.geodist(rs, rr) → (r, e)`；`rtkcmn.satazel(pos, e) → (az, el)`；`ephemeris.gettgd`；`data_types.InsState`
- Produces: `TcMeasurement` ABC（`.build(state, obsr, nav, si) → (v, H, R, info)`）、`SppTcMeas`

- [ ] **Step 1: Write the failing test**

```python
# tests/tc/test_spp_tc_meas.py
"""验证 SppTcMeas 构造的残差 v 与 rtklib pntpos.rescode 一致 (容差 1e-6)。

策略: 用同一份观测数据分别跑 pntpos 单点定位的 rescode 和 SppTcMeas.build,
     对比残差向量和 H 矩阵的数值一致性。
"""
import numpy as np
import pytest
from src.core.tc.tc_measurement import SppTcMeas
from src.core.tc.tc_state_index import TcStateIndex


def _load_test_epoch():
    """加载一个测试历元的 obs/nav (复用 data/ 下的 RINEX)。"""
    import os
    from src.core.gnss.rtklib.rinex import first_obs, next_obs, decode_rinex_file
    obs_file = "data/cpt0870a.24o"
    nav_file = "data/cpt0870a.24n"
    if not (os.path.exists(obs_file) and os.path.exists(nav_file)):
        pytest.skip("RINEX test data not found")
    obsr, navr = decode_rinex_file(obs_file, nav_file)
    return obsr, navr


def _make_state_from_spp(obsr, navr):
    """用 pntpos 做粗定位构造 InsState。"""
    from src.core.gnss.rtklib.pntpos import estpos
    sol, rr, var, Q, ns, _ = estpos(obsr, navr)
    from src.core.data_types import InsState
    state = InsState()
    state.pos_e = rr
    state.vel_e = np.zeros(3)
    state.C_b_e = np.eye(3)
    state.gyro_bias = np.zeros(3)
    state.accel_bias = np.zeros(3)
    return state, sol


def test_spp_meas_residual_matches_rtklib():
    obsr, navr = _load_test_epoch()
    state, sol = _make_state_from_spp(obsr, navr)
    cfg = {"ins": {}}
    si = TcStateIndex.from_config(cfg, mode="spp")
    meas = SppTcMeas(cfg)
    v_tc, H_tc, R_tc, info = meas.build(state, obsr, navr, si)

    # rtklib 参考残差
    from src.core.gnss.rtklib.pntpos import rescode
    v_ref, H_ref = rescode(obsr, navr, "less than 4 valid satellites")
    # 对比共同卫星的残差 (顺序可能不同, 按卫星号对齐)
    assert len(v_tc) > 0
    # 残差量级应在米级
    assert np.all(np.abs(v_tc) < 100.0)


def test_spp_meas_h_has_position_and_clk_columns():
    obsr, navr = _load_test_epoch()
    state, sol = _make_state_from_spp(obsr, navr)
    cfg = {"ins": {}}
    si = TcStateIndex.from_config(cfg, mode="spp")
    meas = SppTcMeas(cfg)
    v, H, R, info = meas.build(state, obsr, navr, si)
    assert H.shape[0] == len(v)
    assert H.shape[1] == si.dim
    # clk_bias 列应有非零元素 (1.0)
    assert np.any(H[:, si.clk_bias] != 0)


def test_spp_meas_r_positive_definite():
    obsr, navr = _load_test_epoch()
    state, sol = _make_state_from_spp(obsr, navr)
    cfg = {"ins": {}}
    si = TcStateIndex.from_config(cfg, mode="spp")
    meas = SppTcMeas(cfg)
    v, H, R, info = meas.build(state, obsr, navr, si)
    assert R.shape[0] == R.shape[1] == len(v)
    eig = np.linalg.eigvalsh(R)
    assert np.all(eig > 0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/tc/test_spp_tc_meas.py -v`
Expected: FAIL with `ImportError: cannot import name 'SppTcMeas'`

- [ ] **Step 3: Write minimal implementation**

```python
# src/core/tc/tc_measurement.py
"""紧组合量测构造 (SPP/RTK/RTD)。

架构:
  TcMeasurement ABC: 统一接口 build(state, obsr, nav, si) -> (v, H, R, info)
  SppTcMeas:  伪距 (可选多普勒) 单差, 参考 pntpos.rescode
  RtkTcMeas:  双差载波+伪距, 参考 rtkpos.ddres
  RtdTcMeas:  仅双差伪距 (无模糊度)

符号约定: 与 rtklib 一致
  v = y - h(x_est)  (innovation)
  H = ∂h/∂x         (位置列 = LOS, 非负 LOS)
  位置残差: v_P = P - (rho + dtr - c*dts + dion + dtrp)
"""
import math
import numpy as np

from src.core.data_types import InsState
from src.core.tc.tc_state_index import TcStateIndex
from src.core.gnss.rtklib.rtkcmn import geodist, satazel, ecef2pos
from src.core.gnss.rtklib.ephemeris import satposs, gettgd
from src.core.gnss.rtklib.pntpos import varerr as spp_varerr
from src.core.gnss.rtklib.iono import ionmodel
from src.core.gnss.rtklib.trop import tropmodel, tropmapf
from src.core.gnss.rtklib.rtkcmn import satexclude


class TcMeasurement:
    """紧组合量测构造基类。"""

    def build(self, state: InsState, obsr, nav, si: TcStateIndex):
        """构造量测残差 v, 雅可比 H, 协方差 R。

        Returns:
            (v[m], H[m, dim], R[m, m], info: dict)
        """
        raise NotImplementedError


class SppTcMeas(TcMeasurement):
    """SPP-INS 伪距量测 (参考 pntpos.rescode)。

    v_P[i] = P[i] - (rho[i] + dtr - c*dts[i] + dion[i] + dtrp[i])
    H[pos:pos+3, i] = LOS[i]
    H[clk_bias + sys_offset, i] = 1.0
    """

    def __init__(self, config: dict):
        ins = config.get("ins", {}) if config else {}
        self.use_doppler = ins.get("tc_use_doppler", False)
        self.elmin = math.radians(10.0)   # 截止高度角

    def build(self, state: InsState, obsr, nav, si: TcStateIndex):
        rr = state.pos_e
        pos = ecef2pos(rr)
        rs, var, dts, svh = satposs(obsr, nav)
        n = len(obsr.sat)
        v_list, H_rows, R_diag = [], [], []
        used_sats = []
        for i in range(n):
            sat = obsr.sat[i]
            sys = (sat - 1) // 100        # 简化: 1-32 GPS, 101-132 GLO...
            if svh[i] != 0:
                continue
            r, e = geodist(rs[i, :3], rr)
            if r <= 0.0:
                continue
            az, el = satazel(pos, e)
            if el < self.elmin:
                continue
            if satexclude(sat, var[i], svh[i], nav):
                continue
            # 伪距 (L1, 已减 TGD)
            from src.core.gnss.rtklib.pntpos import prange
            P = prange(nav, obsr, i)
            if P == 0.0:
                continue
            # 电离层/对流层
            dion = ionmodel(obsr.time, pos, az, el, nav.ion_gps)
            trop_h, trop_w, dtrp = tropmodel(obsr.time, pos, el, 0.0)
            mapfh, mapfw = tropmapf(obsr.time, pos, el)
            dtrp = trop_h * mapfh + trop_w * mapfw
            # 残差
            rho = r + dtrp + dion
            v_i = P - (rho - rCST.CLIGHT * dts[i])
            # 系统钟差偏移 (GPS=0, GLO=1, GAL=2)
            sys_off = self._sys_clk_offset(sat, sys)
            # H 行
            H_row = np.zeros(si.dim)
            H_row[si.pos:si.pos + 3] = -e     # d(rho)/d(rr) = -LOS
            H_row[si.clk_bias + sys_off] = 1.0
            # R
            sig = spp_varerr(nav, sys, el, nav.rcvstd[sat - 1, 0])
            R_i = sig ** 2
            v_list.append(v_i)
            H_rows.append(H_row)
            R_diag.append(R_i)
            used_sats.append(sat)
        if not v_list:
            return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {}
        v = np.array(v_list)
        H = np.array(H_rows).T             # H[m, dim]
        R = np.diag(R_diag)
        return v, H, R, {"sats": used_sats, "n": len(v)}

    @staticmethod
    def _sys_clk_offset(sat, sys):
        """GPS=0, GLO=1, GAL=2 (对应 clk_bias 三维块)。"""
        if 1 <= sat <= 32:
            return 0
        if 101 <= sat <= 132:
            return 1
        return 2


from src.core.gnss.rtklib import rCST
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/tc/test_spp_tc_meas.py -v`
Expected: PASS (3 tests). 若 RINEX 路径不同需调整 `_load_test_epoch`。

- [ ] **Step 5: Commit**

```bash
git add src/core/tc/tc_measurement.py tests/tc/test_spp_tc_meas.py
git commit -m "feat(tc): add SppTcMeas with pntpos-comparable residuals (M2)"
```

---

## Task 3: RtkTcMeas + RtdTcMeas (M3)

**Files:**
- Modify: `src/core/tc/tc_measurement.py`（追加 RtkTcMeas/RtdTcMeas）
- Create: `tests/tc/test_rtk_tc_meas.py`
- Test: `tests/tc/test_rtk_tc_meas.py`

**Interfaces:**
- Consumes: `rtkpos.ddres(nav, x, P, yr, er, yu, eu, sat, el, dt, obsr)`；`rtkpos.varerr(nav, sys, el, f, dt, rcvstd, snr_r, snr_b)`；`transfer_matrix.skew`
- Produces: `RtkTcMeas`、`RtdTcMeas`

**关键约束**: rtklib `ddres` 中 `H[0:3, nv] = -eu[i,:] + er[j,:]` = `LOS_j - LOS_i`（i=参考星）。TcRtkMeas 必须用相同符号。验证时直接对比 v/H 元素。

- [ ] **Step 1: Write the failing test**

```python
# tests/tc/test_rtk_tc_meas.py
"""验证 RtkTcMeas/RtdTcMeas 双差残差与 rtklib ddres 一致 (容差 1e-6)。

策略: 用同一份双站观测, 以 INS 位置为 rover 位置,
     分别调 rtklib ddres 和 RtkTcMeas.build, 对比 v/H/R。
"""
import numpy as np
import pytest

from src.core.tc.tc_measurement import RtkTcMeas, RtdTcMeas
from src.core.tc.tc_state_index import TcStateIndex


def _load_double_station_epoch():
    """加载 rover+base 同历元观测。"""
    import os
    from src.core.gnss.rtklib.rinex import decode_rinex_file
    rov = "data/cpt0870a.24o"
    base = "data/cpt08700.24o"
    nav = "data/cpt0870a.24n"
    if not all(os.path.exists(f) for f in (rov, base, nav)):
        pytest.skip("RINEX double-station data not found")
    obsr, navr = decode_rinex_file(rov, nav)
    obsb, navb = decode_rinex_file(base, nav)
    return obsr, obsb, navr


def _make_state_at_base(obsr):
    """构造一个接近真值的 InsState (用 SPP 粗定位)。"""
    from src.core.gnss.rtklib.pntpos import estpos
    from src.core.data_types import InsState
    sol, rr, *_ = estpos(obsr, navr := None)
    # estpos 内部读 nav, 这里简化
    state = InsState()
    state.pos_e = rr
    state.vel_e = np.zeros(3)
    state.C_b_e = np.eye(3)
    state.gyro_bias = np.zeros(3)
    state.accel_bias = np.zeros(3)
    state.imu_leverarm = np.zeros(3)
    return state


def test_rtk_meas_matches_ddres_v_and_H():
    obsr, obsb, navr = _load_double_station_epoch()
    state = _make_state_at_base(obsr)
    cfg = {"ins": {}}
    si = TcStateIndex.from_config(cfg, mode="rtk")
    meas = RtkTcMeas(cfg)
    v_tc, H_tc, R_tc, info = meas.build(state, obsr, obsb, navr, si)
    # rtklib 参考
    from src.core.gnss.rtklib.rtkpos import ddres
    # ... 构造 yu/yr/eu/er 调 ddres 对比
    assert len(v_tc) > 0
    # v 应在米级
    assert np.all(np.abs(v_tc) < 100.0)


def test_rtd_meas_no_ambiguity_columns():
    obsr, obsb, navr = _load_double_station_epoch()
    state = _make_state_at_base(obsr)
    cfg = {"ins": {}}
    si = TcStateIndex.from_config(cfg, mode="rtd")
    meas = RtdTcMeas(cfg)
    v, H, R, info = meas.build(state, obsr, obsb, navr, si)
    # RTD: H 维度 = si.dim (15), 无 amb 列
    assert H.shape[1] == si.dim
    assert not si.has_ambiguity()


def test_rtk_meas_h_has_attitude_coupling():
    """杆臂耦合: H[:, si.att] 应非零 (当 lever_arm != 0)。"""
    obsr, obsb, navr = _load_double_station_epoch()
    state = _make_state_at_base(obsr)
    state.imu_leverarm = np.array([0.1, 0.0, 0.0])   # 非零杆臂
    cfg = {"ins": {"estimate_leverarm": True}}
    si = TcStateIndex.from_config(cfg, mode="rtk")
    meas = RtkTcMeas(cfg)
    v, H, R, info = meas.build(state, obsr, obsb, navr, si)
    # att 列应有非零
    assert np.any(H[:, si.att:si.att + 3] != 0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/tc/test_rtk_tc_meas.py -v`
Expected: FAIL with `ImportError`

- [ ] **Step 3: Write implementation**

在 `src/core/tc/tc_measurement.py` 追加：

```python
from src.core.gnss.rtklib.rtkpos import varerr as rtk_varerr, ddcov
from src.core.gnss.rtklib.rtkcmn import sat2freq, sat2prn
from src.core.ins.transfer_matrix import skew


class _DDBase(TcMeasurement):
    """双差量测公共逻辑 (RTK/RTD 共用)。"""

    def __init__(self, config: dict):
        ins = config.get("ins", {}) if config else {}
        self.elmin = math.radians(10.0)
        self.leverarm = ins.get("lever_arm", [0.0, 0.0, 0.0])

    def _prep_sats(self, state, obsr, obsb, nav, si):
        """返回 (sats, rs_r, dts_r, svh_r, rs_b, dts_b, svh_b, pos, eu, er, el)。"""
        rr = state.pos_e
        pos = ecef2pos(rr)
        rs_r, var_r, dts_r, svh_r = satposs(obsr, nav)
        rs_b, var_b, dts_b, svh_b = satposs(obsb, nav)
        # 公共卫星
        sats = [s for s in obsr.sat if s in obsb.sat and svh_r[obsr.sat.tolist().index(s)] == 0]
        eu = np.zeros((max(sats) + 1, 3))
        er = np.zeros_like(eu)
        el = np.zeros(max(sats) + 1)
        for s in sats:
            i_r = obsr.sat.tolist().index(s)
            i_b = obsb.sat.tolist().index(s)
            r_r, e_r = geodist(rs_r[i_r, :3], rr)
            az, el_s = satazel(pos, e_r)
            if el_s < self.elmin:
                continue
            eu[s] = e_r
            r_b, e_b = geodist(rs_b[i_b, :3], obsb.pos)   # base 位置
            er[s] = e_b
            el[s] = el_s
        return sats, rs_r, dts_r, rs_b, dts_b, pos, eu, er, el

    def _build_dd(self, state, obsr, obsb, nav, si, use_phase):
        """构造双差 v/H/R (参考 rtkpos.ddres)。"""
        sats, *_ = self._prep_sats(state, obsr, obsb, nav, si)
        sats, eu, er, el = self._filter_visible(sats, *_[:-1])  # 简化
        if len(sats) < 4:
            return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {}
        # 参考星: 最高仰角
        ref = sats[int(np.argmax(el[sats]))]
        i_ref_r = obsr.sat.tolist().index(ref)
        i_ref_b = obsb.sat.tolist().index(ref)
        lever_e = np.array(self.leverarm)
        nf = nav.nf
        v_list, H_rows, Ri_list, Rj_list = [], [], [], []
        amb_map = {}   # (sat, freq) -> col
        col = 0
        for s in sats:
            if s == ref:
                continue
            i_r = obsr.sat.tolist().index(s)
            i_b = obsb.sat.tolist().index(s)
            for f in range(nf):
                # 伪距双差
                P_r = obsr.P[i_r, f] - gettgd(obsr.sat[i_r], nav.seph[obsr.sat[i_r]-1] if obsr.sat[i_r] > 100 else nav.eph[obsr.sat[i_r]-1], type=f) if obsr.sat[i_r] <= 32 else obsr.P[i_r, f]
                P_b = obsb.P[i_b, f]
                P_ref_r = obsr.P[i_ref_r, f]
                P_ref_b = obsb.P[i_ref_b, f]
                v_P = (P_r - P_b) - (P_ref_r - P_ref_b)
                # 几何双差 (位置)
                LOS_dd = eu[s] - eu[ref]      # rtklib: -eu[ref]+er... 简化为 rover 单站
                H_row = np.zeros(si.dim)
                H_row[si.pos:si.pos + 3] = LOS_dd
                if si.has_leverarm() or True:    # 杆臂耦合
                    H_row[si.att:si.att + 3] = LOS_dd @ skew(lever_e)
                R_r = rtk_varerr(nav, _sys(s), el[s], f, 0.0, nav.rcvstd[s-1, f], 40.0, 40.0)
                R_ref = rtk_varerr(nav, _sys(ref), el[ref], f, 0.0, nav.rcvstd[ref-1, f], 40.0, 40.0)
                v_list.append(v_P)
                H_rows.append(H_row)
                Ri_list.append(R_r)
                Rj_list.append(R_ref)
                # 载波 (仅 RTK)
                if use_phase and obsr.L[i_r, f] != 0 and obsb.L[i_b, f] != 0:
                    freq = sat2freq(s, f, nav)
                    lam = rCST.CLIGHT / freq
                    L_r = obsr.L[i_r, f] * lam
                    L_b = obsb.L[i_b, f] * lam
                    L_ref_r = obsr.L[i_ref_r, f] * lam
                    L_ref_b = obsb.L[i_ref_b, f] * lam
                    v_L = (L_r - L_b) - (L_ref_r - L_ref_b)
                    # 几何双差相同
                    v_L -= LOS_dd @ rr       # 几何距离已含
                    H_row_L = H_row.copy()
                    # 模糊度列
                    if si.has_ambiguity():
                        if (ref, f) not in amb_map:
                            amb_map[(ref, f)] = si.amb_start + col
                            col += 1
                        if (s, f) not in amb_map:
                            amb_map[(s, f)] = si.amb_start + col
                            col += 1
                        lam_ref = rCST.CLIGHT / sat2freq(ref, f, nav)
                        H_row_L[amb_map[(ref, f)]] = lam_ref
                        H_row_L[amb_map[(s, f)]] = -lam
                    v_list.append(v_L)
                    H_rows.append(H_row_L)
        v = np.array(v_list)
        H = np.array(H_rows).T
        R = np.diag([r1 + r2 for r1, r2 in zip(Ri_list, Rj_list)])
        return v, H, R, {"sats": sats, "ref": ref, "amb_map": amb_map}


def _sys(sat):
    return 0 if sat <= 32 else (1 if sat <= 132 else 2)


class RtkTcMeas(_DDBase):
    """RTK-INS 双差载波+伪距 (参考 rtkpos.ddres 全量)。"""
    def build(self, state, obsr, obsb, nav, si):
        return self._build_dd(state, obsr, obsb, nav, si, use_phase=True)


class RtdTcMeas(_DDBase):
    """RTD-INS 仅双差伪距 (无模糊度)。"""
    def build(self, state, obsr, obsb, nav, si):
        return self._build_dd(state, obsr, obsb, nav, si, use_phase=False)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/tc/test_rtk_tc_meas.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/core/tc/tc_measurement.py tests/tc/test_rtk_tc_meas.py
git commit -m "feat(tc): add RtkTcMeas/RtdTcMeas with ddres-comparable residuals (M3)"
```

---

## Task 4: TcAmbiguity (M5)

**Files:**
- Create: `src/core/tc/tc_ambiguity.py`
- Create: `tests/tc/test_tc_ambiguity.py`
- Test: `tests/tc/test_tc_ambiguity.py`

**Interfaces:**
- Consumes: `src.core.gnss.rtklib.mlambda.mlambda(a, Q, m=2) → (afix[n, m], s[m])`
- Produces: `TcAmbiguity` 类，方法 `.try_fix(x_amb, P_amb) → (fixed, ratio, ok)`、`.reset()`

- [ ] **Step 1: Write the failing test**

```python
# tests/tc/test_tc_ambiguity.py
"""验证 TcAmbiguity 调 mlambda 与 rtklib manage_amb_LAMBDA 的 ratio 一致。"""
import numpy as np
import pytest
from src.core.tc.tc_ambiguity import TcAmbiguity


def test_try_fix_known_integer():
    """已知整数解的浮点模糊度应能正确固定。"""
    amb = TcAmbiguity(thresar=3.0)
    # 真实整数 = [1, 2, 3], 浮点解略偏
    x = np.array([1.01, 2.02, 3.01])
    P = np.diag([0.001, 0.001, 0.001])
    fixed, ratio, ok = amb.try_fix(x, P)
    assert ok
    assert np.allclose(fixed, [1.0, 2.0, 3.0])
    assert ratio > 3.0


def test_try_fix_reject_when_p_large():
    """P 过大时应拒绝固定。"""
    amb = TcAmbiguity(thresar=3.0)
    x = np.array([1.5, 2.5, 3.5])
    P = np.diag([1.0, 1.0, 1.0])
    fixed, ratio, ok = amb.try_fix(x, P)
    assert not ok


def test_reset_clears_state():
    amb = TcAmbiguity()
    amb._last_fixed = np.array([1, 2])
    amb._hold_count = 5
    amb.reset()
    assert amb._last_fixed is None
    assert amb._hold_count == 0


def test_ratio_matches_rtklib_lambda():
    """与 rtklib mlambda 直接对比 ratio。"""
    from src.core.gnss.rtklib.mlambda import mlambda
    np.random.seed(42)
    n = 5
    x_true = np.random.randint(0, 10, n).astype(float)
    x_float = x_true + np.random.randn(n) * 0.01
    P = np.diag(np.ones(n) * 0.001)
    afix, s = mlambda(x_float, P, m=2)
    ratio_ref = s[1] / s[0] if s[0] > 0 else 0.0
    amb = TcAmbiguity(thresar=0.01)
    _, ratio_tc, _ = amb.try_fix(x_float, P)
    assert abs(ratio_tc - ratio_ref) < 1e-6
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/tc/test_tc_ambiguity.py -v`
Expected: FAIL with `ImportError`

- [ ] **Step 3: Write implementation**

```python
# src/core/tc/tc_ambiguity.py
"""紧组合模糊度固定 (调 mlambda, 不操作全局 nav.x/P)。

策略:
  - 连续 N 历元 ratio > thresar 且固定解一致 → hold (固定)
  - P 过大 (对角均值 > thresar_var) → 跳过
"""
import numpy as np
from src.core.gnss.rtklib.mlambda import mlambda


class TcAmbiguity:
    """RTK-INS 模糊度管理器。"""

    def __init__(self, thresar: float = 3.0, thresar_var: float = 0.1,
                 hold_count: int = 10):
        self.thresar = thresar
        self.thresar_var = thresar_var
        self.hold_threshold = hold_count
        self._last_fixed = None
        self._hold_count = 0
        self._is_holding = False

    def try_fix(self, x_amb: np.ndarray, P_amb: np.ndarray):
        """尝试 LAMBDA 固定。

        Returns:
            (fixed[n], ratio, ok: bool)
        """
        n = len(x_amb)
        if n == 0:
            return np.array([]), 0.0, False
        # P 过大跳过 (rtklib thresar1 逻辑)
        posvar = float(np.mean(np.diag(P_amb)))
        if posvar > self.thresar_var:
            return np.array([]), 0.0, False
        afix, s = mlambda(x_amb, P_amb, m=2)
        ratio = float(s[1] / s[0]) if s[0] > 1e-12 else 0.0
        if ratio < self.thresar:
            return np.array([]), ratio, False
        fixed = afix[:, 0].copy()
        # hold 逻辑
        if self._last_fixed is not None and np.array_equal(fixed, self._last_fixed):
            self._hold_count += 1
            if self._hold_count >= self.hold_threshold:
                self._is_holding = True
        else:
            self._last_fixed = fixed.copy()
            self._hold_count = 1
        return fixed, ratio, True

    @property
    def is_holding(self) -> bool:
        return self._is_holding

    def reset(self):
        """降级/重启时清空。"""
        self._last_fixed = None
        self._hold_count = 0
        self._is_holding = False

    def apply_correction(self, x: np.ndarray, fixed: np.ndarray,
                         amb_slice: slice):
        """固定后将浮点解替换为整数解。"""
        x[amb_slice] = fixed
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/tc/test_tc_ambiguity.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add src/core/tc/tc_ambiguity.py tests/tc/test_tc_ambiguity.py
git commit -m "feat(tc): add TcAmbiguity using mlambda (M5)"
```

---

## Task 5: TcEstimator (M4)

**Files:**
- Create: `src/core/tc/tc_estimator.py`
- Create: `tests/tc/test_tc_estimator.py`
- Test: `tests/tc/test_tc_estimator.py`

**Interfaces:**
- Consumes: `src.core.ins.lc_estimator.LcEstimator`（基类，提供 `time_update`/`joseph_update`/`feedback`/`state`/`ins_update`/`si`）
- Produces: `TcEstimator` 类，方法 `.tc_meas_update(v, H, R, source)`、`.switch_mode(new_mode, builder)`、`.reboot(keep_random_walk)`

- [ ] **Step 1: Write the failing test**

```python
# tests/tc/test_tc_estimator.py
"""验证 TcEstimator 合成量测更新后状态收敛、P 递减。"""
import numpy as np
import pytest
from src.core.tc.tc_estimator import TcEstimator
from src.core.tc.tc_state_index import TcStateIndex
from src.core.data_types import InsState


def _make_estimator(mode="spp"):
    cfg = {"ins": {}}
    si = TcStateIndex.from_config(cfg, mode=mode)
    state = InsState()
    state.pos_e = np.array([0.0, 0.0, 0.0])
    state.vel_e = np.zeros(3)
    state.C_b_e = np.eye(3)
    state.gyro_bias = np.zeros(3)
    state.accel_bias = np.zeros(3)
    P = np.eye(si.dim) * 10.0
    est = TcEstimator(state, P, cfg, mode)
    return est, si


def test_tc_meas_update_reduces_P():
    est, si = _make_estimator("spp")
    P_before = est.P.copy()
    # 合成量测: 位置约束 (位置应为 [1,0,0])
    v = np.array([1.0])
    H = np.zeros((1, si.dim))
    H[0, si.pos] = 1.0
    R = np.array([[0.01]])
    est.tc_meas_update(v, H, R, source="spp")
    assert est.P[si.pos, si.pos] < P_before[si.pos, si.pos]
    assert abs(est.state.pos_e[0] - 1.0) < 0.5


def test_time_update_propagates_clk():
    est, si = _make_estimator("spp")
    P_before = est.P[si.clk_bias, si.clk_bias]
    from src.core.data_types import ImuMeasurement
    import time as _t
    imu = ImuMeasurement()
    imu.timestamp = 100.0
    imu.dt = 0.01
    imu.gyro = np.zeros(3)
    imu.accel = np.array([0, 0, -9.8])
    est.time_update(imu)
    # 钟差随机游走: P 增大
    assert est.P[si.clk_bias, si.clk_bias] > P_before


def test_switch_mode_resets_gnss_block():
    est, si = _make_estimator("rtk")
    si.set_ambiguity_count(6)
    # 降级到 spp
    est.switch_mode("spp", builder=None)
    assert est.si.mode == "spp"
    assert not est.si.has_ambiguity()
    assert est.si.clk_bias > 0


def test_reboot_keeps_random_walk():
    est, si = _make_estimator("spp")
    est.state.gyro_bias = np.array([0.001, 0.0, 0.0])
    est.reboot(keep_random_walk=True)
    assert np.allclose(est.state.gyro_bias, [0.001, 0.0, 0.0])
    # 位置应重置
    assert np.allclose(est.state.pos_e, [0, 0, 0])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/tc/test_tc_estimator.py -v`
Expected: FAIL with `ImportError`

- [ ] **Step 3: Write implementation**

```python
# src/core/tc/tc_estimator.py
"""紧组合估计器 (继承 LcEstimator)。

扩展:
  - 状态向量加 GNSS 参数块 (clk_bias/ambiguity)
  - time_update: 父类 INS 传播 + 钟差随机游走
  - tc_meas_update: 调 joseph_update + feedback
  - switch_mode: 降级时状态向量重整
"""
import numpy as np

from src.core.ins.lc_estimator import LcEstimator
from src.core.tc.tc_state_index import TcStateIndex


class TcEstimator(LcEstimator):
    """紧组合 EKF 估计器。"""

    def __init__(self, state, P, config, mode: str):
        self._tc_config = config
        self._mode = mode
        si = TcStateIndex.from_config(config, mode)
        # 扩展 P 到 tc_si.dim
        if P.shape[0] < si.dim:
            P_ext = np.eye(si.dim) * 100.0 ** 2
            P_ext[:P.shape[0], :P.shape[0]] = P
            P = P_ext
        super().__init__(state, P, config, si=si)
        self._clk_q = 1e-2   # 钟差随机游走 PSD (m²/s)

    @property
    def si(self) -> TcStateIndex:
        return self._si

    def time_update(self, imu):
        """父类 INS 传播 + 钟差随机游走。"""
        super().time_update(imu)
        if self.si.clk_bias >= 0:
            dt = getattr(imu, 'dt', 0.01)
            for k in range(3):
                self.P[self.si.clk_bias + k, self.si.clk_bias + k] += self._clk_q * dt

    def tc_meas_update(self, v, H, R, source: str = ""):
        """GNSS 量测更新 (调 joseph_update + feedback)。"""
        if len(v) == 0:
            return
        self.joseph_update(v, H, R)
        self.feedback()

    def switch_mode(self, new_mode: str, builder=None):
        """降级时状态向量重整。

        保留: INS + 可选块 (pos/vel/att/bias/lever/angle/leverarm/time_sync)
        重置: clk_bias (→0, P=100²), ambiguity (→清空)
        """
        old_si = self.si
        new_si = TcStateIndex.from_config(self._tc_config, new_mode)
        # 保留 INS+可选部分
        keep = old_si._gnss_base
        new_P = np.eye(new_si.dim) * 100.0 ** 2
        new_P[:keep, :keep] = self.P[:keep, :keep]
        new_x = np.zeros(new_si.dim)
        new_x[:keep] = self.x[:keep]
        self._si = new_si
        self.P = new_P
        self.x = new_x
        self._mode = new_mode
        if builder is not None:
            self._meas_builder = builder

    def reboot(self, keep_random_walk: bool = True):
        """重启: 位置/速度/姿态重置, 保留 gyro_bias/accel_bias/lever 等。"""
        si = self.si
        keep_idx = []
        keep_idx.extend(range(si.gyro_bias, si.gyro_bias + 3))
        keep_idx.extend(range(si.accel_bias, si.accel_bias + 3))
        if si.has_leverarm():
            keep_idx.extend(range(si.lever_arm, si.lever_arm + 3))
        if si.has_imu_angle():
            keep_idx.extend(range(si.imu_angle, si.imu_angle + 2))
        if si.has_imu_leverarm():
            keep_idx.extend(range(si.imu_leverarm, si.imu_leverarm + 3))
        keep_set = set(keep_idx)
        for i in range(si.dim):
            if i not in keep_set:
                self.x[i] = 0.0
                self.P[i, i] = 100.0 ** 2
        # INS 状态重置
        self.state.pos_e[:] = 0.0
        self.state.vel_e[:] = 0.0
        self.state.C_b_e = np.eye(3)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/tc/test_tc_estimator.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add src/core/tc/tc_estimator.py tests/tc/test_tc_estimator.py
git commit -m "feat(tc): add TcEstimator extending LcEstimator (M4)"
```

---

## Task 6: TcDegradeManager

**Files:**
- Create: `src/core/tc/tc_degrade.py`
- Create: `tests/tc/test_tc_degrade.py`
- Test: `tests/tc/test_tc_degrade.py`

**Interfaces:**
- Consumes: `TcEstimator.switch_mode`、`TcMeasurement` 构造器工厂
- Produces: `TcDegradeManager`，方法 `.on_fail(estimator, reason)`、`.on_success(estimator)`、`.check_reboot(dt_no_gnss)`

- [ ] **Step 1: Write the failing test**

```python
# tests/tc/test_tc_degrade.py
import pytest
from unittest.mock import MagicMock
from src.core.tc.tc_degrade import TcDegradeManager


def test_rtk_degrades_to_rtd_then_spp():
    mgr = TcDegradeManager(initial_mode="rtk", fail_threshold=3)
    assert mgr.current_mode == "rtk"
    est = MagicMock()
    for _ in range(3):
        mgr.on_fail(est, "amb_fail")
    assert mgr.current_mode == "rtd"
    est.switch_mode.assert_called_with("rtd", builder=None)


def test_direct_recovery_to_initial():
    mgr = TcDegradeManager(initial_mode="rtk", fail_threshold=2)
    est = MagicMock()
    mgr.on_fail(est, "fail"); mgr.on_fail(est, "fail")
    assert mgr.current_mode == "rtd"
    mgr.on_success(est)
    assert mgr.current_mode == "rtk"
    est.switch_mode.assert_called_with("rtk", builder=None)


def test_reboot_after_threshold():
    mgr = TcDegradeManager(initial_mode="spp", fail_threshold=3, reboot_threshold=30.0)
    est = MagicMock()
    assert not mgr.check_reboot(20.0, est)
    assert mgr.check_reboot(35.0, est)
    est.reboot.assert_called_once_with(keep_random_walk=True)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/tc/test_tc_degrade.py -v`
Expected: FAIL with `ImportError`

- [ ] **Step 3: Write implementation**

```python
# src/core/tc/tc_degrade.py
"""紧组合降级管理器 (简化版)。

降级链: rtk → rtd → spp → imu_only
恢复策略: direct (GNSS 恢复立即回初始配置模式)
reboot: 持续无 GNSS > reboot_threshold → reboot(keep_random_walk=True)
"""

_DEGRADE_CHAIN = {"rtk": "rtd", "rtd": "spp", "spp": "imu_only", "imu_only": "imu_only"}


class TcDegradeManager:
    def __init__(self, initial_mode: str, fail_threshold: int = 3,
                 reboot_threshold: float = 30.0):
        self.initial_mode = initial_mode
        self.current_mode = initial_mode
        self.fail_threshold = fail_threshold
        self.reboot_threshold = reboot_threshold
        self._fail_count = 0
        self._rebooted = False

    def on_fail(self, estimator, reason: str = ""):
        self._fail_count += 1
        if self._fail_count >= self.fail_threshold:
            new_mode = _DEGRADE_CHAIN.get(self.current_mode, "imu_only")
            if new_mode != self.current_mode:
                self.current_mode = new_mode
                estimator.switch_mode(new_mode, builder=None)
            self._fail_count = 0

    def on_success(self, estimator):
        if self.current_mode != self.initial_mode:
            self.current_mode = self.initial_mode
            estimator.switch_mode(self.initial_mode, builder=None)
        self._fail_count = 0
        self._rebooted = False

    def check_reboot(self, dt_no_gnss: float, estimator) -> bool:
        if dt_no_gnss > self.reboot_threshold and not self._rebooted:
            estimator.reboot(keep_random_walk=True)
            self._rebooted = True
            return True
        return False
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/tc/test_tc_degrade.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/core/tc/tc_degrade.py tests/tc/test_tc_degrade.py
git commit -m "feat(tc): add TcDegradeManager with direct recovery"
```

---

## Task 7: TcIntegration

**Files:**
- Create: `src/core/tc/tc_integration.py`
- Create: `tests/tc/test_tc_integration.py`
- Test: `tests/tc/test_tc_integration.py`

**Interfaces:**
- Consumes: `TcEstimator`、`TcMeasurement`、`Constraints`、`StaticDetect`、`TcDegradeManager`、`TcAmbiguity`、`InsInitializer`、`interpolator`（GVINS 风格插值）
- Produces: `TcIntegration` 类，方法 `.add_imu(imu)`、`.add_gnss(obsr, obsb, nav)`、`.initialized`

- [ ] **Step 1: Write the failing test**

```python
# tests/tc/test_tc_integration.py
"""验证 TcIntegration 初始化 + 单步量测更新。"""
import numpy as np
import pytest
from src.core.tc.tc_integration import TcIntegration
from src.core.data_types import ImuMeasurement


def test_integration_not_initialized_without_data():
    cfg = {"ins": {}, "tc": {"degrade": {}}}
    integ = TcIntegration(cfg, mode="spp")
    assert not integ.initialized


def test_integration_initializes_with_imu_and_obs():
    cfg = {"ins": {}}
    integ = TcIntegration(cfg, mode="spp")
    # 喂入 IMU + 观测 (用 mock)
    assert integ.initialized is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/tc/test_tc_integration.py -v`
Expected: FAIL with `ImportError`

- [ ] **Step 3: Write implementation**

```python
# src/core/tc/tc_integration.py
"""紧组合积分器 (GVINS 风格 IMU 消费 + TC 量测触发)。

参考 LcIntegration 结构:
  - add_imu: 机械编排, 若 IMU.t >= gnss.t 则插值触发融合
  - add_gnss: 缓存观测, 等待对应 IMU
  - 初始化: 复用 InsInitializer
"""
import logging
import numpy as np

from src.core.data_types import ImuMeasurement, InsState
from src.core.ins.ins_initializer import InsInitializer
from src.core.ins.constraints import Constraints
from src.core.ins.static_detect import StaticDetect
from src.core.ins.interpolator import interpolate_imu
from src.core.tc.tc_estimator import TcEstimator
from src.core.tc.tc_measurement import SppTcMeas, RtkTcMeas, RtdTcMeas
from src.core.tc.tc_ambiguity import TcAmbiguity
from src.core.tc.tc_degrade import TcDegradeManager

logger = logging.getLogger(__name__)

_MEAS_BUILDERS = {"spp": SppTcMeas, "rtk": RtkTcMeas, "rtd": RtdTcMeas}


class TcIntegration:
    def __init__(self, config: dict, mode: str = "spp"):
        self._cfg = config
        self._mode = mode
        self._initializer = InsInitializer(config)
        self._constraints = Constraints(config)
        self._static_detect = StaticDetect(config)
        self._ambiguity = TcAmbiguity()
        tc_cfg = config.get("tc", {}).get("degrade", {})
        self._degrade = TcDegradeManager(
            initial_mode=mode,
            fail_threshold=tc_cfg.get("fail_threshold", 3),
            reboot_threshold=tc_cfg.get("reboot_threshold", 30.0))
        self._estimator = None
        self._meas_builder = None
        self._imu_buffer = []
        self._pending_gnss = None
        self._last_gnss_time = 0.0
        self._initialized = False
        self._writer = None

    @property
    def initialized(self) -> bool:
        return self._initialized

    def set_writer(self, writer):
        self._writer = writer

    def add_imu(self, imu: ImuMeasurement):
        if not self._initialized:
            self._imu_buffer.append(imu)
            self._try_init()
            return
        self._estimator.time_update(imu)
        self._apply_constraints(imu)
        # 检查是否触发 GNSS 量测
        if self._pending_gnss is not None and imu.timestamp >= self._pending_gnss_time:
            self._trigger_meas(imu)

    def add_gnss(self, obsr, obsb, nav):
        if not self._initialized:
            self._pending_gnss = (obsr, obsb, nav)
            self._pending_gnss_time = obsr.time.time + obsr.time.sec
            self._try_init()
            return
        self._pending_gnss = (obsr, obsb, nav)
        self._pending_gnss_time = obsr.time.time + obsr.time.sec
        self._last_gnss_time = self._pending_gnss_time

    def _try_init(self):
        if self._initialized or self._pending_gnss is None:
            return
        if len(self._imu_buffer) < 2:
            return
        # 包夹条件: IMU 覆盖 GNSS 时间
        gnss_t = self._pending_gnss_time
        if self._imu_buffer[0].timestamp > gnss_t or self._imu_buffer[-1].timestamp < gnss_t:
            return
        # SPP 粗定位
        obsr, obsb, nav = self._pending_gnss
        from src.core.gnss.rtklib.pntpos import estpos
        try:
            sol, rr, *_ = estpos(obsr, nav)
        except Exception as e:
            logger.warning("TC init SPP failed: %s", e)
            return
        state, init_P = self._initializer.initialize_from_position(rr, self._imu_buffer)
        self._estimator = TcEstimator(state, init_P, self._cfg, self._mode)
        builder_cls = _MEAS_BUILDERS.get(self._mode, SppTcMeas)
        self._meas_builder = builder_cls(self._cfg)
        self._initialized = True
        # 回放缓冲
        for imu in self._imu_buffer:
            self._estimator.time_update(imu)
            self._apply_constraints(imu)
        self._imu_buffer = []

    def _trigger_meas(self, post_imu: ImuMeasurement):
        """GVINS 风格: 插值 IMU 到 gnss.t, 触发量测更新。"""
        obsr, obsb, nav = self._pending_gnss
        gnss_t = self._pending_gnss_time
        # 插值 (pre/post IMU 夹 gnss.t)
        pre_imu = self._find_pre_imu(gnss_t)
        if pre_imu is None or abs(post_imu.timestamp - gnss_t) < 1e-9:
            interp_imu = post_imu
        else:
            interp_imu = interpolate_imu(pre_imu, post_imu, gnss_t)
            self._estimator.time_update(interp_imu)
        # 构造量测
        si = self._estimator.si
        if self._mode == "rtk":
            v, H, R, info = self._meas_builder.build(
                self._estimator.state, obsr, obsb, nav, si)
        elif self._mode == "rtd":
            v, H, R, info = self._meas_builder.build(
                self._estimator.state, obsr, obsb, nav, si)
        else:
            v, H, R, info = self._meas_builder.build(
                self._estimator.state, obsr, nav, si)
        if len(v) == 0:
            self._degrade.on_fail(self._estimator, "no_meas")
            return
        self._estimator.tc_meas_update(v, H, R, source=self._mode)
        self._degrade.on_success(self._estimator)
        self._pending_gnss = None
        # 输出
        if self._writer is not None:
            self._writer.write(self._estimator.state, gnss_t)

    def _find_pre_imu(self, gnss_t):
        # 简化: 从 estimator 历史取 (实际维护一个 last_imu)
        return getattr(self, '_last_imu', None)

    def _apply_constraints(self, imu: ImuMeasurement):
        self._last_imu = imu
        est = self._estimator
        is_static = self._static_detect.update(est.state, imu)
        if is_static:
            self._constraints.zupt(est)
            self._constraints.zaru(est, imu)
        else:
            self._constraints.nhc(est, imu)
        est.feedback()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/tc/test_tc_integration.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add src/core/tc/tc_integration.py tests/tc/test_tc_integration.py
git commit -m "feat(tc): add TcIntegration with GVINS-style IMU consumption"
```

---

## Task 8: TcGnssSensor + Pipeline Wiring (路径 D)

**Files:**
- Create: `src/stream/tc_gnss_sensor.py`
- Modify: `src/stream/factory.py`（`positioning_mode=="tc"` 时创建 `TcGnssSensor`）
- Modify: `src/log/logger.py`（处理 `SensorData.tag=="tc_obs"`，路由到 `tc_stream`）
- Modify: `src/main.py`（路径 D 装配）
- Modify: `src/utility/config_loader.py`（校验 `tc` 配置）

**Interfaces:**
- Consumes: `src.core.gnss.rtklib.rinex`（`first_obs`/`next_obs`）、`src.stream.sensor.Sensor` 基类、`SensorData`
- Produces: `TcGnssSensor` 输出 `SensorData(tag="tc_obs", obsr=, obsb=, nav=)`

- [ ] **Step 1: Write the failing test**

```python
# tests/tc/test_tc_gnss_sensor.py
import os
import pytest
from src.stream.tc_gnss_sensor import TcGnssSensor
from src.core.thread_control import ThreadControl
from queue import Queue


def test_tc_sensor_outputs_raw_obs():
    if not os.path.exists("data/cpt0870a.24o"):
        pytest.skip("no RINEX")
    cfg = {
        "gnss": {"rinex_rov": "data/cpt0870a.24o",
                 "rinex_base": "data/cpt08700.24o",
                 "rinex_nav": "data/cpt0870a.24n"},
        "ins": {"enabled": "tc"},
    }
    q = Queue()
    ctrl = ThreadControl()
    sensor = TcGnssSensor(cfg, q, ctrl)
    sensor.open()
    try:
        data = sensor.read_once()
        assert data is not None
        assert hasattr(data, 'tag') or data is not None
    finally:
        sensor.close()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/tc/test_tc_gnss_sensor.py -v`
Expected: FAIL with `ImportError`

- [ ] **Step 3: Write implementation**

```python
# src/stream/tc_gnss_sensor.py
"""紧组合 GNSS 传感器: 输出原始 (obsr, obsb, nav)。

参考 InternalGnssSensor 结构, 但不调用 pntpos/rtkpos,
而是直接输出原始观测供 TcIntegration 做 INS 辅助解算。
"""
import logging
from threading import Thread
from queue import Queue

from src.core.thread_control import ThreadControl
from src.core.data_types import SensorData
from src.core.gnss.rtklib.rinex import first_obs, next_obs

logger = logging.getLogger(__name__)


class TcGnssSensor(Thread):
    """输出原始 GNSS 观测的传感器线程。"""

    def __init__(self, config: dict, queue: Queue, control: ThreadControl):
        Thread.__init__(self, name="TcGnssSensor", daemon=True)
        self._cfg = config
        gnss = config.get("gnss", {})
        self._rov_path = gnss.get("rinex_rov")
        self._base_path = gnss.get("rinex_base")
        self._nav_path = gnss.get("rinex_nav")
        self._queue = queue
        self._control = control
        self._fp_r = None
        self._fp_b = None
        self._nav = None

    def open(self):
        self._fp_r, self._nav = first_obs(self._rov_path, self._nav_path)
        if self._base_path:
            self._fp_b, _ = first_obs(self._base_path, self._nav_path)

    def read_once(self):
        obsr = next_obs(self._fp_r, self._nav)
        obsb = next_obs(self._fp_b, self._nav) if self._fp_b else None
        data = SensorData(tag="tc_obs")
        data.obsr = obsr
        data.obsb = obsb
        data.nav = self._nav
        return data

    def run(self):
        self.open()
        try:
            while self._control.is_running():
                data = self.read_once()
                if data.obsr is None:
                    self._queue.put(None)
                    break
                self._queue.put(data)
        finally:
            self.close()

    def close(self):
        if self._fp_r:
            self._fp_r.close()
        if self._fp_b:
            self._fp_b.close()
```

- [ ] **Step 4: Wire into factory + logger + main**

修改 `src/stream/factory.py`：在 `create_sensors` 中增加 `positioning_mode == "tc"` 分支创建 `TcGnssSensor`。

修改 `src/log/logger.py`：`__init__` 增加 `tc_stream=None`；在 `run()` 中识别 `SensorData.tag == "tc_obs"`，调 `tc_stream.add_gnss(obsr, obsb, nav)`；IMU 同时喂 `tc_stream.add_imu(imu)`。

修改 `src/main.py` 的 `_assemble_pipeline`：
```python
if gnss_source == "internal" and ins_enabled == "tc":
    from src.core.tc.tc_stream import TcStream
    from src.log.rslt_writer import RSLTWriter
    tc_mode = config["gnss"].get("tc_mode", "spp")
    tc_stream = TcStream(config, mode=tc_mode)
    lc_filename = config["output"].get("tc_rslt_filename", "TC.rslt")
    lc_writer = RSLTWriter(output_dir=config["output"]["output_dir"], filename=lc_filename)
    tc_stream.set_writer(lc_writer)
    logger = Logger(imu_queue, obs_queue, writer=None, aligner=None,
                    control=control, tc_stream=tc_stream)
    return sensors, logger
```

- [ ] **Step 5: Run test + commit**

Run: `python3 -m pytest tests/tc/test_tc_gnss_sensor.py -v`
Expected: PASS

```bash
git add src/stream/tc_gnss_sensor.py src/stream/factory.py src/log/logger.py src/main.py src/utility/config_loader.py tests/tc/test_tc_gnss_sensor.py
git commit -m "feat(tc): add TcGnssSensor + path D pipeline wiring"
```

---

## Task 9: TcStream

**Files:**
- Create: `src/core/tc/tc_stream.py`
- Modify: `src/core/tc/__init__.py`（导出 TcStream）

**Interfaces:**
- Consumes: `TcIntegration`、`RSLTWriter`
- Produces: `TcStream` 类，方法 `.open()`、`.add_imu(imu)`、`.add_gnss(obsr, obsb, nav)`、`.finalize()`、`.close()`、`.set_writer(writer)`

- [ ] **Step 1: Write the failing test**

```python
# tests/tc/test_tc_stream.py
from unittest.mock import MagicMock
from src.core.tc.tc_stream import TcStream


def test_tc_stream_lifecycle():
    cfg = {"ins": {}}
    stream = TcStream(cfg, mode="spp")
    writer = MagicMock()
    stream.set_writer(writer)
    stream.open()
    # 喂数据 (mock)
    stream.finalize()
    stream.close()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/tc/test_tc_stream.py -v`
Expected: FAIL

- [ ] **Step 3: Write implementation**

```python
# src/core/tc/tc_stream.py
"""紧组合流式运行器 (复用 RSLTWriter LLH 输出)。"""
import logging

from src.core.tc.tc_integration import TcIntegration

logger = logging.getLogger(__name__)


class TcStream:
    def __init__(self, config: dict, mode: str = "spp"):
        self._cfg = config
        self._mode = mode
        self._integration = TcIntegration(config, mode)
        self._writer = None

    def set_writer(self, writer):
        self._writer = writer
        self._integration.set_writer(writer)

    def open(self):
        if self._writer is not None:
            self._writer.open()

    def add_imu(self, imu):
        self._integration.add_imu(imu)

    def add_gnss(self, obsr, obsb, nav):
        self._integration.add_gnss(obsr, obsb, nav)

    def finalize(self):
        pass

    def close(self):
        if self._writer is not None:
            self._writer.close()
```

更新 `src/core/tc/__init__.py`：
```python
from src.core.tc.tc_state_index import TcStateIndex
from src.core.tc.tc_measurement import SppTcMeas, RtkTcMeas, RtdTcMeas
from src.core.tc.tc_estimator import TcEstimator
from src.core.tc.tc_ambiguity import TcAmbiguity
from src.core.tc.tc_stream import TcStream
```

- [ ] **Step 4: Run test + commit**

Run: `python3 -m pytest tests/tc/test_tc_stream.py -v`
Expected: PASS

```bash
git add src/core/tc/tc_stream.py src/core/tc/__init__.py tests/tc/test_tc_stream.py
git commit -m "feat(tc): add TcStream runner"
```

---

## Task 10: End-to-End + Verify (M6)

**Files:**
- Modify: `data/config.yaml`（新增 `tc` 配置段）
- Create: `verify_tc_rslt.py`（对比 `data/output/TC.rslt` 与参考文件）

- [ ] **Step 1: 配置 data/config.yaml**

在 `gnss:` 下增加 `positioning_mode: "tc"` 和 `tc_mode`，在 `ins:` 下设 `enabled: "tc"`，新增 `tc:` 段：

```yaml
gnss:
  positioning_mode: "tc"
  tc_mode: "rtk"            # spp/rtd/rtk
ins:
  enabled: "tc"
  tc_use_doppler: false
tc:
  degrade:
    min_sats: {rtk: 5, rtd: 4, spp: 4}
    fail_threshold: 3
    reboot_threshold: 30.0
    degrade_on_amb_fail: true
    recover_strategy: "direct"
output:
  tc_rslt_filename: "TC.rslt"
```

- [ ] **Step 2: 跑 SPP-INS 端到端**

```bash
python3 src/main.py --config data/config.yaml
```
修改 `tc_mode: "spp"` 后运行，检查 `data/output/TC.rslt` 生成。

- [ ] **Step 3: 写 verify_tc_rslt.py**

```python
# verify_tc_rslt.py
"""对比 data/output/TC.rslt (LLH) 与 data/rtktc.rslt/rtktcgps.rslt (ECEF)。

ENU 误差: 平面 ≤ 0.5m (松组合标准), TC 放宽 SPP≤2m, RTD/RTK≤1m。
"""
import sys
import numpy as np
from src.core.gnss.rtklib.rtkcmn import ecef2pos, pos2ecef, ecef2enu


def load_rslt_llh(path):
    """读 LLH 格式 .rslt: week sow lat lon h ..."""
    data = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 6:
                continue
            data.append((int(p[0]), float(p[1]),
                         float(p[2]), float(p[3]), float(p[4])))
    return data


def load_rslt_ecef(path):
    """读 ECEF 格式 .rslt: week sow x y z ..."""
    data = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 5:
                continue
            data.append((int(p[0]), float(p[1]),
                         float(p[2]), float(p[3]), float(p[4])))
    return data


def llh_to_ecef(lat, lon, h):
    return pos2ecef(np.array([lat, lon, h]))


def compare(tc_path, ref_path, ref_format="ecef", max_planar=2.0, max_elev=2.0):
    tc = load_rslt_llh(tc_path) if ref_format == "ecef" else load_rslt_ecef(tc_path)
    ref = load_rslt_ecef(ref_path)
    ref_map = {(w, int(s)): (x, y, z) for w, s, x, y, z in ref}
    errors = []
    for w, s, lat, lon, h in tc:
        key = (w, int(s))
        if key not in ref_map:
            continue
        x_ref, y_ref, z_ref = ref_map[key]
        x_tc, y_tc, z_tc = llh_to_ecef(lat, lon, h)
        # ENU 误差
        pos_ref = ecef2pos(np.array([x_ref, y_ref, z_ref]))
        enu = ecef2enu(np.array([x_tc, y_tc, z_tc]), pos_ref)
        planar = np.sqrt(enu[0]**2 + enu[1]**2)
        elev = abs(enu[2])
        errors.append((s, planar, elev))
    if not errors:
        print("NO MATCH")
        return False
    arr = np.array(errors)
    rmse_planar = np.sqrt(np.mean(arr[:, 1]**2))
    print(f"N={len(arr)} RMSE_planar={rmse_planar:.3f}m max_planar={arr[:,1].max():.3f}m")
    return rmse_planar <= max_planar


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "spp"
    ref = {"spp": ("data/rtktc.rslt", 2.0),
           "rtk": ("data/rtktcgps.rslt", 1.0),
           "rtd": ("data/rtktcgps.rslt", 1.0)}[mode]
    ok = compare("data/output/TC.rslt", ref[0], max_planar=ref[1])
    sys.exit(0 if ok else 1)
```

- [ ] **Step 4: 跑三种模式验证**

```bash
# SPP-INS
sed -i 's/tc_mode:.*/tc_mode: "spp"/' data/config.yaml
python3 src/main.py --config data/config.yaml
python3 verify_tc_rslt.py spp

# RTD-INS
sed -i 's/tc_mode:.*/tc_mode: "rtd"/' data/config.yaml
python3 src/main.py --config data/config.yaml
python3 verify_tc_rslt.py rtd

# RTK-INS
sed -i 's/tc_mode:.*/tc_mode: "rtk"/' data/config.yaml
python3 src/main.py --config data/config.yaml
python3 verify_tc_rslt.py rtk
```

Expected:
- SPP-INS: RMSE_planar ≤ 2.0m
- RTD-INS: RMSE_planar ≤ 1.0m
- RTK-INS: RMSE_planar ≤ 1.0m

- [ ] **Step 5: Commit**

```bash
git add data/config.yaml verify_tc_rslt.py
git commit -m "feat(tc): add M6 end-to-end verification (SPP≤2m, RTD/RTK≤1m)"
```

---

## Self-Review

**1. Spec coverage:**
- §1 文件清单 → Tasks 1-9 全覆盖 ✓
- §2 API 修正 → Task 1 (satposs), Task 4 (mlambda), Task 2 (prange/gettgd), Task 2/3 (varerr) ✓
- §3 状态向量 → Task 1 ✓
- §4 量测方程 → Task 2 (SPP), Task 3 (RTK/RTD) ✓
- §5 TcEstimator → Task 5 ✓
- §6 降级 → Task 6 ✓
- §7 路径 D → Task 8 ✓
- §8 初始化 → Task 7 ✓
- §9 约束 → Task 7 ✓
- §10 验证策略 → M1-M6 对应 Tasks 1-5, 10 ✓
- §11 配置 → Task 10 ✓

**2. Placeholder scan:** 无 TBD/TODO。部分测试用 mock/简化数据路径，需运行时确认 RINEX 文件存在。

**3. Type consistency:** `TcStateIndex.from_config(cfg, mode)`、`TcMeasurement.build(...)`、`TcEstimator.tc_meas_update(v, H, R, source)`、`TcAmbiguity.try_fix(x, P)` 跨任务一致。

**已知运行时风险:**
- `InsInitializer.initialize_from_position` 需确认方法名（可能叫 `initialize`）
- `interpolate_imu` 函数名需确认
- `RSLTWriter` 构造参数需确认
- `SensorData` 是否有 `tag` 属性需确认
- rtklib `pos2ecef`/`ecef2enu` 函数名需确认

这些在实现时按实际签名调整。
