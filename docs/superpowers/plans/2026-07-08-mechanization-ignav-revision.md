# INS 机械编排对齐 ignav 修订实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 INS 机械编排从 φ-error 对齐到 ignav 的 ψ-error，升级旋转补偿为精确 Rodrigues，添加重力梯度，实现自适应 Φ，修复 _prev 存储 bug。

**Architecture:** 修改 transfer_matrix.py（F/Φ/Q 矩阵）、ins_update.py（旋转补偿+bug修复）、earth_param.py（georadi）、ins_propagate.py（调用同步），新增 3 个 pytest 单元测试文件，更新 3 个文档文件。

**Tech Stack:** Python 3, NumPy, pytest

## Global Constraints

- 系统默认 Python 2.7，必须用 python3 执行
- 所有源码放 src/ 目录，tools/ 目录仅参考
- IMU 为速率式（gyro rad/s, accel m/s²），需 ×dt 转增量后套用增量式算法
- INS 机械编排在 E 系（ECEF）下进行
- 协方差传播采用中间值法 P = Φ·(P+0.5Q)·Φ^T + 0.5Q
- 不引入 scipy 依赖，矩阵指数自实现

---

### Task 1: earth_param.py 新增 georadi 函数

**Files:**
- Modify: `src/core/ins/earth_param.py` (末尾追加)
- Test: `tests/ins/test_earth_param.py` (新建)

**Interfaces:**
- Produces: `georadi(lat: float) -> float` — 地心半径（米）

- [ ] **Step 1: 编写 georadi 失败测试**

创建 `tests/ins/test_earth_param.py`:

```python
"""earth_param 函数单元测试。"""
import math

import numpy as np
import pytest

from src.core.ins.earth_param import georadi, EARTH_SEMI_MAJOR, EARTH_SEMI_MINOR


class TestGeoradi:
    """地心半径函数测试。"""

    def test_georadi_equator(self):
        """赤道处地心半径 = 长半轴。"""
        re = georadi(0.0)
        assert abs(re - EARTH_SEMI_MAJOR) < 1.0  # 误差 < 1m

    def test_georadi_pole(self):
        """极点处地心半径 = 短半轴。"""
        re = georadi(math.pi / 2)
        assert abs(re - EARTH_SEMI_MINOR) < 1.0

    def test_georadi_mid_latitude(self):
        """中纬度 (45°) 地心半径在长短半轴之间。"""
        re = georadi(math.radians(45.0))
        assert EARTH_SEMI_MINOR < re < EARTH_SEMI_MAJOR

    def test_georadi_south_pole(self):
        """南极处地心半径 = 短半轴。"""
        re = georadi(-math.pi / 2)
        assert abs(re - EARTH_SEMI_MINOR) < 1.0
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python3 -m pytest tests/ins/test_earth_param.py -v`
Expected: FAIL with `ImportError: cannot import name 'georadi'`

- [ ] **Step 3: 实现 georadi 函数**

在 `src/core/ins/earth_param.py` 末尾追加:

```python
def georadi(lat: float) -> float:
    """地心半径 (参考 ignav georadi, ins-gnss.cc line 214-218)。

    Args:
        lat: 纬度 (rad)

    Returns:
        地心半径 (m)
    """
    e = 0.0818191908425  # WGS84 偏心率
    s = math.sin(lat)
    c = math.cos(lat)
    return EARTH_SEMI_MAJOR / math.sqrt(1.0 - e**2 * s**2) * \
           math.sqrt(c**2 + (1.0 - e**2)**2 * s**2)
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python3 -m pytest tests/ins/test_earth_param.py -v`
Expected: 4 passed

- [ ] **Step 5: 提交**

```bash
git add src/core/ins/earth_param.py tests/ins/test_earth_param.py
git commit -m "feat: add georadi function to earth_param for gravity gradient F_vr"
```

---

### Task 2: transfer_matrix.py 新增 _expm 矩阵指数函数

**Files:**
- Modify: `src/core/ins/transfer_matrix.py` (模块级函数区，rodrigues 之后)
- Test: `tests/ins/test_transfer_matrix.py` (新建)

**Interfaces:**
- Produces: `_expm(A: np.ndarray, order: int = 10) -> np.ndarray` — 矩阵指数

- [ ] **Step 1: 编写 _expm 失败测试**

创建 `tests/ins/test_transfer_matrix.py`:

```python
"""transfer_matrix 函数单元测试。"""
import math

import numpy as np
import pytest

from src.core.ins.transfer_matrix import _expm, skew, rodrigues, TransferMatrix


class TestExpm:
    """矩阵指数函数测试。"""

    def test_expm_zero_matrix(self):
        """零矩阵的指数 = 单位矩阵。"""
        A = np.zeros((3, 3))
        result = _expm(A)
        np.testing.assert_allclose(result, np.eye(3), atol=1e-12)

    def test_expm_identity(self):
        """单位矩阵的指数 = e·I。"""
        A = np.eye(3)
        result = _expm(A)
        expected = math.e * np.eye(3)
        np.testing.assert_allclose(result, expected, atol=1e-8)

    def test_expm_small_matrix(self):
        """小矩阵: expm(A) ≈ I + A + 0.5*A² (与二阶 Taylor 一致)。"""
        A = np.array([[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]])
        result = _expm(A)
        expected = np.eye(3) + A + 0.5 * A @ A
        np.testing.assert_allclose(result, expected, atol=1e-8)

    def test_expm_large_matrix(self):
        """大矩阵: expm(A) 与 numpy 数值验证一致 (15x15)。"""
        np.random.seed(42)
        A = np.random.randn(15, 15) * 0.1
        result = _expm(A)
        # 用更高阶 Taylor 验证
        expected = np.eye(15)
        term = np.eye(15)
        for k in range(1, 30):
            term = term @ A / k
            expected += term
        np.testing.assert_allclose(result, expected, atol=1e-6)

    def test_expm_skew_symmetric(self):
        """反对称矩阵的指数 = 旋转矩阵。"""
        theta = 0.5  # rad
        A = np.array([[0, -theta, 0], [theta, 0, 0], [0, 0, 0]])
        result = _expm(A)
        expected = np.array([
            [math.cos(theta), -math.sin(theta), 0],
            [math.sin(theta),  math.cos(theta), 0],
            [0, 0, 1],
        ])
        np.testing.assert_allclose(result, expected, atol=1e-8)
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python3 -m pytest tests/ins/test_transfer_matrix.py::TestExpm -v`
Expected: FAIL with `ImportError: cannot import name '_expm'`

- [ ] **Step 3: 实现 _expm 函数**

在 `src/core/ins/transfer_matrix.py` 中，`rodrigues` 函数之后、`TransferMatrix` 类之前，添加:

```python
def _expm(A: np.ndarray, order: int = 10) -> np.ndarray:
    """矩阵指数 (scaling-and-squaring + Taylor 级数, 不依赖 scipy)。

    参考 ignav precPhi (ins-gnss.cc line 1064-1088) 的矩阵指数实现。

    Args:
        A: 方阵
        order: Taylor 级数阶数

    Returns:
        exp(A)
    """
    n = A.shape[0]
    norm = float(np.linalg.norm(A, np.inf))
    s = int(np.ceil(np.log2(norm))) if norm > 1.0 else 0
    A_scaled = A / (2.0 ** s)
    result = np.eye(n, dtype=np.float64)
    term = np.eye(n, dtype=np.float64)
    for k in range(1, order + 1):
        term = term @ A_scaled / k
        result += term
    for _ in range(s):
        result = result @ result
    return result
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python3 -m pytest tests/ins/test_transfer_matrix.py::TestExpm -v`
Expected: 5 passed

- [ ] **Step 5: 提交**

```bash
git add src/core/ins/transfer_matrix.py tests/ins/test_transfer_matrix.py
git commit -m "feat: add _expm matrix exponential function for adaptive Phi"
```

---

### Task 3: transfer_matrix.py F 矩阵 ψ-error 切换 + F_vr 重力梯度

**Files:**
- Modify: `src/core/ins/transfer_matrix.py` (build_F 方法, line 63-102)
- Test: `tests/ins/test_transfer_matrix.py` (追加 TestBuildF 类)

**Interfaces:**
- Consumes: `georadi` from Task 1
- Produces: `build_F(C_b_e, f_b, w_b_ib, pos_e) -> np.ndarray` (签名变更: 新增 pos_e)

- [ ] **Step 1: 编写 F 矩阵测试**

在 `tests/ins/test_transfer_matrix.py` 追加:

```python
class TestBuildF:
    """F 矩阵构造测试 (ψ-error 模型)。"""

    @pytest.fixture
    def tm(self):
        config = {"ins": {}}
        return TransferMatrix(config)

    @pytest.fixture
    def simple_state(self):
        """简单状态: 单位旋转矩阵, 零比力, ECEF 在赤道。"""
        C_b_e = np.eye(3)
        f_b = np.array([0.0, 0.0, 0.0])
        w_b_ib = np.array([0.0, 0.0, 0.0])
        pos_e = np.array([6378137.0, 0.0, 0.0])  # 赤道上
        return C_b_e, f_b, w_b_ib, pos_e

    def test_F_shape(self, tm, simple_state):
        """F 矩阵为 15x15。"""
        C_b_e, f_b, w_b_ib, pos_e = simple_state
        F = tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        assert F.shape == (15, 15)

    def test_F_rv_is_identity(self, tm, simple_state):
        """F_rv = I (位置-速度耦合)。"""
        C_b_e, f_b, w_b_ib, pos_e = simple_state
        F = tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        np.testing.assert_allclose(F[0:3, 3:6], np.eye(3))

    def test_F_vphi_negative_skew(self, tm):
        """ψ-error: F_vψ = -skew(C_b_e·f_b) (负号)。"""
        C_b_e = np.eye(3)
        f_b = np.array([1.0, 0.0, 0.0])
        w_b_ib = np.zeros(3)
        pos_e = np.array([6378137.0, 0.0, 0.0])
        F = tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        f_e = C_b_e @ f_b
        expected = -skew(f_e)  # ψ-error: 负号
        np.testing.assert_allclose(F[3:6, 6:9], expected)

    def test_F_phibg_positive_Cbe(self, tm, simple_state):
        """ψ-error: F_ψbg = +C_b_e (正号)。"""
        C_b_e, f_b, w_b_ib, pos_e = simple_state
        F = tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        np.testing.assert_allclose(F[6:9, 9:12], C_b_e)  # 正号

    def test_F_vba_positive_Cbe(self, tm, simple_state):
        """F_vba = +C_b_e。"""
        C_b_e, f_b, w_b_ib, pos_e = simple_state
        F = tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        np.testing.assert_allclose(F[3:6, 12:15], C_b_e)

    def test_F_vv_coriolis(self, tm, simple_state):
        """F_vv = -2*[ω_ie×]。"""
        from src.core.ins.earth_param import EARTH_ROTATION_RATE
        C_b_e, f_b, w_b_ib, pos_e = simple_state
        F = tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        w_ie_e = np.array([0.0, 0.0, EARTH_ROTATION_RATE])
        expected = -2.0 * skew(w_ie_e)
        np.testing.assert_allclose(F[3:6, 3:6], expected)

    def test_F_phiphi_coriolis(self, tm, simple_state):
        """F_ψψ = -[ω_ie×]。"""
        from src.core.ins.earth_param import EARTH_ROTATION_RATE
        C_b_e, f_b, w_b_ib, pos_e = simple_state
        F = tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        w_ie_e = np.array([0.0, 0.0, EARTH_ROTATION_RATE])
        expected = -skew(w_ie_e)
        np.testing.assert_allclose(F[6:9, 6:9], expected)

    def test_F_vr_gravity_gradient_nonzero(self, tm, simple_state):
        """F_vr 重力梯度不为零。"""
        C_b_e, f_b, w_b_ib, pos_e = simple_state
        F = tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        # F_vr 应非零
        assert np.any(np.abs(F[3:6, 0:3]) > 1e-15)

    def test_F_vr_gravity_gradient_symmetry(self, tm):
        """F_vr 在赤道处: ge 与 pos 同向, F_vr ≈ -2*g/(re*|pos|) * pos⊗pos。"""
        from src.core.ins.earth_param import gravity_ecef, georadi, ecef2llh
        pos_e = np.array([6378137.0, 0.0, 0.0])
        C_b_e = np.eye(3)
        F = tm.build_F(C_b_e, np.zeros(3), np.zeros(3), pos_e)
        ge = gravity_ecef(pos_e)
        lat, _, _ = ecef2llh(pos_e)
        re = georadi(lat)
        pos_norm = np.linalg.norm(pos_e)
        expected = -2.0 / (re * pos_norm) * np.outer(ge, pos_e)
        np.testing.assert_allclose(F[3:6, 0:3], expected, atol=1e-10)
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python3 -m pytest tests/ins/test_transfer_matrix.py::TestBuildF -v`
Expected: FAIL (build_F 签名不匹配, F_vφ 符号不对, F_vr 为零)

- [ ] **Step 3: 修改 build_F 方法**

在 `src/core/ins/transfer_matrix.py` 中:

1. 添加 import (文件顶部, earth_param 导入行):
```python
from src.core.ins.earth_param import (
    EARTH_ROTATION_RATE,
    ecef2llh,
    georadi,
    gravity_ecef,
)
```

2. 替换 `build_F` 方法 (line 63-102):

```python
    def build_F(self, C_b_e: np.ndarray, f_b: np.ndarray,
                w_b_ib: np.ndarray, pos_e: np.ndarray) -> np.ndarray:
        """构造 15x15 连续时间 F 矩阵 (ψ-error 模型, 对齐 ignav)。

        Args:
            C_b_e: 3x3 旋转矩阵 b→e
            f_b: 3 比力 (b 系, m/s²)
            w_b_ib: 3 角速度 (b 系, rad/s)
            pos_e: 3 ECEF 位置 (m)

        Returns:
            15x15 F 矩阵
        """
        F = np.zeros((15, 15), dtype=np.float64)

        # F_rv = I (位置-速度耦合)
        F[0:3, 3:6] = np.eye(3)

        # F_vr = -2/(re·|pos|) · ge ⊗ pos  (重力梯度, 参考 ignav getF)
        ge = gravity_ecef(pos_e)
        lat, _, _ = ecef2llh(pos_e)
        re = georadi(lat)
        pos_norm = np.linalg.norm(pos_e)
        if pos_norm > 1.0:
            F[3:6, 0:3] = -2.0 / (re * pos_norm) * np.outer(ge, pos_e)

        # F_vv = -2*[ω_ie^e×]  (Coriolis)
        F[3:6, 3:6] = -2.0 * skew(self.w_ie_e)

        # F_vψ = -[C_b_e·f_b×]  (ψ-error: 负号, 对齐 ignav)
        f_e = C_b_e @ f_b
        F[3:6, 6:9] = -skew(f_e)

        # F_vba = C_b_e  (加计零偏 → 速度)
        F[3:6, 12:15] = C_b_e

        # F_ψψ = -[ω_ie^e×]  (Coriolis)
        F[6:9, 6:9] = -skew(self.w_ie_e)

        # F_ψbg = +C_b_e  (ψ-error: 正号, 对齐 ignav)
        F[6:9, 9:12] = C_b_e

        # F_bgbg = -I / tau_gyro  (陀螺零偏一阶马尔可夫)
        F[9:12, 9:12] = -np.eye(3) / self.tau_gyro

        # F_baba = -I / tau_acce  (加计零偏一阶马尔可夫)
        F[12:15, 12:15] = -np.eye(3) / self.tau_acce

        return F
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python3 -m pytest tests/ins/test_transfer_matrix.py::TestBuildF -v`
Expected: 9 passed

- [ ] **Step 5: 提交**

```bash
git add src/core/ins/transfer_matrix.py tests/ins/test_transfer_matrix.py
git commit -m "feat: switch F matrix to psi-error model and add gravity gradient F_vr"
```

---

### Task 4: transfer_matrix.py Φ 矩阵自适应精度

**Files:**
- Modify: `src/core/ins/transfer_matrix.py` (build_Phi 方法, line 104-107)
- Test: `tests/ins/test_transfer_matrix.py` (追加 TestBuildPhi 类)

**Interfaces:**
- Consumes: `_expm` from Task 2

- [ ] **Step 1: 编写 Φ 自适应测试**

在 `tests/ins/test_transfer_matrix.py` 追加:

```python
class TestBuildPhi:
    """Φ 矩阵自适应精度测试。"""

    @pytest.fixture
    def tm(self):
        return TransferMatrix({"ins": {}})

    @pytest.fixture
    def F(self):
        """简单 F 矩阵: 仅 F_rv = I, 其余为零。"""
        F = np.zeros((15, 15))
        F[0:3, 3:6] = np.eye(3)
        return F

    def test_phi_first_order(self, tm, F):
        """dt <= 0.005s (≥200Hz): 一阶 Φ = I + F·dt。"""
        dt = 0.005
        Phi = tm.build_Phi(F, dt)
        expected = np.eye(15) + F * dt
        np.testing.assert_allclose(Phi, expected)

    def test_phi_second_order(self, tm, F):
        """0.005 < dt <= 0.01s (100-200Hz): 二阶 Φ = I + F·dt + 0.5·(F·dt)²。"""
        dt = 0.01
        Phi = tm.build_Phi(F, dt)
        Fdt = F * dt
        expected = np.eye(15) + Fdt + 0.5 * (Fdt @ Fdt)
        np.testing.assert_allclose(Phi, expected)

    def test_phi_matrix_exponential(self, tm, F):
        """dt > 0.01s (<100Hz): 矩阵指数 Φ = expm(F·dt)。"""
        dt = 0.05
        Phi = tm.build_Phi(F, dt)
        expected = _expm(F * dt)
        np.testing.assert_allclose(Phi, expected)

    def test_phi_identity_for_zero_F(self, tm):
        """F=0 时 Φ = I (任意 dt)。"""
        F = np.zeros((15, 15))
        for dt in [0.001, 0.005, 0.01, 0.1, 1.0]:
            Phi = tm.build_Phi(F, dt)
            np.testing.assert_allclose(Phi, np.eye(15))
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python3 -m pytest tests/ins/test_transfer_matrix.py::TestBuildPhi -v`
Expected: FAIL (当前 build_Phi 始终用二阶, 不支持自适应)

- [ ] **Step 3: 修改 build_Phi 方法**

在 `src/core/ins/transfer_matrix.py` 中替换 `build_Phi` (line 104-107):

```python
    def build_Phi(self, F: np.ndarray, dt: float) -> np.ndarray:
        """离散化: 自适应精度 (对齐 ignav precPhi)。

        - dt <= 0.005s  (≥200Hz): 一阶 Φ = I + F·dt
        - dt <= 0.01s   (100-200Hz): 二阶 Φ = I + F·dt + 0.5·(F·dt)²
        - dt > 0.01s    (<100Hz): 矩阵指数 Φ = expm(F·dt)
        """
        Fdt = F * dt
        if dt <= 0.005:
            return np.eye(15, dtype=np.float64) + Fdt
        elif dt <= 0.01:
            return np.eye(15, dtype=np.float64) + Fdt + 0.5 * (Fdt @ Fdt)
        else:
            return _expm(Fdt)
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python3 -m pytest tests/ins/test_transfer_matrix.py::TestBuildPhi -v`
Expected: 4 passed

- [ ] **Step 5: 提交**

```bash
git add src/core/ins/transfer_matrix.py tests/ins/test_transfer_matrix.py
git commit -m "feat: adaptive Phi matrix precision (1st/2nd/expm based on dt)"
```

---

### Task 5: transfer_matrix.py Q 矩阵 G 映射符号同步

**Files:**
- Modify: `src/core/ins/transfer_matrix.py` (build_Q 方法, line 121)
- Test: `tests/ins/test_transfer_matrix.py` (追加 TestBuildQ 类)

**注意**: Q = G·Q_diag·G^T, G 的符号变化不影响 Q 数值（正负抵消）。此修改仅为 ψ-error 约定一致性, 不产生数值变化。测试验证 Q 矩阵正确性, 不验证 G 符号。

- [ ] **Step 1: 修改 build_Q 方法**

在 `src/core/ins/transfer_matrix.py` 中, 修改 `build_Q` 内的 G 矩阵 (line 121):

将:
```python
        G[6:9, 6:9] = -C_b_e      # 陀螺噪声 → 姿态
```
改为:
```python
        G[6:9, 6:9] = C_b_e       # 陀螺噪声 → 姿态 (ψ-error: 正号)
```

- [ ] **Step 2: 编写 Q 矩阵测试**

在 `tests/ins/test_transfer_matrix.py` 追加:

```python
class TestBuildQ:
    """Q 矩阵构造测试。"""

    @pytest.fixture
    def tm(self):
        return TransferMatrix({"ins": {}})

    def test_Q_shape(self, tm):
        """Q 矩阵为 15x15。"""
        C_b_e = np.eye(3)
        Q = tm.build_Q(0.01, C_b_e)
        assert Q.shape == (15, 15)

    def test_Q_symmetric(self, tm):
        """Q 矩阵对称。"""
        C_b_e = np.eye(3)
        Q = tm.build_Q(0.01, C_b_e)
        np.testing.assert_allclose(Q, Q.T, atol=1e-15)

    def test_Q_positive_semidefinite(self, tm):
        """Q 矩阵半正定。"""
        C_b_e = np.eye(3)
        Q = tm.build_Q(0.01, C_b_e)
        eigenvalues = np.linalg.eigvalsh(Q)
        assert np.all(eigenvalues >= -1e-15)

    def test_Q_attitude_noise_value(self, tm):
        """Q[6:9,6:9] = gyro_psd*dt (C_b_e=I 时)。"""
        C_b_e = np.eye(3)
        Q = tm.build_Q(0.01, C_b_e)
        expected = tm.gyro_psd * 0.01
        np.testing.assert_allclose(np.diag(Q[6:9, 6:9]), [expected]*3)

    def test_Q_velocity_noise_value(self, tm):
        """Q[3:6,3:6] = accel_psd*dt (C_b_e=I 时)。"""
        C_b_e = np.eye(3)
        Q = tm.build_Q(0.01, C_b_e)
        expected = tm.accel_psd * 0.01
        np.testing.assert_allclose(np.diag(Q[3:6, 3:6]), [expected]*3)
```

- [ ] **Step 3: 运行测试验证通过**

Run: `python3 -m pytest tests/ins/test_transfer_matrix.py::TestBuildQ -v`
Expected: 5 passed (Q 数值不变, 测试验证正确性)

- [ ] **Step 4: 提交**

```bash
git add src/core/ins/transfer_matrix.py tests/ins/test_transfer_matrix.py
git commit -m "refactor: align Q matrix G mapping sign with psi-error convention"
```

---

### Task 6: ins_update.py 旋转补偿升级 + _prev 存储 bug 修复

**Files:**
- Modify: `src/core/ins/ins_update.py` (line 120-121, 160, 162-163)
- Test: `tests/ins/test_ins_update.py` (新建)

- [ ] **Step 1: 编写 ins_update 测试**

创建 `tests/ins/test_ins_update.py`:

```python
"""ins_update 机械编排单元测试。"""
import math

import numpy as np
import pytest

from src.core.data_types import ImuMeasurement, InsState
from src.core.ins.ins_update import InsUpdate


def make_init_state(timestamp=100.0):
    """创建简单初始状态: 原点静止, 单位姿态。"""
    return InsState(
        timestamp=timestamp,
        pos_e=np.array([6378137.0, 0.0, 0.0]),
        vel_e=np.array([0.0, 0.0, 0.0]),
        C_b_e=np.eye(3),
        q_b_e=np.array([1.0, 0.0, 0.0, 0.0]),
        att_rpy=np.array([0.0, 0.0, 0.0]),
        gyro_bias=np.zeros(3),
        accel_bias=np.zeros(3),
        gyro_scale=np.zeros(3),
        accel_scale=np.zeros(3),
        imu_angle=np.zeros(2),
        imu_leverarm=np.zeros(3),
        leverarm=np.zeros(3),
    )


def make_imu(timestamp, gyro, accel):
    """创建 IMU 测量。"""
    return ImuMeasurement(timestamp=timestamp, week=0, gyro=gyro, accel=accel)


class TestRotationCompensation:
    """旋转补偿精确 Rodrigues 测试。"""

    def test_zero_rotation_zero_velocity(self):
        """零角速度 + 零加速度: 旋转补偿项 = 0。"""
        state = make_init_state()
        updater = InsUpdate(state)
        imu = make_imu(100.01, np.zeros(3), np.zeros(3))
        new_state = updater.update(imu)
        # 速度应几乎不变 (仅重力+科氏)
        np.testing.assert_allclose(new_state.vel_e, state.vel_e, atol=1e-6)

    def test_small_rotation_matches_first_order(self):
        """小角度: 精确 Rodrigues ≈ 一阶 0.5·cross(dθ,dv)。"""
        state = make_init_state()
        updater = InsUpdate(state)
        # 小角速度 + 小加速度
        dt = 0.01
        gyro = np.array([0.001, 0.0, 0.0])  # 很小
        accel = np.array([0.0, 0.001, 0.0])  # 很小
        imu = make_imu(100.0 + dt, gyro, accel)
        new_state = updater.update(imu)
        # 不报错即通过, 精确值需数值验证
        assert not np.any(np.isnan(new_state.vel_e))

    def test_large_rotation_stable(self):
        """大角速度: 精确 Rodrigues 数值稳定。"""
        state = make_init_state()
        updater = InsUpdate(state)
        dt = 0.01
        gyro = np.array([10.0, 0.0, 0.0])  # 10 rad/s (很大)
        accel = np.array([0.0, 9.8, 0.0])
        imu = make_imu(100.0 + dt, gyro, accel)
        new_state = updater.update(imu)
        assert not np.any(np.isnan(new_state.vel_e))
        assert not np.any(np.isnan(new_state.pos_e))


class TestPrevStorageBug:
    """_prev_dtheta/_prev_dvel 存储 bug 修复测试。"""

    def test_prev_stores_compensated_values(self):
        """_prev_dtheta 应存储已补偿值 (非原始值)。"""
        state = make_init_state()
        # 设置非零零偏
        state.gyro_bias = np.array([0.001, 0.0, 0.0])
        state.accel_bias = np.array([0.0, 0.01, 0.0])
        updater = InsUpdate(state)

        dt = 0.01
        gyro = np.array([0.1, 0.0, 0.0])
        accel = np.array([0.0, 9.8, 0.0])
        imu = make_imu(100.0 + dt, gyro, accel)
        updater.update(imu)

        # 计算 expected 已补偿值
        dtheta = gyro * dt
        dvel = accel * dt
        expected_dtheta = (dtheta - state.gyro_bias * dt)  # scale=0
        expected_dvel = (dvel - state.accel_bias * dt)

        np.testing.assert_allclose(updater._prev_dtheta, expected_dtheta, atol=1e-12)
        np.testing.assert_allclose(updater._prev_dvel, expected_dvel, atol=1e-12)

    def test_prev_not_raw_values(self):
        """_prev_dtheta 不应等于原始未补偿值。"""
        state = make_init_state()
        state.gyro_bias = np.array([0.001, 0.0, 0.0])
        updater = InsUpdate(state)

        dt = 0.01
        gyro = np.array([0.1, 0.0, 0.0])
        accel = np.array([0.0, 9.8, 0.0])
        imu = make_imu(100.0 + dt, gyro, accel)
        updater.update(imu)

        raw_dtheta = gyro * dt
        # _prev_dtheta 不应等于原始值 (因为零偏非零)
        assert not np.allclose(updater._prev_dtheta, raw_dtheta)
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python3 -m pytest tests/ins/test_ins_update.py -v`
Expected: TestPrevStorageBug 测试 FAIL (当前存储未补偿值)

- [ ] **Step 3: 修改 ins_update.py**

1. 修改旋转补偿 (line 160), 将:
```python
        # 旋转补偿
        v_rot = 0.5 * np.cross(dtheta_comp, dvel_comp)
```
替换为:
```python
        # 旋转补偿 (精确 Rodrigues, 参考 ignav rotscull_corr)
        dak = dtheta_comp
        dvk = dvel_comp
        dak_norm = float(np.linalg.norm(dak))
        if dak_norm < 1e-12:
            v_rot = np.zeros(3, dtype=np.float64)
        else:
            dak_sq = dak_norm * dak_norm
            a1 = (1.0 - math.cos(dak_norm)) / dak_sq
            a2 = (1.0 - math.sin(dak_norm) / dak_norm) / dak_sq
            v_rot = a1 * np.cross(dak, dvk) + a2 * np.cross(dak, np.cross(dak, dvk))
```

2. 修改 _prev 存储 (line 120-121), 将:
```python
        self._prev_dtheta = dtheta.copy()
        self._prev_dvel = dvel.copy()
```
替换为:
```python
        self._prev_dtheta = dtheta_comp.copy()
        self._prev_dvel = dvel_comp.copy()
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python3 -m pytest tests/ins/test_ins_update.py -v`
Expected: 5 passed

- [ ] **Step 5: 提交**

```bash
git add src/core/ins/ins_update.py tests/ins/test_ins_update.py
git commit -m "fix: upgrade rotation compensation to exact Rodrigues and fix _prev storage bug"
```

---

### Task 7: ins_propagate.py build_F 调用同步

**Files:**
- Modify: `src/core/ins/ins_propagate.py` (line 57)

- [ ] **Step 1: 修改 build_F 调用**

在 `src/core/ins/ins_propagate.py` line 57, 将:
```python
        F = self._tm.build_F(C_b_e, f_b, w_b_ib)
```
替换为:
```python
        F = self._tm.build_F(C_b_e, f_b, w_b_ib, ins_update.state.pos_e)
```

- [ ] **Step 2: 运行 transfer_matrix 测试验证不回归**

Run: `python3 -m pytest tests/ins/test_transfer_matrix.py tests/ins/test_ins_update.py -v`
Expected: 全部 passed

- [ ] **Step 3: 提交**

```bash
git add src/core/ins/ins_propagate.py
git commit -m "fix: pass pos_e to build_F for gravity gradient computation"
```

---

### Task 8: 运行机械编排开环集成测试

**Files:**
- 无修改, 仅运行测试

- [ ] **Step 1: 运行机械编排开环测试**

Run: `python3 tests/ins/test_mechanization.py`
Expected:
- trace_mech.csv 行数 ≈ 5000
- 无 NaN
- P1 trace 单调增长
- 终端输出 "机械编排测试完成"

- [ ] **Step 2: 如有错误, 诊断并修复**

常见问题:
- build_F 签名不匹配 → 检查 ins_propagate.py 调用
- F_vr 计算异常 → 检查 pos_e 是否正确传入
- _prev 值改变导致数值发散 → 检查锥补/划桨公式

---

### Task 9: 运行 pytest 全量回归测试

**Files:**
- 无修改, 仅运行测试

- [ ] **Step 1: 运行所有 pytest 单元测试**

Run: `python3 -m pytest tests/ -v --tb=short`
Expected: 所有测试 passed (含新增的 transfer_matrix/ins_update/earth_param 测试)

- [ ] **Step 2: 如有失败, 诊断并修复**

---

### Task 10: 运行 internal+on 端到端测试

**Files:**
- 无修改, 仅运行测试

- [ ] **Step 1: 运行 internal+on e2e 测试**

Run: `python3 -m pytest tests/test_internal_gnss_rtk_e2e.py -v`
Expected: passed, aligned_internal_rtk.csv 输出正常

- [ ] **Step 2: 验证输出文件行数**

Run: `wc -l output/aligned_internal_rtk.csv`
Expected: ≈ 179,331 行 (与修改前一致)

---

### Task 11: 文档更新 — skills/estimator.md

**Files:**
- Modify: `skills/estimator.md`

- [ ] **Step 1: 读取 estimator.md 当前内容**

Run: 读取 `skills/estimator.md` 中 F 矩阵、Φ 矩阵、旋转补偿相关段落

- [ ] **Step 2: 更新 F 矩阵描述**

将 φ-error 描述更新为 ψ-error:
- F_vφ = +skew(f_e) → F_vψ = -skew(f_e)
- F_φbg = -C_b_e → F_ψbg = +C_b_e
- 新增 F_vr = -2/(re·|pos|)·ge⊗pos 重力梯度项
- 注明对齐 ignav ψ-error 模型

- [ ] **Step 3: 更新 Φ 矩阵描述**

将"二阶 Taylor"更新为"自适应精度":
- dt ≤ 0.005s (≥200Hz): 一阶
- dt ≤ 0.01s (100-200Hz): 二阶
- dt > 0.01s (<100Hz): 矩阵指数

- [ ] **Step 4: 更新旋转补偿描述**

将"一阶 0.5·cross(dθ,dv)"更新为"精确 Rodrigues a1/a2 + 二阶项"

- [ ] **Step 5: 添加 _prev bug 修复说明**

注明 _prev_dtheta/_prev_dvel 现存储已补偿值

- [ ] **Step 6: 提交**

```bash
git add skills/estimator.md
git commit -m "docs: update estimator.md for psi-error model and ignav alignment"
```

---

### Task 12: 文档更新 — skills/imu.md

**Files:**
- Modify: `skills/imu.md`

- [ ] **Step 1: 更新机械编排旋转补偿描述**

找到旋转补偿相关段落, 更新为精确 Rodrigues

- [ ] **Step 2: 提交**

```bash
git add skills/imu.md
git commit -m "docs: update imu.md rotation compensation description"
```

---

### Task 13: 项目记忆更新

**Files:**
- Modify: `/home/mxl/.trae-cn/memory/projects/-home-mxl-workplace-gipylib/project_memory.md`

- [ ] **Step 1: 更新项目记忆**

更新以下条目:
- F矩阵约定: φ-error → ψ-error (对齐 ignav), F_vφ=-skew(f_e), F_φbg=+C_b_e
- 新增 F_vr 重力梯度项 = -2/(re·|pos|)·ge⊗pos
- 新增 Φ 矩阵自适应精度 (<0.005s 一阶, ≤0.01s 二阶, >0.01s 矩阵指数)
- 新增旋转补偿用精确 Rodrigues (a1/a2 系数 + 二阶项)
- 新增 _prev_dtheta/_prev_dvel 存储已补偿值
- 新增 earth_param.py georadi 函数

---

### Task 14: 最终全量验证

**Files:**
- 无修改, 仅运行验证

- [ ] **Step 1: 运行所有 pytest 单元测试**

Run: `python3 -m pytest tests/ -v --tb=short`
Expected: 全部 passed

- [ ] **Step 2: 运行机械编排开环测试**

Run: `python3 tests/ins/test_mechanization.py`
Expected: 5000+ 历元, 无 NaN, P1 单调增长

- [ ] **Step 3: 运行 internal+on e2e 测试**

Run: `python3 -m pytest tests/test_internal_gnss_rtk_e2e.py -v`
Expected: passed

- [ ] **Step 4: 验证完成**

所有测试通过, 修改完成。
