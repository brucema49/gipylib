# Attitude Normalization Mechanism Comparison Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Match ignav's per-IMU quaternion normalization path and quantify whether it reduces gipylib's fixed-window mechanization difference from ignav.

**Architecture:** `InsUpdate` will reconstruct the propagated `C_b_e` from a normalized quaternion immediately after the ECEF/body rotations, while preserving the existing SVD projection during EKF feedback. `mechanism-diff.py` will parse the two native `.rslt` formats, restrict both to GPS week 2046 and SOW 359000--359100, match timestamps, and report/plot position and velocity differences without truth interpolation.

**Tech Stack:** Python 3, NumPy, pytest, Matplotlib, existing `data/plot/error-rslt.py` conventions.

---

### Task 1: Preserve the pre-normalization comparison baseline

**Files:**
- Read: `data/rtktc.rslt`
- Read: `data/output/RTKINS.rslt`
- Create: `data/problem/mechanism-diff-before-normalization.png`
- Create: `data/problem/mechanism-diff-before-normalization.txt`

- [ ] **Step 1: Run the comparison with the existing output**

Run after the comparison script exists but before changing `InsUpdate`:

```bash
python3 data/plot/mechanism-diff.py \
  --reference data/rtktc.rslt \
  --evaluation data/output/RTKINS.rslt \
  --output-prefix data/problem/mechanism-diff-before-normalization
```

Expected: exactly 10,000 timestamp-matched samples in GPS week 2046, SOW 359000--359100, with position and velocity difference metrics written alongside a PNG plot.

### Task 2: Add the fixed-window comparison utility

**Files:**
- Create: `data/plot/mechanism-diff.py`
- Test: `tests/plot/test_mechanism_diff.py`

- [ ] **Step 1: Write a failing parser/matching test**

Create temporary native-format `.rslt` fixtures with matching timestamps and assert that `read_rslt()` returns ECEF position, ECEF velocity, and Qins fields and that `match_by_gps_time()` retains only common timestamps.

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest tests/plot/test_mechanism_diff.py -q
```

Expected: FAIL because `mechanism_diff` cannot yet be imported.

- [ ] **Step 3: Implement the minimal parser and comparison**

Implement native `.rslt` parsing by skipping `%` lines, reading columns `[week, sow, x, y, z, Q, Qins, ns, ..., vx, vy, vz]`, filtering the fixed window, matching rounded millisecond GPS timestamps, transforming position and velocity differences into ENU at the first reference position, and writing one PNG plus one text report.

- [ ] **Step 4: Run the test to verify it passes**

Run:

```bash
python3 -m pytest tests/plot/test_mechanism_diff.py -q
```

Expected: PASS.

### Task 3: Normalize propagated attitude through the ignav-equivalent quaternion path

**Files:**
- Modify: `src/core/ins/attitude.py`
- Modify: `src/core/ins/ins_update.py`
- Modify: `tests/ins/test_ins_update.py`

- [ ] **Step 1: Write a failing propagation test**

Add a test that injects a slightly non-orthogonal `C_b_e` into an `InsState`, performs one valid IMU update, and asserts that the resulting matrix has `C_b_e @ C_b_e.T == I` and `det(C_b_e) == 1` within `1e-12`.

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
python3 -m pytest tests/ins/test_ins_update.py::TestRotationCompensation::test_attitude_update_reconstructs_normalized_quaternion -q
```

Expected: FAIL because the current matrix multiplication preserves the injected orthogonality error.

- [ ] **Step 3: Implement the minimal normalization path**

Add a `normalize_quaternion()` helper that rejects zero-norm input and returns a unit quaternion. In `_attitude_update()`, compute the existing matrix propagation, convert it with `dcm2quat()`, normalize it, and return `quat2dcm()` of that quaternion. Keep the feedback SVD projection unchanged because it handles a separate measurement-feedback boundary.

- [ ] **Step 4: Run the focused tests to verify they pass**

Run:

```bash
python3 -m pytest tests/ins/test_ins_update.py tests/ins/test_lc_estimator.py tests/tc/test_tc_estimator.py -q
```

Expected: PASS.

### Task 4: Measure the single-variable effect

**Files:**
- Modify: `data/output/RTKINS.rslt` (generated result)
- Create: `data/problem/mechanism-diff-after-normalization.png`
- Create: `data/problem/mechanism-diff-after-normalization.txt`

- [ ] **Step 1: Run the standard replay**

Run:

```bash
python3 src/main.py data/rtk-ins紧组合.yaml
```

Wait until `data/output/RTKINS.rslt` stops growing and has 171585 lines.

- [ ] **Step 2: Compare normalized output to ignav**

Run:

```bash
python3 data/plot/mechanism-diff.py \
  --reference data/rtktc.rslt \
  --evaluation data/output/RTKINS.rslt \
  --output-prefix data/problem/mechanism-diff-after-normalization
python3 data/plot/error-rslt.py
```

Expected: the comparison report states before/after metrics externally; the truth evaluation remains no worse than 3D RMSE 0.534 m, 3D Max 0.663 m, and horizontal RMSE 0.213 m.

### Task 5: Record verification evidence

**Files:**
- Modify: `issue/8-17机械编排优化.md`

- [ ] **Step 1: Append the timestamped before/after mechanism metrics**

Record the exact report metrics, the output line-count stability check, focused test output, and whether normalization improved the ignav comparison. Do not characterize it as a root cause unless the measured change supports that conclusion.

