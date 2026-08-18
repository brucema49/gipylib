# IMU-GNSS Fusion Timing Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make LC and TC emit each GNSS measurement update at the interpolated GNSS timestamp with Qins=3, without changing filter mathematics or degrading the configured TC replay accuracy.

**Architecture:** Keep the existing pending-GNSS queues and linear rate-IMU interpolation. Add an explicit post-update write at the interpolated state, then continue the same current IMU to its original timestamp and write the normal propagation state. Apply the same event ordering in both integrations and cover it with writer-capture tests plus a configured replay comparison.

**Tech Stack:** Python, NumPy, pytest, Matplotlib evaluation script.

---

### Task 1: Establish the regression tests

**Files:**
- Modify: `tests/ins/test_lc_integration.py`
- Create: `tests/tc/test_tc_integration.py`

- [x] **Step 1: Add a writer capture and failing LC assertion**

Capture every state write as `(timestamp, qins)` and add a test with IMUs at `1.00` and `1.01` and GNSS at `1.006`. Assert that the captured sequence contains `(1.006, 3)` before `(1.01, 2)`. The current implementation fails because it only writes the later IMU with Qins=3.

- [x] **Step 2: Run the focused LC test and verify the expected failure**

Run: `pytest -q tests/ins/test_lc_integration.py -k interpolated_update_timestamp`

Expected: FAIL because no Qins=3 write exists at `1.006`.

- [x] **Step 3: Add the analogous failing TC assertion**

Use a minimal fake raw observation timestamp at `1.006`, IMUs at `1.00/1.01`, and a fake TC measurement hook/writer. Assert the post-measurement state is written at `1.006` with Qins=3 and the continuation at `1.01` is Qins=2.

- [x] **Step 4: Run the focused TC test and verify the expected failure**

Run: `pytest -q tests/tc/test_tc_integration.py -k interpolated_update_timestamp`

Expected: FAIL for the same timestamp/marker mismatch.

### Task 2: Fix LC event output ordering

**Files:**
- Modify: `src/core/ins/lc_integration.py`
- Test: `tests/ins/test_lc_integration.py`

- [x] **Step 1: Change the interpolation branch to write only pre-update state if required by existing output semantics**

When `imu_interpolate_linear(cur, imu, t_gnss)` succeeds, preserve `time_update(interp)` and constraints, but do not let the later `last_qins=3` flag leak into the final raw-IMU write.

- [x] **Step 2: Write the fused state immediately after `_apply_gnss_update(gnss)`**

Call the existing state writer with `self.imucur` at `t_gnss` and `qins=3`, then reset the per-IMU marker to propagation mode before continuing to the raw `imu` update. Preserve the existing direct/expired GNSS branch by writing its fused state at the GNSS timestamp as well.

- [x] **Step 3: Run focused LC tests**

Run: `pytest -q tests/ins/test_lc_integration.py tests/ins/test_lc_e2e.py`

Expected: PASS, including the new timestamp assertion and all existing propagation/update tests.

### Task 3: Fix TC event output ordering

**Files:**
- Modify: `src/core/tc/tc_integration.py`
- Test: `tests/tc/test_tc_integration.py`

- [x] **Step 1: Remove the pre-measurement Qins=3 leakage**

In the interpolation branch, keep the interpolated time update and any existing diagnostic write, but ensure it is not labeled as the completed GNSS update.

- [x] **Step 2: Write the fused state immediately after `_trigger_meas(...)`**

Emit the current estimator state at `t_gnss` with `qins=3`, pop exactly that pending observation, set the integration cursor to the interpolated IMU, and continue to the current raw IMU. Reset `last_qins` to 2 before the continuation write unless a constraint update changes it.

- [x] **Step 3: Run focused TC tests**

Run: `pytest -q tests/tc tests/ins/test_lc_integration.py`

Expected: PASS for timestamp, queue, measurement, degradation, and existing TC regression tests.

### Task 4: Validate interpolation boundaries and output invariants

**Files:**
- Modify: `tests/ins/test_lc_integration.py`
- Modify: `tests/tc/test_tc_integration.py`
- Optionally modify: `tests/ins/test_interpolator.py` if present

- [x] **Step 1: Add boundary cases**

Cover GNSS exactly at `cur.timestamp`, GNSS later than current IMU (remains pending), expired GNSS, and two pending GNSS observations in one IMU interval. Assert each GNSS is consumed once and each Qins=3 timestamp is monotonic and equal to the corresponding GNSS timestamp.

- [x] **Step 2: Run all relevant unit tests**

Run: `pytest -q tests/ins tests/tc`

Expected: no new failures attributable to this change.

### Task 5: Baseline, replay, and accuracy comparison

**Files:**
- Read/execute: `data/rtk-ins紧组合.yaml`
- Read/execute: `data/plot/error-rslt(1).py`
- Generate: configured output `.rslt` and evaluation plot

- [x] **Step 1: Record baseline metrics from the current configured result**

Copy the current configured `.rslt` to a temporary baseline path, run the evaluator, and record Qins=3 count plus E/N/U/horizontal/3D RMSE, max, and 95th percentile.

- [x] **Step 2: Run the configured TC replay**

Run: `python src/main.py data/rtk-ins紧组合.yaml`

Expected: exit code 0 and the configured `rslt_filename` is regenerated.

- [x] **Step 3: Verify Qins=3 timestamps**

Parse the regenerated result and assert each Qins=3 epoch is within `1e-3 s` of the corresponding integer-second GNSS epoch; assert the marker is no longer systematically `+0.006 s`.

- [x] **Step 4: Run the evaluator and compare metrics**

Run: `python 'data/plot/error-rslt(1).py'` against the regenerated result using its configured/default window. Compare all recorded metrics; no metric may increase by more than 1% relative to the baseline.

- [x] **Step 5: Run syntax and focused/full regression checks**

Run: `python -m py_compile src/core/ins/lc_integration.py src/core/tc/tc_integration.py && pytest -q tests/ins tests/tc`

Record any pre-existing unrelated failures separately.
