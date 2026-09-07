# Zero-Bias Comparison Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute the 9-7 zero-bias comparison with opt-in propagation/update diagnostics and a configurable 36 s/360 s Gauss-Markov candidate, while proving that the default RTK-TC accuracy does not regress.

**Architecture:** Keep the existing filter equations and default configuration path unchanged. Parameterize only the two bias correlation times in `TransferMatrix`; capture immutable 15×15 INS propagation snapshots in `TcEstimator`; emit opt-in JSONL snapshots from a focused writer connected at the existing IMU and GNSS boundaries. Existing CSV/stat output remains backward-compatible.

**Tech Stack:** Python 3.12, NumPy, PyYAML, pytest, JSONL diagnostics, existing `data/plot/error-rslt.py` evaluation functions.

---

## Files and responsibilities

- Create: `src/log/tc_matrix_diagnostics.py` — opt-in JSONL writer; serializes only copied 15×15 matrices and scalar/event metadata.
- Modify: `src/core/ins/transfer_matrix.py` — read and validate optional bias correlation times, defaulting to the current 36 s.
- Modify: `src/core/tc/tc_estimator.py` — retain the latest propagation snapshot and expose it without changing the propagation calculation.
- Modify: `src/core/tc/tc_integration.py` — create/close the writer, record the first ten valid IMU propagations, and record every GNSS update boundary including rejected updates.
- Modify: `src/core/tc/tc_measurement.py` — add attempted/rejected phase/code counts and preserve the existing accepted pair/reference metadata in `info`.
- Create: `tests/log/test_tc_matrix_diagnostics.py` — writer schema, copying, shape, and disabled-output tests.
- Modify: `tests/ins/test_transfer_matrix.py` — default/custom/invalid correlation-time tests.
- Modify: `tests/tc/test_tc_integration.py` or a focused integration test file — verify the estimator snapshot is captured without changing state/P behavior.
- Create: `tools/issue_9_7_compare.py` — deterministic report helper for baseline/candidate `.rslt`/`.stat`/JSONL metrics; it must not feed truth data into the filter.
- Modify: `issue/9-7零偏估计的仔细对比.md` — append dated execution evidence, commands, metrics, and accept/reject decision only after fresh runs.

No existing user-modified file outside this list may be changed. Do not alter `feedback_pos_enable`, `armode`, `eratio`, `vel_var_floor`, IMU calibration values, or GNSS thresholds.

### Task 1: Add failing tests for correlation-time configuration

**Files:**
- Modify: `tests/ins/test_transfer_matrix.py`
- Modify: `src/core/ins/transfer_matrix.py` only after the red test is observed

- [ ] **Step 1: Write the failing tests**

Append to `TestBuildF`:

```python
    def test_bias_correlation_time_defaults_to_current_36_seconds(self):
        tm = TransferMatrix({"ins": {}})
        assert tm.tau_gyro == pytest.approx(36.0)
        assert tm.tau_acce == pytest.approx(36.0)

    def test_bias_correlation_time_is_configurable_per_sensor(self):
        tm = TransferMatrix({"ins": {
            "gyro_bias_corr_time_s": 360.0,
            "acce_bias_corr_time_s": 180.0,
        }})
        assert tm.tau_gyro == pytest.approx(360.0)
        assert tm.tau_acce == pytest.approx(180.0)

    @pytest.mark.parametrize("key", [
        "gyro_bias_corr_time_s", "acce_bias_corr_time_s",
    ])
    def test_bias_correlation_time_rejects_nonpositive_or_nonfinite(self, key):
        for value in (0.0, -1.0, float("nan"), float("inf")):
            with pytest.raises(ValueError, match="corr_time"):
                TransferMatrix({"ins": {key: value}})
```

- [ ] **Step 2: Run the focused tests and verify the intended failure**

Run:

```powershell
python -m pytest tests/ins/test_transfer_matrix.py -k bias_correlation_time -v
```

Expected: the default assertion fails because the current implementation does not expose validated configurable values; the test must not fail because of collection or a syntax error.

- [ ] **Step 3: Implement the minimal configuration read/validation**

In `TransferMatrix.__init__`, replace the two fixed assignments with:

```python
        self.tau_gyro = self._read_bias_corr_time(
            ins_cfg, "gyro_bias_corr_time_s")
        self.tau_acce = self._read_bias_corr_time(
            ins_cfg, "acce_bias_corr_time_s")
```

Add this static method before `build_F`:

```python
    @staticmethod
    def _read_bias_corr_time(ins_cfg: dict, key: str) -> float:
        value = float(ins_cfg.get(key, _CORR_TIME_BIAS_H * 3600.0))
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{key} must be a finite positive number")
        return value
```

Keep `F[9:12, 9:12] = -I / self.tau_gyro` and `F[12:15, 12:15] = -I / self.tau_acce` unchanged.

- [ ] **Step 4: Run the focused tests and the existing transfer-matrix tests**

Run:

```powershell
python -m pytest tests/ins/test_transfer_matrix.py -k "bias_correlation_time or TestBuildF" -v
```

Expected: all selected tests pass and no existing `TestBuildF` assertion changes.

- [ ] **Step 5: Commit the isolated code/test change**

```powershell
git add -- src/core/ins/transfer_matrix.py tests/ins/test_transfer_matrix.py
git commit -m "feat: configure INS bias correlation times"
```

### Task 2: Add failing tests for the opt-in JSONL matrix writer

**Files:**
- Create: `tests/log/test_tc_matrix_diagnostics.py`
- Create: `src/log/tc_matrix_diagnostics.py` only after the red test is observed

- [ ] **Step 1: Write the failing writer tests**

Create the test file:

```python
import json

import numpy as np

from src.log.tc_matrix_diagnostics import TcMatrixDiagnosticWriter


def test_writer_emits_header_and_15x15_propagation_snapshot(tmp_path):
    path = tmp_path / "tc-matrix.jsonl"
    writer = TcMatrixDiagnosticWriter(path, mode="rtk")
    p0 = np.eye(15)
    p1 = 2.0 * np.eye(15)
    phi = 3.0 * np.eye(15)
    q = 4.0 * np.eye(15)

    writer.open()
    writer.write_imu_prop(
        timestamp=1.0, dt=0.01, p_before=p0, phi=phi, q=q, p_after=p1)
    writer.close()

    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert records[0]["schema"] == "GIPY_TC_MATRIX_DIAG_V1"
    record = records[1]
    assert record["event"] == "imu_prop"
    assert record["matrix_order"] == "pos,vel,att,gyro_bias,accel_bias"
    assert np.asarray(record["p_before"]).shape == (15, 15)
    assert np.asarray(record["phi"]).shape == (15, 15)
    assert record["dt"] == 0.01


def test_writer_copies_input_arrays_and_writes_update_metadata(tmp_path):
    path = tmp_path / "tc-update.jsonl"
    writer = TcMatrixDiagnosticWriter(path, mode="rtk")
    p = np.eye(15)
    phi = np.eye(15)
    q = np.eye(15)
    v = np.array([1.0, 2.0])
    k = np.ones((15, 2))

    writer.open()
    writer.write_update(
        timestamp=2.0, p_before=p, p_after=2.0 * p, phi=phi, q=q,
        innovation=v, S_diag=np.array([3.0, 4.0]), K=k,
        feedback_x=np.arange(15, dtype=float), accepted=False,
        info={"mode": "rtk", "n_phase_att": 3, "n_phase_acc": 2,
              "n_code_att": 4, "n_code_acc": 3,
              "ref_sats": [7], "pairs": [(7, 8, 0, 0)]})
    p[0, 0] = 99.0
    writer.close()

    record = json.loads(path.read_text().splitlines()[1])
    assert record["event"] == "tc_update"
    assert record["accepted"] is False
    assert record["p_before"][0][0] == 1.0
    assert record["n_phase_att"] == 3
    assert record["ref_sats"] == [7]


def test_disabled_writer_does_not_create_file(tmp_path):
    path = tmp_path / "disabled.jsonl"
    writer = TcMatrixDiagnosticWriter(None, mode="rtk")
    writer.open()
    writer.close()
    assert not path.exists()
```

- [ ] **Step 2: Run the new tests and verify the intended failure**

Run:

```powershell
python -m pytest tests/log/test_tc_matrix_diagnostics.py -v
```

Expected: collection fails with the missing `src.log.tc_matrix_diagnostics` module. Fix test syntax first if the failure is different.

- [ ] **Step 3: Implement the minimal JSONL writer**

Create `src/log/tc_matrix_diagnostics.py` with these interfaces:

```python
class TcMatrixDiagnosticWriter:
    SCHEMA = "GIPY_TC_MATRIX_DIAG_V1"

    def __init__(self, path, mode="tc"):
        self.path = Path(path) if path else None
        self.mode = str(mode)
        self._fp = None
        self._imu_count = 0

    def open(self):
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self.path.open("w", encoding="utf-8")
        self._write({
            "schema": self.SCHEMA,
            "mode": self.mode,
            "time_system": "GPST+unix_timestamp",
            "units": {"dt": "s", "P": "SI covariance",
                      "Phi": "dimensionless", "Q": "SI covariance"},
            "matrix_order": "pos,vel,att,gyro_bias,accel_bias",
            "matrix_shape": [15, 15],
        })

    def close(self):
        if self._fp is not None:
            self._fp.close()
            self._fp = None

    def write_imu_prop(self, timestamp, dt, p_before, phi, q, p_after):
        if self._fp is None or self._imu_count >= 10:
            return
        self._imu_count += 1
        self._write(self._matrix_record(
            "imu_prop", timestamp, dt, p_before, phi, q, p_after))

    def write_update(self, timestamp, p_before, p_after, phi, q,
                     innovation, S_diag, K, feedback_x, accepted, info):
        if self._fp is None:
            return
        record = self._matrix_record(
            "tc_update", timestamp, info.get("dt", -1.0),
            p_before, phi, q, p_after)
        record.update({
            "accepted": bool(accepted),
            "innovation": np.asarray(innovation, dtype=float).tolist(),
            "innovation_norm": float(np.linalg.norm(innovation)),
            "S_diag": np.asarray(S_diag, dtype=float).tolist(),
            "K_block_norms": self._block_norms(K),
            "feedback_x": np.asarray(feedback_x, dtype=float).tolist(),
            "n_phase_att": int(info.get("n_phase_att", 0)),
            "n_phase_acc": int(info.get("n_phase_acc", 0)),
            "n_code_att": int(info.get("n_code_att", 0)),
            "n_code_acc": int(info.get("n_code_acc", 0)),
            "n_phase_rej": int(info.get("n_phase_rej", 0)),
            "n_code_rej": int(info.get("n_code_rej", 0)),
            "ref_sats": [int(x) for x in info.get("ref_sats", [])],
            "pairs": [list(x) for x in info.get("pairs", [])],
        })
        self._write(record)
```

The implementation must copy arrays to lists at write time, use finite-value conversion that preserves `nan`/`inf` as JSON-safe strings or explicit `None`, and catch only file I/O errors at the integration boundary (the writer itself should not silently change matrices).

- [ ] **Step 4: Run the writer tests and inspect JSONL output**

Run:

```powershell
python -m pytest tests/log/test_tc_matrix_diagnostics.py -v
```

Expected: all writer tests pass; one header record and one event record are present, each matrix is 15×15, and no file is created when the path is disabled.

- [ ] **Step 5: Commit the isolated writer/test change**

```powershell
git add -- src/log/tc_matrix_diagnostics.py tests/log/test_tc_matrix_diagnostics.py
git commit -m "feat: add opt-in TC matrix diagnostics"
```

### Task 3: Capture propagation snapshots without changing filter behavior

**Files:**
- Modify: `src/core/tc/tc_estimator.py`
- Modify: `tests/tc/test_tc_estimator.py`

- [ ] **Step 1: Add a failing snapshot test**

Add a test using the existing estimator fixture and one accepted IMU step:

```python
def test_time_update_exposes_last_15_state_propagation_snapshot(estimator, imu):
    estimator.time_update(imu)
    snapshot = estimator.last_propagation_snapshot
    assert snapshot is not None
    assert snapshot["p_before"].shape == (15, 15)
    assert snapshot["p_after"].shape == (15, 15)
    assert snapshot["phi"].shape == (15, 15)
    assert snapshot["q"].shape == (15, 15)
    assert snapshot["dt"] > 0.0
```

Use the actual fixture names and IMU construction already present in `tests/tc/test_tc_estimator.py`; do not add mocks for NumPy propagation.

- [ ] **Step 2: Run the test and verify it fails**

```powershell
python -m pytest tests/tc/test_tc_estimator.py -k propagation_snapshot -v
```

Expected: `AttributeError` for the missing snapshot property, not a propagation failure.

- [ ] **Step 3: Add an immutable latest-snapshot field**

In `TcEstimator.__init__`, initialize:

```python
        self._last_propagation_snapshot = None

    @property
    def last_propagation_snapshot(self):
        return self._last_propagation_snapshot
```

In `time_update`, copy `self.P[:15, :15]` before modifying `P`, and after the existing propagation/floor/symmetrization store:

```python
        self._last_propagation_snapshot = {
            "timestamp": float(self.ins_update.state.timestamp),
            "dt": float(dt),
            "p_before": p_before_15,
            "p_after": self.P[:15, :15].copy(),
            "phi": Phi_ins[:15, :15].copy(),
            "q": Q_ins[:15, :15].copy(),
        }
```

Reset the field to `None` when `last_update_accepted` is false or `dt <= 0`. Do not reuse mutable `P` views and do not move any existing propagation operation.

- [ ] **Step 4: Run focused estimator and transfer-matrix tests**

```powershell
python -m pytest tests/tc/test_tc_estimator.py tests/ins/test_transfer_matrix.py -v
```

Expected: all selected tests pass and existing state/P assertions remain unchanged.

- [ ] **Step 5: Commit the snapshot change**

```powershell
git add -- src/core/tc/tc_estimator.py tests/tc/test_tc_estimator.py
git commit -m "feat: expose TC propagation diagnostic snapshots"
```

### Task 4: Wire GNSS update diagnostics and rejection metadata

**Files:**
- Modify: `src/core/tc/tc_integration.py`
- Modify: `src/core/tc/tc_measurement.py`
- Modify: `src/log/tc_matrix_diagnostics.py`
- Modify: `tests/tc/test_tc_integration.py`

- [ ] **Step 1: Add failing metadata tests**

Add a focused `_build_dd` test or extend the existing measurement fixture to assert that `info` contains integer attempted/accepted/rejected phase/code counts and `pairs`. The expected invariant is:

```python
assert info["n_phase_att"] >= info["n_phase_acc"]
assert info["n_code_att"] >= info["n_code_acc"]
assert info["n_phase_rej"] == info["n_phase_att"] - info["n_phase_acc"]
assert info["n_code_rej"] == info["n_code_att"] - info["n_code_acc"]
assert all(len(pair) == 4 for pair in info["pairs"])
```

Add a writer integration assertion that a rejected `tc_meas_update` produces one `tc_update` record with `accepted=false` and leaves `P`/state unchanged.

- [ ] **Step 2: Run the new tests and verify failure**

```powershell
python -m pytest tests/tc/test_tc_integration.py -k "diagnostic or rejected" -v
```

Expected: missing rejection fields or missing integration writer output causes failure.

- [ ] **Step 3: Add rejected counters at the source**

In `_DdBase._build_dd`, initialize `n_phase_rej = n_code_rej = 0`; increment the corresponding counter immediately before each `continue` caused by `abs(v_nv) > nav.maxinno[code] * thresadj`; include all six counters in `info`:

```python
        info = {
            "pairs": used_pairs,
            "n": len(v),
            "ref_sats": sorted({p[0] for p in used_pairs}),
            "n_phase_att": n_phase_att,
            "n_phase_acc": n_phase_acc,
            "n_phase_rej": n_phase_rej,
            "n_code_att": n_code_att,
            "n_code_acc": n_code_acc,
            "n_code_rej": n_code_rej,
        }
```

Do not change the rejection threshold, residual, `H`, `R`, or row ordering.

- [ ] **Step 4: Add an explicit diagnostic writer to `TcIntegration`**

In `__init__`, read `tc.matrix_diagnostics_path`; construct `TcMatrixDiagnosticWriter` only when non-empty, open it after initialization metadata is available, and close it in the existing stream shutdown path. If opening or writing raises `OSError`, log a warning, disable the writer, and continue filtering.

After each successful `self._est.time_update(...)` in both interpolation and ordinary IMU paths, call a helper that writes the estimator's latest snapshot. The helper must enforce the writer's first-ten-IMU limit.

At `_trigger_meas`, use the already captured `pre_p` and latest propagation snapshot. Compute `S = H @ pre_p @ H.T + R`, `K = pre_p @ H.T @ inv(S)` and the pre-update innovation exactly as the current filter does for diagnostic purposes only. Call `write_update(..., accepted=True, feedback_x=feedback_x, info=info)` after a successful update. In the post-fit rejection branch call it with `accepted=False`, `p_after=pre_p`, `feedback_x=np.zeros_like(self._est.x)`, then continue the existing failure handling. The diagnostic call must not alter rollback or recovery behavior.

Pass `dt`, `n_phase_rej`, and `n_code_rej` through the metadata dictionary. Keep `last_update_info` and the existing CSV writer fields backward-compatible.

- [ ] **Step 5: Run focused tests and compare default state behavior**

```powershell
python -m pytest tests/tc/test_tc_measurement_outlier.py tests/tc/test_tc_integration.py tests/tc/test_tc_estimator.py -v
```

The repository currently has a pre-existing collection failure because `test_tc_measurement_outlier.py` imports missing `detslp_code`; record that exact blocker if it remains. Run the other focused TC tests separately and require zero new failures.

- [ ] **Step 6: Commit the integration change**

```powershell
git add -- src/core/tc/tc_integration.py src/core/tc/tc_measurement.py src/log/tc_matrix_diagnostics.py tests/tc/test_tc_integration.py
git commit -m "feat: record TC update and rejection diagnostics"
```

### Task 5: Add deterministic comparison/report tooling and experiment configurations

**Files:**
- Create: `tools/issue_9_7_compare.py`
- Create: `data/rtk-ins-9-7-tau360.yaml` only if a reproducible candidate snapshot is needed
- Modify: `data/rtk-ins紧组合.yaml` only to document optional keys; leave their effective defaults at 36 s

- [ ] **Step 1: Add report-tool tests or self-checks first**

The tool must parse `.rslt` with the existing `data/plot/error-rslt.py` functions, count Qins, compute the target window and full-overlap metrics, parse `$GBIAS/$ABIAS`, and validate JSONL matrix records. Add a small test fixture or a `--self-check` path that rejects non-finite matrices, wrong shape, missing header, and inconsistent accepted/rejected metadata.

- [ ] **Step 2: Implement report generation**

The report tool accepts:

```text
--rslt PATH --stat PATH --matrix PATH --truth PATH
--start-week 2046 --start-sec 359000 --end-sec 359100
--label baseline|tau360 --json-out PATH --markdown-out PATH
```

It reports, without modifying inputs:

- Qins histogram and accepted update count;
- target-window, pre-registered worst-window, and full-overlap E/N/U/H/3D RMSE;
- zero-bias median/std/p95 in SI units, with ignav legacy ABIAS Y/Z marked unavailable;
- matrix event counts, first non-finite/negative-eigenvalue/symmetry violation, and first ten IMU records;
- phase/code attempted/accepted/rejected counts, reference satellites, and accepted/rejected TC updates.

Use the existing fixed worst-window definition from `issue/9-1机械编排协方差更新精调.md`; do not choose a new worst window after seeing candidate results.

- [ ] **Step 3: Prepare independent experiment output paths**

Use separate directories:

```text
data/output/exp-9-7-baseline/
data/output/exp-9-7-tau360/
```

The baseline must use the exact `data/rtk-ins紧组合.yaml` effective values. The candidate changes only:

```yaml
ins:
  gyro_bias_corr_time_s: 360.0
  acce_bias_corr_time_s: 360.0
tc:
  matrix_diagnostics_path: "data/output/exp-9-7-tau360/tc-matrix.jsonl"
```

Both runs enable `tc.diagnostics_path` in their experiment copy and use distinct `.rslt/.stat` names. The formal configuration remains at 36 s unless the candidate passes all gates.

- [ ] **Step 4: Commit the report/tool/config changes**

```powershell
git add -- tools/issue_9_7_compare.py data/rtk-ins-9-7-tau360.yaml data/rtk-ins紧组合.yaml
git commit -m "tools: add zero-bias comparison report"
```

### Task 6: Execute regression, tau experiment, and accuracy gates

**Files:**
- Generate only ignored experiment artifacts under `data/output/exp-9-7-*` and reports under the same directories.

- [ ] **Step 1: Run the current formal YAML after code changes**

Run exactly:

```powershell
python src/main.py data/rtk-ins紧组合.yaml
```

Capture exit code, runtime, output row count, Qins histogram, and MD5 of `data/output/RTKINS.rslt`. Compare to the current immediate-feedback baseline (`173191` rows, Qins `0/2/3 = 657/170827/1707`, MD5 recorded before the run). If default behavior changes, stop and investigate before running tau360.

- [ ] **Step 2: Run the formal YAML regression tests and focused tests**

```powershell
python -m pytest tests/tc/test_tc_estimator.py tests/tc/test_tc_integration.py tests/ins/test_transfer_matrix.py tests/log/test_tc_matrix_diagnostics.py -v
```

The known unrelated `detslp_code` collection issue must be reported separately; no new failure is acceptable.

- [ ] **Step 3: Run baseline and tau360 independently**

Run the exact YAML-driven pipeline for each experiment directory, keeping input files, GPS-only/L1/AR-off settings, initial states, PSDs, and GNSS thresholds identical. Then run:

```powershell
python tools/issue_9_7_compare.py --label baseline ...
python tools/issue_9_7_compare.py --label tau360 ...
```

- [ ] **Step 4: Apply the no-regression gate**

Accept tau360 only if all are true:

```text
target-window E/N/U/H/3D RMSE <= baseline + 1e-6 m
fixed-worst-window E/N/U/H/3D RMSE <= baseline + 1e-6 m
full-overlap E/N/U/H/3D RMSE <= baseline + 1e-6 m
Qins histogram and update count unchanged
no new downgrade/reboot/recovery events
all recorded P/Phi/Q finite, 15×15, symmetric P/Q within 1e-10,
and no unexplained negative eigenvalue
```

If any gate fails, do not change the formal YAML; retain the candidate output and mark it `ROLLBACK tau360`.

- [ ] **Step 5: Compare zero-bias curves with ignav safely**

Run the existing `data/plot/mech-babg.py` against the experiment stat files or use the report tool. Compare all gyro axes; compare only ignav `ba_x` for legacy ABIAS. Report the comparison as internal equivalent estimates, not sensor truth.

### Task 7: Record evidence and final verification

**Files:**
- Modify: `issue/9-7零偏估计的仔细对比.md`

- [ ] **Step 1: Append the execution record**

Add a dated section containing code commit, config hashes, input paths/hashes, exact commands, experiment directories, baseline/candidate metrics, matrix health, zero-bias statistics, and the accept/reject decision. State explicitly that the pre-existing test collection error is unrelated if still present.

- [ ] **Step 2: Run final verification before claiming completion**

```powershell
git diff --check
git status --short
python -m pytest tests/tc/test_tc_estimator.py tests/tc/test_tc_integration.py tests/ins/test_transfer_matrix.py tests/log/test_tc_matrix_diagnostics.py -v
python src/main.py data/rtk-ins紧组合.yaml
python tools/issue_9_7_compare.py --label formal ...
```

Read all exit codes and report actual metrics. Do not claim that precision is preserved unless the fresh formal YAML run and comparison output prove it.

- [ ] **Step 3: Commit only intended source/docs changes**

```powershell
git add -- src/core/ins/transfer_matrix.py src/core/tc/tc_estimator.py src/core/tc/tc_integration.py src/core/tc/tc_measurement.py src/log/tc_matrix_diagnostics.py tests/ins/test_transfer_matrix.py tests/tc/test_tc_estimator.py tests/tc/test_tc_integration.py tests/log/test_tc_matrix_diagnostics.py tools/issue_9_7_compare.py issue/9-7零偏估计的仔细对比.md
git commit -m "feat: execute zero-bias comparison diagnostics"
```

Do not stage `.pyc`, generated `data/output` artifacts, or the user's pre-existing `.gitignore`/issue changes.

## Plan self-review

- Spec coverage: configurable 36/360 s model, opt-in propagation/update matrices, phase/code metadata, ignav-safe bias interpretation, independent experiments, YAML regression, three-window no-regression gate, and issue evidence are covered by Tasks 1–7.
- Placeholder scan: no `TBD`, `TODO`, or unspecified “appropriate” implementation steps remain; command paths and required fields are explicit.
- Type consistency: the writer exposes `write_imu_prop` and `write_update`; the estimator exposes `last_propagation_snapshot`; integration passes copied 15×15 arrays and the shared metadata dictionary.
- Scope: no changes to GNSS equations, feedback policy, PSD, initial bias, AR, thresholds, or truth evaluation semantics.
