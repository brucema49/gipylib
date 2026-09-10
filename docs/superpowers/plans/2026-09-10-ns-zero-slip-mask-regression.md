# NS=0 Slip-Mask Regression Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent a consumed rover/base LLI cycle-slip bit from being reused when a later RTK epoch has no common satellites and then common satellites return.

**Architecture:** Exercise the real `RtkTcMeas.build()` early-return path with satellite-processing dependencies reduced to an `ns=0` result. The regression test begins with a stale shared slip bit, runs the no-common-satellite epoch, and then invokes the existing LLI detector on a restored common observation. The production fix calls the existing `save_tc_phase_state()` before the early return, preserving the current state ownership and avoiding a second reset mechanism.

**Tech Stack:** Python 3.12, NumPy, pytest, existing RTK-TC measurement code.

---

### Task 1: Capture the stale-mask failure

**Files:**

- Modify: `tests/tc/test_tc_measurement_outlier.py`
- Test: `tests/tc/test_tc_measurement_outlier.py::test_rtk_no_common_epoch_clears_consumed_slip_before_common_recovery`

- [ ] **Step 1: Write the failing test**

```python
def test_rtk_no_common_epoch_clears_consumed_slip_before_common_recovery(monkeypatch):
    """An ns=0 epoch must not carry an old LLI reset into the next common epoch."""
    nav = _synthetic_nav()
    nav.rb = np.zeros(3)
    nav.slip = np.zeros((uGNSS.MAXSAT, nav.nf), dtype=int)
    nav.prev_lli = np.zeros((uGNSS.MAXSAT, nav.nf, 2), dtype=int)
    nav.slip[6, 0] = 1
    # Monkeypatch satposs/zdres/selsat so RtkTcMeas.build follows its real
    # no-common-satellite early-return path.
    ...
    RtkTcMeas({"ins": {}, "gnss": {}}).build(...)
    detslp_ll(nav, restored_rover, np.array([0]), 1)
    detslp_ll(nav, restored_base, np.array([0]), 0)
    assert not (nav.slip[6, 0] & 1)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python -m pytest tests/tc/test_tc_measurement_outlier.py::test_rtk_no_common_epoch_clears_consumed_slip_before_common_recovery -q`

Expected: fail because the `ns <= 0` early return bypasses `save_tc_phase_state()` and leaves `nav.slip[6, 0]` set.

### Task 2: Clear the consumed epoch state on the early-return path

**Files:**

- Modify: `src/core/tc/tc_measurement.py:625-627`
- Test: `tests/tc/test_tc_measurement_outlier.py::test_rtk_no_common_epoch_clears_consumed_slip_before_common_recovery`

- [ ] **Step 1: Add the minimal fix**

```python
if ns <= 0:
    save_tc_phase_state(nav, obsb, obsr, iu, ir)
    return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {}
```

- [ ] **Step 2: Run the regression test to verify it passes**

Run: `python -m pytest tests/tc/test_tc_measurement_outlier.py::test_rtk_no_common_epoch_clears_consumed_slip_before_common_recovery -q`

Expected: pass; recovered normal LLI observations do not reset the old ambiguity.

- [ ] **Step 3: Run the neighboring TC tests**

Run: `python -m pytest tests/tc/test_tc_measurement_outlier.py tests/tc/test_tc_integration.py -q`

Expected: all tests pass.
