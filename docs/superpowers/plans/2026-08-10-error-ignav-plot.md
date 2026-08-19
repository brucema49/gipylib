# ignav RTKTC Plot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a dedicated plotter for `data/rtktc.rslt` that marks the nearest sample to each integer GPST second in green without using `Qins`, draws all other samples red, and computes RMS only from green samples.

**Architecture:** Keep `error-rslt.py` unchanged. Put the dedicated parser, deterministic integer-second selector, ENU error calculation, statistics, and plotting entry point in `error-ignav.py`; expose the selector as a small pure function for tests.

**Tech Stack:** Python 3.12, NumPy, Matplotlib, pytest, existing `truth.csv` and `rtktc.rslt` formats.

---

### Task 1: Lock deterministic integer-second selection

**Files:**
- Create: `tests/test_error_ignav_plot.py`
- Modify: `data/plot/error-ignav.py`

- [ ] Write a failing test importing `select_integer_second_indices` and assert that `358082.996` is selected for integer second `358083`, that every selected index is unique, and that equal-distance candidates select the earlier timestamp.
- [ ] Write a failing test importing `generate_error_statistics` and assert that red/non-selected samples do not affect the green-only RMS.
- [ ] Run `python -m pytest tests/test_error_ignav_plot.py -q`; expect collection failure because the selector is not defined.
- [ ] Implement only the pure selector using absolute GPST seconds and nearest-neighbor search; do not inspect `Qins`.
- [ ] Run the focused test and confirm it passes.

### Task 2: Add ignav parser, ENU statistics, and plot

**Files:**
- Modify: `data/plot/error-ignav.py`

- [ ] Parse `data/rtktc.rslt` comments and rows as week, sow, ECEF x/y/z; parse `data/truth.csv` as the reference trajectory.
- [ ] Reuse WGS84 conversion and ENU error equations, retaining all 100 Hz samples.
- [ ] Select nearest integer-second indices from the full overlapping interval, color all points red and selected points green, and calculate statistics from selected points only.
- [ ] Save the figure to `data/problem/error_ignav_plot.png` and fail clearly when either file has no valid rows or no overlap.

### Task 3: Verify real data and regression

**Files:**
- Test: `tests/test_error_ignav_plot.py`

- [ ] Add a real-file smoke test asserting `data/rtktc.rslt` parses, selected indices are non-empty, and selected sample `2046 358082.996` is in the green set when present.
- [ ] Run `python -m pytest tests/test_error_ignav_plot.py tests/test_error_rslt_plot.py -q`.
- [ ] Run `python data/plot/error-ignav.py` and verify the output reports green integer-second samples and creates `data/problem/error_ignav_plot.png`.
- [ ] Run `python -m py_compile data/plot/error-ignav.py` and `git diff --check`.
