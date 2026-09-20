# README Results and Relative Config Paths Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Add the three existing comparison plots and GREAT-MSF dataset context to README, and make the reference YAML use paths relative to the gipylib repository root.

**Architecture:** Keep the change documentation/config-only. README references the existing PNG files with plot/ paths in a compact HTML table; the YAML keeps current parameter values and changes only external absolute input paths to ../GREAT-MSF/.... No path-resolution code or runtime behavior is changed.

**Tech Stack:** Markdown, HTML embedded in Markdown, YAML, Git static checks.

---

### Task 1: Convert the GREAT reference configuration paths

**Files:**
- Modify: config/rtktc-rate.yaml lines 12-16 and 113
- Do not modify: configuration values, algorithm switches, output filenames, or existing repository-relative output paths.

- [x] Step 1: Replace only the five external absolute paths.

Use these exact repository-root-relative values:

    rover_path: "../GREAT-MSF/sample_data/MSF_20201027/GNSS/Rover/campus-day.20O"
    base_path:
      - "../GREAT-MSF/sample_data/MSF_20201027/GNSS/Base/WUH200CHN_R_20203010200_15M_01S_MO.crx.20O"
      - "../GREAT-MSF/sample_data/MSF_20201027/GNSS/Base/WUH200CHN_R_20203010215_15M_01S_MO.crx.20O"
    eph_path: "../GREAT-MSF/sample_data/MSF_20201027/GNSS/Product/BRDM00DLR_S_20203010000_01D_MN.rnx"
    imu_data_path: "../GREAT-MSF/sample_data/MSF_20201027/IMU/campus-01-MEMS.txt"

Leave these existing repository-relative paths unchanged:

    diagnostics_path: "data-great/output/rtktc-rate/tc-measurement-diagnostics.csv"
    matrix_diagnostics_path: "data-great/output/rtktc-rate/tc-matrix-diagnostics.jsonl"
    output_dir: "data-great/output/rtktc-rate"

- [x] Step 2: Confirm the path-only scope.

Run: git diff -- config/rtktc-rate.yaml

Expected: only the five /home/mxl/workplace/GREAT-MSF/... strings change to ../GREAT-MSF/...; no numeric or boolean configuration values change.

### Task 2: Add the compact comparison gallery to README

**Files:**
- Modify: README.md before the 目录结构 section
- Reference: plot/tra-diff-rtkdc.png, plot/rtk-tc-error.png, plot/tra-enu.png

- [x] Step 1: Add the GREAT-MSF dataset context.

Add a result-comparison section stating that the plots use the GREAT-MSF/sample_data/MSF_20201027 campus01 dataset and these inputs:

- rover: GNSS/Rover/campus-day.20O
- base: GNSS/Base/WUH200CHN_R_20203010200_15M_01S_MO.crx.20O and GNSS/Base/WUH200CHN_R_20203010215_15M_01S_MO.crx.20O
- ephemeris: GNSS/Product/BRDM00DLR_S_20203010000_01D_MN.rnx
- IMU: IMU/campus-01-MEMS.txt, 100 Hz rate data

- [x] Step 2: Insert the three-image HTML table.

Use repository-relative image sources and one caption per image:

    <table>
    <tr>
    <td align="center"><img src="plot/tra-diff-rtkdc.png" alt="GInsStream 与 ignav2s 轨迹差异" width="100%"><br>GInsStream 与 ignav2s 的轨迹差异</td>
    <td align="center"><img src="plot/rtk-tc-error.png" alt="GInsStream 与 GREAT 定位误差" width="100%"><br>GInsStream 与 GREAT RTK-TC 的 ENU 定位误差</td>
    <td align="center"><img src="plot/tra-enu.png" alt="GInsStream 与 GREAT 的 ENU 轨迹差异" width="100%"><br>GInsStream 与 GREAT/真值的 ENU 轨迹差异</td>
    </tr>
    </table>

Keep surrounding README content unchanged except for the new section and dataset description.

### Task 3: Perform static verification only

**Files:**
- Verify: README.md, config/rtktc-rate.yaml, and the three PNG files.

- [x] Step 1: Verify assets and relative references.

Run:
    test -f plot/tra-diff-rtkdc.png
    test -f plot/rtk-tc-error.png
    test -f plot/tra-enu.png
    rg -n 'plot/(tra-diff-rtkdc|rtk-tc-error|tra-enu)\.png|MSF_20201027|campus-01-MEMS\.txt' README.md

Expected: all three files exist and README contains the three image references plus the GREAT-MSF dataset identifiers.

- [x] Step 2: Verify no absolute GREAT input paths remain.

Run:
    if rg -n '/home/mxl/workplace/GREAT-MSF|^[[:space:]]*(rover_path|eph_path|imu_data_path):[[:space:]]*"/' config/rtktc-rate.yaml; then
      exit 1
    fi
    rg -n '\.\./GREAT-MSF/sample_data' config/rtktc-rate.yaml

Expected: the first command produces no output and exits successfully; the second prints the five relative external input paths.

- [x] Step 3: Check whitespace and review the final diff without running the configuration.

Run:
    git diff --check
    git diff -- README.md config/rtktc-rate.yaml

Expected: no whitespace errors; the diff contains only the requested README gallery/dataset note and path-string changes. Do not invoke src/main.py and do not generate result files.
