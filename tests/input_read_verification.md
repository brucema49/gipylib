# Input read error verification — 2026-10-02

## Changed behavior

- Unsupported RINEX observation/navigation versions (below 3.02 or 4.x) fail with the original filename and supported version range.
- Input paths and RINEX headers are checked before output files open.
- Invalid nonempty numbers, NaN/Inf, malformed data rows, incomplete records, missing inputs, and empty/no-usable-data inputs raise visible errors rather than becoming zero or being silently skipped.
- Reader errors include filenames and physical line numbers where available. Blank/comment rows and legal missing observation fields remain accepted.
- The first worker failure cancels the pipeline, releases queue-blocked producers, and causes CLI exit code 1. Output/writer exceptions also propagate.
- Text inputs are scanned in full before output files open, so malformed IMU tails beyond the GNSS interval are detected. This adds one input scan at startup.
- Mandatory navigation fields and calendar dates are validated; known optional/spare fields can still be blank.
- Valid optional phase-shift and GLONASS bias header records are parsed within their data columns; label text is not read as numeric data.

## Verification

- `python -m pytest tests/test_input_read_errors.py -q`: 39 passed.
- `python -m pytest tests -q --tb=short`: 65 passed, 6 failed.
- The same six failures were reproduced against a separate `git archive HEAD` snapshot of the original source, using the existing test and plotting helper files. They are unrelated to these input-reading changes.
- Actual `phone1515` GPS+BDS input reading: 5777 rover epochs, 5777 base epochs, 1724 ephemerides, both observation file handles closed. No solution output was regenerated during this check.

## Existing suite failures

1. `tests/test_error_ignav_plot.py::test_real_ignav_file_selects_the_example_sample`: missing `data/rtktc.rslt`.
2. `tests/test_error_rslt_plot.py::test_nearest_match_keeps_qins_for_error_plot`: existing plotting helper has no `match_ref_to_eval_nearest`.
3. `tests/test_rtkpos_rtd.py::test_rtd_ddres_position_jacobian_uses_rover_line_of_sight_only`: `rtkcmn.trace_level` is not initialized by that test.
4. `tests/test_rtkpos_rtd.py::test_nav_gnss_t_filters_spp_and_relative_position_satellites`: current satellite exclusion behavior differs from its expectation.
5. `tests/tc/test_tc_multisystem_clock.py::test_spp_clock_block_matches_rtklib_constellation_layout`: current `TcStateIndex` has no `n_clk`.
6. `tests/tc/test_tc_multisystem_clock.py::test_spp_measurement_accepts_configured_non_gps_systems`: current TC measurement module has no `update_frequency_code_bias`.

Regression tests and this report reside under the repository's already-ignored `tests/` directory. These two files are explicitly included in the input-reading fix commit; other ignored data and tests remain excluded.
