# RTD-TC Initialization Mode Plan

**Goal:** Stop exposing forced SPP during TC pre-initialization and use the configured GNSS positioning mode for both pre-initialization output and the initial INS position. In `rtd` mode, the position source must be the rover/base code double-difference solution.

**Architecture:** Keep the existing SPP calculation only as an internal numerical seed for RTKLIB `relpos()` and for optional clock-bias initialization. The RTD result, not the SPP result, becomes the reported/pre-initialization position and the `InsState` initial position. Do not modify `src/core/gnss/rtklib` or the RTD measurement model.

**Files:**
- Modify `src/core/tc/tc_stream.py`: make `_write_gnss_only()` select SPP/RTK/RTD by `self._mode`; use `relpos()` for `rtd` and map the output quality to the TC RTD quality code (`Q=4`).
- Modify `src/core/tc/tc_integration.py`: keep SPP as a relpos seed, accept the RTD differential position as the initialization position, and limit the existing RTK-vs-SPP false-fix rejection to `rtk` mode. Preserve the current RTK guard.
- Modify `tests/tc/test_tc_integration.py` or add `tests/tc/test_tc_init_mode.py`: mock the GNSS wrappers and assert that RTD pre-initialization/initialization consumes the differential position, while SPP mode remains SPP-only.

## Implementation Steps

- [ ] Add a failing regression test for mode-specific pre-initialization: an `rtd` stream must call `relpos()` and write its returned position with `Q=4`; an `spp` stream must not call `relpos()` and must write `Q=5`.
- [ ] Add a failing initialization regression test: when `relpos()` returns a valid RTD position that differs from SPP by more than 50 m, `_try_init()` must use the RTD position instead of rejecting it. The test should assert the assembled `GnssSolution.position` equals the differential result.
- [ ] Update `TcStream._write_gnss_only()` so SPP is used only to seed `relpos()` in `rtk`/`rtd`; write the returned relative position and mode-specific quality/std values. Restore all modified `nav` state in `finally` as today.
- [ ] Update `TcIntegration._try_init()` so the SPP-vs-relative-position rejection applies only to `rtk`; in `rtd`, set `rr` and the initialization quality from the differential position and continue initialization. Keep SPP clock estimates as optional auxiliary seeds.
- [ ] Run the focused TC/configuration tests, then run the phone RTD-TC replay and verify pre-initialization rows are `Q=4` (not `Q=5`), the initialization state position matches the RTD source, and no `TC meas build` interface errors return.

**Non-goals:** No changes to `src/core/gnss/rtklib/rtkpos.py`, `pntpos.py`, the RTD double-difference measurement equations, or the INS propagation/update equations.
