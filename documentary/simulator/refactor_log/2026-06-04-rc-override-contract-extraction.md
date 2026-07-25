# 2026-06-04 RC Override Contract Extraction

Goal: move ArduSub RC override normalization out of
`bridge/sitl_transport.py` and into the shared RC contract module without
changing the command stream.

## Actions

- Added `neutral_rc_override_frame()` to `sim/contracts/rc.py`.
- Added `normalize_ardusub_rc_override()` to `sim/contracts/rc.py`.
- Exported both functions from `sim.contracts`.
- Updated `bridge/sitl_transport.py` to keep compatibility wrapper methods
  while delegating to the shared contract functions.

## What did not change

- Channels 1..8 still map `0` and `65534` to release.
- Channels 1..8 still preserve `65535` as no-change.
- Channels 9..18 still preserve `0`, `65534`, and `65535` marker semantics.
- Missing channels still fill C1..C8 with `65535` and C9..C18 with `0`.
- RC override timing, command forwarding, and MAVLink send behavior are
  unchanged.
- No ArduPilot files were edited.

## Validation

- `python3 -m py_compile` passed for `sim/contracts/rc.py`,
  `sim/contracts/__init__.py`, and `bridge/sitl_transport.py`.
- Pure RC contract smoke verified neutral frame generation, primary-channel
  marker handling, missing-channel fill, and extension-channel marker handling.
- Source contract audit was rerun after the split: 10 PASS, 5 WARN, 0 FAIL.
- Plant-input gate checks were rerun after the split:
  - missing CSV failed with `csv_data_rows_lt_1`;
  - known non-neutral `full_mujoco_rcout.csv` passed with 42 data rows.
- Refactor inventory now reports `bridge/sitl_transport.py` at 2898 LOC and
  475 branches.

## Open contract work

Move remaining policy-heavy blocks out of `SitlTransport`:

1. Sensor replay clock and immediate JSON reply scheduler.
2. ExternalNav/VPD output adapter.
3. Readiness state machine and GUI command-ready reporting.
