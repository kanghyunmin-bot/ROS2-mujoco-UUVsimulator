# Real Start State Extraction Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Changes

- Split `tools/real_start_state.py` into focused real-start contract modules:
  - `tools/real_start_common.py`
  - `tools/real_start_csv.py`
  - `tools/real_start_geometry.py`
  - `tools/real_start_extractors.py`
  - `tools/real_start_baro.py`
  - `tools/real_start_builder.py`
  - `tools/real_start_output.py`

## Contract Notes

- The CLI entry remains `tools/real_start_state.py`.
- The JSON output still includes the initial-state contract fields consumed by
  GUI and runtime real-start handling.
- The shell output still emits `UUV_REAL_START_*` assignments.
- Bar30/AP_Baro pressure datum handling remains in the same contract:
  `static_pressure_pa`, inferred real ground pressure, SITL ground pressure,
  frontend depth, and JSON depth are all preserved.
- DVL is still only used as a velocity fallback or explicit velocity source
  policy.  It is not introduced as an estimator workaround.

## Verification

- `python3 -m py_compile` passed for all split real-start modules.
- Real feedback CSV smoke:
  `tools/real_start_state.py --csv .../real_controller_feedback_20hz.csv --start 69.35 --format json`
  produced:
  - `source_t_s=69.35`
  - `depth_m=0.017334678`
  - `base_depth_m=0.106070034`
  - `static_pressure_pa=102069.995117188`
  - `baro_real_ground_pressure_pa=101901.97330239399`
  - `baro_json_depth_m=0.03154776209275964`
  - `mode=MANUAL`
  - `armed=true`
  - `rc_override_ch1_8=[1500,1500,1500,1488,1500,1500,1500,1500]`
- Shell output smoke printed the expected `UUV_REAL_START_*` assignments.
- `tools/refactor_inventory.py --root sim/current --limit 15` no longer
  lists `tools/real_start_state.py` in the top hotspot list.
