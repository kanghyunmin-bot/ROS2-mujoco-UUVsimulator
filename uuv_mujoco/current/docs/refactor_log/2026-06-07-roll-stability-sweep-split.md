# Roll Stability Sweep Split

Date: 2026-06-07

Scope: active runtime tooling under `uuv_mujoco/current/tools`.

## Change

- Split `tools/roll_stability_sweep.py` into:
  - `roll_stability_candidates.py`
  - `roll_stability_file_edits.py`
  - `roll_stability_metrics.py`
  - `roll_stability_probe.py`
  - `roll_stability_runner.py`
- Kept the original executable name as a CLI compatibility entry point.
- Moved `rclpy` and `StabilityProbe` imports into the actual run path, so
  `roll_stability_sweep.py --help` works without a sourced ROS2 environment.

## Contract Notes

- The tool still restores `sim_profiles.json`, scene XML, and thruster mapping
  after each candidate.
- No live runtime physics coefficient was changed by this refactor.
- No controller-parity observation point or plant input contract changed.

## Validation

```text
python3 -m py_compile uuv_mujoco/current/tools/roll_stability_sweep.py \
  uuv_mujoco/current/tools/roll_stability_candidates.py \
  uuv_mujoco/current/tools/roll_stability_file_edits.py \
  uuv_mujoco/current/tools/roll_stability_metrics.py \
  uuv_mujoco/current/tools/roll_stability_probe.py \
  uuv_mujoco/current/tools/roll_stability_runner.py
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/roll_stability_sweep.py --help
PYTHONPATH=uuv_mujoco/current/tools python3 <non-ROS helper smoke>
python3 uuv_mujoco/current/tools/refactor_inventory.py --limit 25 --format markdown
```

Results:

- Compile passed.
- Non-ROS helper smoke passed.
- CLI help passed without importing ROS2/rclpy.
- `tools/roll_stability_sweep.py` dropped out of the top 25 hotspot list.
