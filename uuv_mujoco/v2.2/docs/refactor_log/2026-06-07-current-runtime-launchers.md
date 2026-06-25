# Current Runtime Launchers

Date: 2026-06-07

Problem:

- The active runtime still lived behind the compatibility directory name
  `uuv_mujoco/v2.2`, so repeated manual commands could look stale even when the
  implementation had changed.

Change:

- Added root-level launch wrappers:
  - `uuv_mujoco/run_mujoco.sh`
  - `uuv_mujoco/start_sitl_mujoco.sh`
  - `uuv_mujoco/start_docker_sitl_mujoco.sh`
  - `uuv_mujoco/reset_sim.sh`
- Updated `uuv_mujoco/RUNTIME_VERSION.json` and `uuv_mujoco/CURRENT.md` so the
  active launch contract is explicit.

Contract:

- The wrappers resolve `UUV_MUJOCO_RUNTIME_DIR` first.
- If unset, they resolve `uuv_mujoco/current`.
- Before resolving, the wrappers source `.uuv_mujoco_env.sh` when present so
  Python, ROS, ArduPilot, and active-runtime defaults are shared with the rest
  of the stack.
- `uuv_mujoco/current` currently points to `v2.2`, but `v2.2` is now only the
  compatibility backing directory name.

Not changed:

- No ArduPilot files were modified.
- No ArduPilot submodule pointer was changed.
- Controller parity and plant input semantics were not changed.
