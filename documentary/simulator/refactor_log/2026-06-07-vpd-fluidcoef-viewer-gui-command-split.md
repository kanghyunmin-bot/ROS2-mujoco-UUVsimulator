# VPD, Dynamic Fluidcoef, Viewer, And GUI Command Split

Date: 2026-06-07

Scope: active runtime code under `sim/current`.

## What changed

- Split synthetic ExternalNav `VISION_POSITION_DELTA` generation into:
  - `bridge/sitl_external_nav_vpd_clock.py`
  - `bridge/sitl_external_nav_vpd_pose.py`
  - `bridge/sitl_external_nav_vpd_rate.py`
  - `bridge/sitl_external_nav_vpd_send.py`
- Split dynamic MuJoCo fluid coefficient setup into:
  - `sim/physics/dynamic_fluidcoef_setup_config.py`
  - `sim/physics/dynamic_fluidcoef_setup_arrays.py`
  - `sim/physics/dynamic_fluidcoef_setup_rows.py`
  - `sim/physics/dynamic_fluidcoef_setup_logging.py`
- Split MuJoCo viewer debug drawing into:
  - `sim/runtime/viewer_scene_builder.py`
  - `sim/runtime/viewer_scene_sensors.py`
  - `sim/runtime/viewer_scene_thrusters.py`
- Split GUI command common helpers into:
  - `gui/node_trigger_services.py`
  - `gui/node_initial_depth_commands.py`
  - `gui/node_command_timing.py`
  - `gui/node_command_state_gates.py`
  - `gui/node_command_retries.py`
  - `gui/node_command_override_pub.py`

## Contract boundaries preserved

- VPD position delta remains current `BODY_FRD`, matching the existing
  ArduSub `VISION_POSITION_DELTA` contract.
- Dynamic fluid coefficient reference/current ratio, row weights, axis weights,
  angular-axis weights, and logging values are unchanged.
- Viewer changes only move debug drawing helpers; no physics, sensor, or
  control state is changed.
- GUI command retry timing, arm/mode gate reason, trigger-service behavior, and
  command-override payload serialization are unchanged.
- ArduPilot source and submodule pointer were not touched.

## Verification

```text
python3 -m compileall -q sim/current uuv_control_gui.py
PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_source_audit_vpd_fluid_viewer_gui_split
PYTHONPATH=sim/current /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  - <<'PY'
from gui.node_commanding import _call_trigger_service
from bridge.sitl_external_nav_synthetic_vpd import _send_external_nav
from sim.physics.dynamic_fluidcoef_setup import build_dynamic_fluidcoef_setup
from sim.runtime.viewer_scene import ViewerSceneBuilder
print(callable(_call_trigger_service), callable(_send_external_nav), callable(build_dynamic_fluidcoef_setup), ViewerSceneBuilder.__name__)
PY
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/sim/current \
  --fetch --refresh-version
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  sim/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_vpd_fluid_viewer_gui_split \
  --simulate-s 0
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 sim/current/tools/refactor_inventory.py --limit 30
```

Results:

- Compile/import: pass.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Runtime freshness: pass, `sim/current -> v2.2`.
- Static physics audit: neutral force balance still
  `net_down=+0.000N`, `required_scale=1.000000`.
- Ubuntu compatibility: `fail=0`, `pass=16`, `warn=2`
  (`docker_daemon`, `ros2_env`).
- `git diff --check`: pass.
- Removed from top 30 hotspot inventory:
  `bridge/sitl_external_nav_synthetic_vpd.py`,
  `sim/physics/dynamic_fluidcoef_setup.py`,
  `sim/runtime/viewer_scene.py`, and
  `gui/node_commanding_common.py`.
