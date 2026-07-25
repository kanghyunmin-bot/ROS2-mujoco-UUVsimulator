# Vertical State, GUI Launch, Thruster Params, Dev-OS, and Axis Metrics Split

Date: 2026-06-07

## Scope

Continue reducing branch-heavy active-runtime hotspots under
`sim/current` while preserving Bar30/SITL contracts, GUI launch
behavior, thruster parameter semantics, dev-OS checks, and axis RC report
formats.

No ArduPilot source, submodule pointer, controller-parity observation point,
plant input contract, JSON servo semantics, or physics coefficients were
changed.

## Changed Files

Vertical state estimation:

- `bridge/ros2_state_vertical_hold.py`: initial-depth-hold state, zero-feedback
  reason, and zero-feedback logging.
- `bridge/ros2_state_vertical_truth.py`: base acceleration, vertical truth, and
  Bar30 pressure helpers.
- `bridge/ros2_state_sitl_vertical.py`: canonical ArduSub SITL Bar30/frontend
  match vertical estimate.
- `bridge/ros2_state_vertical.py`: compatibility facade.

GUI simulator launch:

- `gui/sim_stack_launch_args.py`: GUI launch environment and argument helpers.
- `gui/sim_stack_launch_runtime.py`: start/restart subprocess lifecycle.
- `gui/sim_stack_launch_mixin.py`: compatibility mixin.

Thruster parameter loading:

- `sim/physics/thruster_param_global.py`: global JSON field parsing.
- `sim/physics/thruster_param_per_thruster.py`: per-thruster reset and
  parameter clamp/application logic.
- `sim/physics/thruster_param_loader.py`: JSON IO and public loader facade.

Dev-OS compatibility:

- `tools/dev_os_compat_python_runtime.py`: runtime Python and MuJoCo import
  checks.
- `tools/dev_os_compat_viewer.py`: mjpython/viewer/display checks.
- `tools/dev_os_compat_runtime.py`: compatibility exports.

Axis RC metrics:

- `tools/axis_rc_metric_math.py`: quaternion, RMS, mean, and finite-value
  helpers.
- `tools/axis_rc_summary_metrics.py`: phase summary metrics.
- `tools/axis_rc_health_metrics.py`: health gates and sign-pair checks.
- `tools/axis_rc_metrics.py`: compatibility exports.

Contract audit:

- `tools/audit_code_contract_paths.py`: added the split SITL vertical estimate
  evidence path.
- `tools/audit_code_contract_runtime_checks.py`: points the Bar30 frontend
  match evidence at `bridge/ros2_state_sitl_vertical.py`.

## Contract Notes

- Bar30/SITL frontend-match logic still calls
  `sitl_depth_m_for_frontend_match(pressure_pa)`.
- JSON payload still feeds ArduSub through `position.z`, not a pressure field.
- `SimStackLaunchMixin` still exposes `_start_sim_stack`,
  `_restart_sim_stack_after_mavros_mode_change`, and the launch argument helper
  methods used by GUI controls.
- Thruster parameter clamp ranges and JSON keys are unchanged.
- `check_dev_os_compat.py --headless --target-os ubuntu` still returns the same
  pass/warn shape.
- Axis RC summary and health payload field names are unchanged.

## Validation

```text
PYTHONPATH=sim/current /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  <vertical surface smoke>

PYTHONPATH=sim/current /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  <SimStackLaunchMixin surface smoke>

PYTHONPATH=sim/current python3 <thruster_params.json loader smoke>
PYTHONPATH=sim/current/tools python3 <axis RC metrics smoke>

python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
python3 -m compileall -q sim/current uuv_control_gui.py

PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_source_audit_vertical_gui_thruster_devos_axis_split_2

python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/sim/current \
  --fetch --refresh-version

/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  sim/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_vertical_gui_thruster_devos_axis_split_2 \
  --simulate-s 0

python3 sim/current/tools/refactor_inventory.py --limit 30
git diff --check
```

Observed status:

```text
vertical surface: PASS
GUI launch surface: PASS
thruster param smoke: PASS
axis RC metrics smoke: PASS
dev-os compatibility: fail=0 pass=16 warn=2
compileall: PASS
source audit: fail=0 pass=11 warn=5
runtime_readiness_policy=PASS
thruster-contract: OK
runtime freshness: PASS
physics static balance: net_down=+0.000 N
git diff --check: PASS
```

## Inventory Effect

Removed from the top 30 hotspot list:

- `bridge/ros2_state_vertical.py`
- `gui/sim_stack_launch_mixin.py`
- `sim/physics/thruster_param_loader.py`
- `tools/dev_os_compat_runtime.py`
- `tools/axis_rc_metrics.py`

Current top branch-heavy targets after this pass:

```text
tools/dev_os_compat_system.py        176 LOC / 23 branches
tools/audit_closed_loop_params.py    171 LOC / 16 branches
tools/refactor_inventory.py          170 LOC / 16 branches
tools/althold_diagnostics_node.py    170 LOC / 14 branches
gui/node_motion_callbacks.py         169 LOC / 16 branches
tools/verify_ardusub_thruster_contract.py 162 LOC / 18 branches
bridge/ros2_bridge_runtime.py        156 LOC / 24 branches
```
