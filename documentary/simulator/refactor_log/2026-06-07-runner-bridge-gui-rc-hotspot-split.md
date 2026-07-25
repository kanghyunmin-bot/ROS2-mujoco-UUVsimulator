# Runner, Bridge Public API, GUI Arm/Mode, and RC Output Hotspot Split

Date: 2026-06-07

## Scope

Continue reducing active-runtime spaghetti under `sim/current` without
changing controller-parity contracts, plant input semantics, RC channel
mapping, ArduPilot source, or MuJoCo physics coefficients.

## Changed Files

Roll stability sweep:

- `tools/roll_stability_launch.py`: reset, launcher start, readiness polling,
  and teardown helpers.
- `tools/roll_stability_probe_runtime.py`: ROS2 probe execution and servo-output
  validity gates.
- `tools/roll_stability_summary.py`: summary CSV/JSON writer.
- `tools/roll_stability_runner.py`: candidate orchestration facade.

ROS2 bridge public API:

- `bridge/ros2_bridge_servo_api.py`: SITL servo handler/status plus force-next
  publish and odometry reset helpers.
- `bridge/ros2_bridge_spin_publish.py`: simulator-loop `spin_once()` and
  `publish()` helpers.
- `bridge/ros2_bridge_shutdown.py`: bridge shutdown helper.
- `bridge/ros2_bridge_public_api.py`: compatibility facade.

GUI arm/mode command path:

- `gui/node_arm_commands.py`: arm/disarm retry, gate, service, and response
  handling.
- `gui/node_mode_commands.py`: mode retry, ALT_HOLD initial-depth release gate,
  service, and response handling.
- `gui/node_arm_mode_commands.py`: compatibility facade.

Bridge RC path:

- `bridge/ros2_rc_override_input.py`: `/mavros/rc/override` input, MAVLink
  forwarding, normalized fallback, and `/mavros/rc/in` mirror.
- `bridge/ros2_replay_rcout.py`: replay RCOUT plant-input injection path.
- `bridge/ros2_rcout_telemetry.py`: SITL servo-output to MAVROS `RCOut`
  telemetry mirror, including sensor-replay real-time stamp handling.
- `bridge/ros2_rc_output_commands.py`: compatibility facade.

## Contract Notes

- `roll_stability_sweep.py --help` still works without importing ROS2/rclpy.
- Roll-stability active-stimulus safety still rejects runs with no active servo
  output.
- `Ros2Bridge` still exposes the same public methods:
  `set_sitl_servo_handler`, `set_replay_rcout_handler`, `sitl_vehicle_armed`,
  `sitl_vehicle_mode`, `spin_once`, `publish`, `force_next_publish`,
  `reset_odometry`, and `shutdown`.
- `UuvGuiNode` still exposes `_send_arm_request`, `arm`, `_on_arm_response`,
  `set_mode`, `_send_mode_request`, and `_on_mode_response`.
- The bridge RC command surface still exposes `_on_mavros_rc_override`,
  `_handle_replay_rcout_channels`, `_on_replay_rcout_override`, and
  `_on_sitl_servo_output_for_ros`.
- The split preserves the important observation-point boundary:
  RC override input, replay RCOUT plant injection, and SITL `RCOut` telemetry
  mirror are now in separate files instead of sharing one implementation file.

## Validation

```text
python3 -m py_compile \
  sim/current/tools/roll_stability_sweep.py \
  sim/current/tools/roll_stability_runner.py \
  sim/current/tools/roll_stability_launch.py \
  sim/current/tools/roll_stability_probe_runtime.py \
  sim/current/tools/roll_stability_summary.py

PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/roll_stability_sweep.py --help

PYTHONPATH=sim/current /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  <surface import smokes for Ros2Bridge, UuvGuiNode, and bridge RC callbacks>

python3 -m compileall -q sim/current uuv_control_gui.py

PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_source_audit_roll_bridge_gui_rc_split

python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/sim/current \
  --fetch --refresh-version

/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  sim/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_roll_bridge_gui_rc_split \
  --simulate-s 0

python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
python3 sim/current/tools/refactor_inventory.py --limit 30
git diff --check
```

Observed status:

```text
roll_stability --help: PASS
roll_stability runner surface: PASS
bridge public API surface: PASS
GUI arm/mode surface: PASS
bridge RC surface: PASS
compileall: PASS
source audit: fail=0 pass=11 warn=5
runtime_readiness_policy=PASS
thruster-contract: OK
runtime freshness: PASS
physics static balance: net_down=+0.000 N
dev-os compatibility: fail=0 pass=16 warn=2
git diff --check: PASS
```

Dev-os warnings remain external environment state:

```text
docker_daemon: Docker daemon socket unavailable
ros2_env: ROS2 not sourced in current shell
```

## Inventory Effect

Removed from the top 30 hotspot list:

- `tools/roll_stability_runner.py`
- `bridge/ros2_bridge_public_api.py`
- `gui/node_arm_mode_commands.py`
- `bridge/ros2_rc_output_commands.py`

Current top branch-heavy runtime targets after this pass:

```text
tools/dev_os_compat_system.py       176 LOC / 23 branches
bridge/ros2_state_vertical.py       173 LOC / 22 branches
gui/sim_stack_launch_mixin.py       171 LOC / 23 branches
sim/physics/thruster_param_loader.py 168 LOC / 24 branches
tools/dev_os_compat_runtime.py      167 LOC / 28 branches
tools/axis_rc_metrics.py            156 LOC / 31 branches
bridge/ros2_bridge_runtime.py       156 LOC / 24 branches
```
