# Visual And Lifecycle Hotspot Split

Date: 2026-06-10

## Scope

This pass reduces high-scoring structural hotspots in display and lifecycle
code without changing controller, sensor, RC, thruster, or plant-input
contracts.

## Files

- `bridge/ping360_image_renderer.py`
- `bridge/ping360_image_lookup.py`
- `bridge/ping360_image_layers.py`
- `gui/control_draw_mixin.py`
- `gui/control_attitude_draw.py`
- `gui/control_depth_draw.py`
- `sim/runtime/viewer_scene_builder.py`
- `sim/runtime/viewer_scene_primitives.py`
- `sim/runtime/viewer_scene_bubbles.py`
- `gui/ping360_view_mixin.py`
- `gui/ping360_view_process.py`
- `gui/ping360_view_status.py`
- `bridge/ros2_bridge_shutdown_steps.py`
- `bridge/ros2_bridge_shutdown_thread.py`
- `bridge/ros2_bridge_shutdown_sitl.py`
- `bridge/ros2_bridge_shutdown_ros.py`
- `bridge/sitl_mavlink_servo_heartbeat.py`
- `bridge/sitl_mavlink_servo_heartbeat_target.py`
- `gui/sim_stack_reset_mixin.py`
- `gui/sim_stack_reset_thread.py`
- `gui/sim_stack_reset_worker.py`
- `gui/sim_stack_reset_commands.py`

## Result

Removed these files from the top structural-complexity inventory:

- `bridge/ping360_image_renderer.py`
- `gui/control_draw_mixin.py`
- `sim/runtime/viewer_scene_builder.py`
- `gui/ping360_view_mixin.py`
- `bridge/ros2_bridge_shutdown_steps.py`
- `bridge/sitl_mavlink_servo_heartbeat.py`
- `gui/sim_stack_reset_mixin.py`

The pass also removed avoidable `gui.runtime` imports from visual/reset helper
paths, so these modules can be imported without pulling `rclpy` into basic
smoke tests.

## Validation

```text
compileall: PASS
ping360_renderer_split_smoke=PASS
control_draw_split_import=PASS
viewer_scene_split_import=PASS
ping360_view_split_import=PASS
ros2_shutdown_steps_split_import=PASS
servo_heartbeat_split_import=PASS
sim_stack_reset_split_import=PASS
gui_readiness_contract=PASS
gui_backend_selection=PASS
runtime_readiness_policy=PASS
rc_frame_contract=PASS
verify_ardusub_thruster_contract.py --quiet: OK
audit_code_contract_sources.py: {"fail": 0, "pass": 15, "warn": 5}
audit_closed_loop_contract.py: real_vs_sitl_mismatches={}, missing_sitl_params=[]
```
