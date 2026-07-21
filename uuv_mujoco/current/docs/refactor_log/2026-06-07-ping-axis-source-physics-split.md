# Ping360, Axis RC Node, Source Audit, and Physics Factory Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current`.

## Changed

- Split Ping360 polar image rendering out of
  `bridge/ros2_ping360_messages.py`:
  - `bridge/ping360_image_renderer.py`
- Split the axis RC validation node out of `tools/axis_rc_node.py`:
  - `tools/axis_rc_node_callbacks.py`
  - `tools/axis_rc_node_control.py`
  - `tools/axis_rc_node_services.py`
- Split firmware source-audit checks:
  - `tools/audit_code_contract_firmware_json_checks.py`
  - `tools/audit_code_contract_firmware_rc_checks.py`
- Split source identity checks:
  - `tools/audit_code_contract_runtime_identity.py`
  - `tools/audit_code_contract_ardupilot_identity.py`
- Split runtime physics factory wiring:
  - `sim/runtime/physics_runtime_factory_context.py`
  - `sim/runtime/physics_runtime_factory_hydro.py`
  - `sim/runtime/physics_runtime_factory_thrusters.py`

## Contract Preserved

- `bridge.ros2_ping360_messages.Ping360ImageRenderer` is still exported from
  the same public module.
- Ping360 image lookup cache still clamps image size to `[128, 1200]` and
  reuses the cached lookup for the same `(size, samples)` key.
- Axis RC node still owns the same ROS topics, service clients, callback names,
  publishing methods, and helper methods.
- `axis_rc_override_check.py --help` still avoids importing ROS2/rclpy.
- Source audit check ids, ordering, metadata keys, and pass/warn/fail counts are
  unchanged.
- `create_runtime_physics_setup()` keeps the same public function signature and
  physics wiring order.

## Validation

- Ping360 renderer smoke:
  - public import preserved
  - lookup keys: `angle_idx`, `mask`, `range_idx`, `ring_mask`, `rr`,
    `spoke_mask`
  - cache reuse confirmed
- Axis RC node surface:
  - expected methods: `20`
  - missing methods: `[]`
- Source identity check ids:
  - `active_runtime_alias_current`
  - `ardupilot_source_tag`
  - `top_level_ardupilot_gitlink`
- Source audit:
  - checks: `16`
  - `PASS=11`, `WARN=5`, `FAIL=0`
- Physics static audit:
  - `net_down=+0.000N`
  - `required_scale=1.000000`
- Runtime freshness:
  - `current -> v2.2`
  - local `HEAD` equals `origin/uuv_sim`

## Results

- Removed these previous top hotspots from the active inventory:
  - `bridge/ros2_ping360_messages.py`
  - `tools/audit_code_contract_firmware_checks.py`
  - `tools/audit_code_contract_source_identity.py`
  - `tools/axis_rc_node.py`
  - `sim/runtime/physics_runtime_factory.py`
- Current largest file reported by the inventory is now
  `sim/transport/mavlink_telemetry_observer.py` at `197 LOC`.
