# SITL Transport, GUI RC, and Axis CLI Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current`.

## Changed

- Split `bridge/sitl_transport.py` method bindings into focused mixins:
  - `bridge/sitl_transport_state_bindings.py`
  - `bridge/sitl_transport_handler_bindings.py`
  - `bridge/sitl_transport_command_bindings.py`
  - `bridge/sitl_transport_mavlink_bindings.py`
  - `bridge/sitl_transport_extnav_bindings.py`
  - `bridge/sitl_transport_json_bindings.py`
  - `bridge/sitl_transport_bindings.py`
- Kept `bridge.sitl_transport.SitlTransport` as the public runtime class.
- Split `gui/gui_rc_helpers.py` into focused RC helper modules:
  - `gui/gui_rc_pwm.py`
  - `gui/gui_rc_althold.py`
  - `gui/gui_rc_axes.py`
  - `gui/gui_rc_messages.py`
- Kept `gui.gui_rc_helpers` as the public compatibility facade used by
  `gui.helpers`, GUI node publishers, and replay loading.
- Split `tools/axis_rc_override_check.py` into CLI argument, vehicle/phase
  sequence, and report modules:
  - `tools/axis_rc_cli_args.py`
  - `tools/axis_rc_sequence.py`
  - `tools/axis_rc_report.py`

## Contract Preserved

- `SitlTransport` still exposes the same JSON servo/sensor, MAVLink,
  ExternalNav, command, status, replay, handler, and lifecycle method names.
- `SitlTransport.__init__()` still initializes JSON servo transport, SITL
  control state, MAVLink transport, ExternalNav state, then connects SITL and
  MAVLink in the same order.
- GUI RC PWM mapping remains `axis=-1/0/1 -> 1100/1500/1900` with the current
  GUI span.
- GUI RC override messages still emit 18 channels, neutral primary channels,
  MAVLink no-change markers on extensions, and release markers for primary
  release.
- `axis_rc_override_check.py --help` still runs without importing ROS2/rclpy.

## Validation

- `SitlTransport` binding surface check:
  - expected bindings: `84`
  - missing bindings: `[]`
- GUI RC sample contract:
  - `axis_to_pwm(-1, 0, 1) -> 1100, 1500, 1900`
  - `make_rc_override_message(...)` returns 18 channels
  - `padded_rc_channels(..., sanitize_override_markers=True)` zeros markers
- `python3 uuv_mujoco/current/tools/axis_rc_override_check.py --help`
- `python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py`
- `PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_transport_gui_axis_split`
- `python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py`
- `python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet`
- `python3 uuv_mujoco/current/tools/check_runtime_freshness.py --workspace /Users/kanghyunmin/Desktop/uuv_sim --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current --fetch --refresh-version`

## Results

- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: `PASS`.
- Thruster contract: `OK`.
- Runtime freshness: `PASS`; `current -> v2.2`; local `HEAD` equals
  `origin/uuv_sim`.
- `bridge/sitl_transport.py`, `gui/gui_rc_helpers.py`, and
  `tools/axis_rc_override_check.py` are removed from the top hotspot list.
