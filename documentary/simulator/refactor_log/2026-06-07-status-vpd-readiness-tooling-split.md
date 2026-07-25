# Status, Native VPD, Readiness, And Tooling Split

Date: 2026-06-07

Scope: active runtime `sim/current`, backed by `uuv_mujoco/v2.2`.

## Change

GUI simulator-stack status detection is split by responsibility:

- `gui/sim_stack_process_probe.py`
- `gui/sim_stack_status_logic.py`

Native `VISION_POSITION_DELTA` replay is split by replay step:

- `bridge/sitl_native_vpd_start.py`
- `bridge/sitl_native_vpd_send.py`
- `bridge/sitl_native_vpd_rate.py`
- `bridge/sitl_native_vpd_debug.py`

Runtime command-readiness labeling is split by policy stage:

- `sim/runtime/readiness_preflight_label.py`
- `sim/runtime/readiness_command_path_label.py`
- `sim/runtime/readiness_control_label.py`
- `sim/runtime/readiness_label_types.py`

Roll-stability and golden-control-loop tooling were split where repeated wait
or metric loops were hiding intent:

- `tools/roll_stability_command_wait.py`
- `tools/roll_stability_command_requests.py`
- `tools/control_loop_golden_thruster_rows.py`
- `tools/control_loop_golden_thruster_metrics.py`

## Verification

Commands run:

```text
python3 -m compileall -q sim/current uuv_control_gui.py
git diff --check
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/audit_code_contract_sources.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python sim/current/tools/physics_contract_audit.py --simulate-s 0.05
python3 sim/current/tools/refactor_inventory.py --root sim/current --format markdown --limit 25
```

Targeted smoke checks:

- `sim_stack_status_logic`: PASS.
- native VPD helper cursor/send/rate smoke: PASS.

Results:

- Runtime readiness policy: PASS.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- ArduSub thruster contract: OK.
- Static physics audit remains neutral: `net_down=+0.000N`,
  `required_scale=1.000000`, neutral drift about `+0.00001m`.

## Contract Notes

No controller-parity observation point, RC override frame semantics, native VPD
message fields, ArduPilot source, submodule pointer, or plant coefficient was
changed.  The remaining highest-risk hotspot is now `sim/contracts/rc_frames.py`;
it should be split with explicit regression checks because it is an input
contract boundary.
