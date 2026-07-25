# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 22
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.183 |
| `rc_frame_contract` | PASS | 0.025 |
| `ros2_command_payload` | PASS | 0.018 |
| `bridge_live_imports` | PASS | 0.239 |
| `sitl_command_link_readiness` | PASS | 0.029 |
| `runtime_readiness_policy` | PASS | 0.023 |
| `gui_backend_selection` | PASS | 0.026 |
| `gui_readiness_contract` | PASS | 0.030 |
| `gui_initial_depth_contract` | PASS | 0.025 |
| `gui_pilot_toggle_contract` | PASS | 0.028 |
| `axis_rc_health_contract` | PASS | 0.022 |
| `initial_hold_pose` | PASS | 0.071 |
| `real_start_measurements` | PASS | 0.049 |
| `physics_contract_geometry` | PASS | 0.045 |
| `model_runtime_setup` | PASS | 0.493 |
| `physics_runtime_hydrostatic` | PASS | 0.059 |
| `thruster_param_loader` | PASS | 0.056 |
| `odometry_publish_builders` | PASS | 0.052 |
| `ros2_sitl_command_override` | PASS | 0.019 |
| `ping360_stl_io` | PASS | 0.024 |
| `source_contract_audit` | PASS | 0.461 |
| `dev_os_compat_headless` | PASS | 0.311 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
