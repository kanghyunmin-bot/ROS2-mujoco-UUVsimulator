# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 17
- fail: 1

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.204 |
| `rc_frame_contract` | PASS | 0.024 |
| `ros2_command_payload` | PASS | 0.020 |
| `sitl_command_link_readiness` | PASS | 0.032 |
| `runtime_readiness_policy` | PASS | 0.024 |
| `gui_backend_selection` | PASS | 0.023 |
| `gui_readiness_contract` | PASS | 0.026 |
| `gui_initial_depth_contract` | PASS | 0.021 |
| `initial_hold_pose` | PASS | 0.063 |
| `real_start_measurements` | PASS | 0.045 |
| `physics_contract_geometry` | PASS | 0.041 |
| `model_runtime_setup` | FAIL | 0.488 |
| `physics_runtime_hydrostatic` | PASS | 0.051 |
| `thruster_param_loader` | PASS | 0.048 |
| `odometry_publish_builders` | PASS | 0.045 |
| `ros2_sitl_command_override` | PASS | 0.017 |
| `source_contract_audit` | PASS | 0.365 |
| `dev_os_compat_headless` | PASS | 0.306 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
