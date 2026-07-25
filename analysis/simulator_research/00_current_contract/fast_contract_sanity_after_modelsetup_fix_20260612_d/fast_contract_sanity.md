# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 17
- fail: 1

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.156 |
| `rc_frame_contract` | PASS | 0.022 |
| `ros2_command_payload` | PASS | 0.018 |
| `sitl_command_link_readiness` | PASS | 0.036 |
| `runtime_readiness_policy` | PASS | 0.022 |
| `gui_backend_selection` | PASS | 0.025 |
| `gui_readiness_contract` | PASS | 0.026 |
| `gui_initial_depth_contract` | PASS | 0.022 |
| `initial_hold_pose` | PASS | 0.044 |
| `real_start_measurements` | PASS | 0.044 |
| `physics_contract_geometry` | PASS | 0.041 |
| `model_runtime_setup` | FAIL | 0.389 |
| `physics_runtime_hydrostatic` | PASS | 0.059 |
| `thruster_param_loader` | PASS | 0.050 |
| `odometry_publish_builders` | PASS | 0.054 |
| `ros2_sitl_command_override` | PASS | 0.016 |
| `source_contract_audit` | PASS | 0.358 |
| `dev_os_compat_headless` | PASS | 0.198 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
