# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 17
- fail: 1

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.151 |
| `rc_frame_contract` | PASS | 0.021 |
| `ros2_command_payload` | PASS | 0.016 |
| `sitl_command_link_readiness` | PASS | 0.024 |
| `runtime_readiness_policy` | PASS | 0.018 |
| `gui_backend_selection` | PASS | 0.021 |
| `gui_readiness_contract` | PASS | 0.023 |
| `gui_initial_depth_contract` | PASS | 0.027 |
| `initial_hold_pose` | PASS | 0.061 |
| `real_start_measurements` | PASS | 0.042 |
| `physics_contract_geometry` | PASS | 0.044 |
| `model_runtime_setup` | FAIL | 0.076 |
| `physics_runtime_hydrostatic` | PASS | 0.046 |
| `thruster_param_loader` | PASS | 0.044 |
| `odometry_publish_builders` | PASS | 0.041 |
| `ros2_sitl_command_override` | PASS | 0.014 |
| `source_contract_audit` | PASS | 0.456 |
| `dev_os_compat_headless` | PASS | 0.300 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
