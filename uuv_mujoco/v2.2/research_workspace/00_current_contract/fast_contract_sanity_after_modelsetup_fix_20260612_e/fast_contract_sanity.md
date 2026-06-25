# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 18
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.166 |
| `rc_frame_contract` | PASS | 0.023 |
| `ros2_command_payload` | PASS | 0.017 |
| `sitl_command_link_readiness` | PASS | 0.027 |
| `runtime_readiness_policy` | PASS | 0.021 |
| `gui_backend_selection` | PASS | 0.023 |
| `gui_readiness_contract` | PASS | 0.026 |
| `gui_initial_depth_contract` | PASS | 0.021 |
| `initial_hold_pose` | PASS | 0.055 |
| `real_start_measurements` | PASS | 0.045 |
| `physics_contract_geometry` | PASS | 0.041 |
| `model_runtime_setup` | PASS | 0.391 |
| `physics_runtime_hydrostatic` | PASS | 0.058 |
| `thruster_param_loader` | PASS | 0.050 |
| `odometry_publish_builders` | PASS | 0.048 |
| `ros2_sitl_command_override` | PASS | 0.018 |
| `source_contract_audit` | PASS | 0.371 |
| `dev_os_compat_headless` | PASS | 0.292 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
