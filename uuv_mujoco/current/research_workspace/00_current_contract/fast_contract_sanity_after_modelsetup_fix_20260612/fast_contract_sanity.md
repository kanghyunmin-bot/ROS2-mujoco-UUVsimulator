# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 17
- fail: 1

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.159 |
| `rc_frame_contract` | PASS | 0.021 |
| `ros2_command_payload` | PASS | 0.017 |
| `sitl_command_link_readiness` | PASS | 0.028 |
| `runtime_readiness_policy` | PASS | 0.021 |
| `gui_backend_selection` | PASS | 0.024 |
| `gui_readiness_contract` | PASS | 0.026 |
| `gui_initial_depth_contract` | PASS | 0.022 |
| `initial_hold_pose` | PASS | 0.043 |
| `real_start_measurements` | PASS | 0.045 |
| `physics_contract_geometry` | PASS | 0.042 |
| `model_runtime_setup` | FAIL | 0.051 |
| `physics_runtime_hydrostatic` | PASS | 0.051 |
| `thruster_param_loader` | PASS | 0.051 |
| `odometry_publish_builders` | PASS | 0.053 |
| `ros2_sitl_command_override` | PASS | 0.017 |
| `source_contract_audit` | PASS | 0.409 |
| `dev_os_compat_headless` | PASS | 0.367 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
