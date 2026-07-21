# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 17
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.164 |
| `rc_frame_contract` | PASS | 0.023 |
| `ros2_command_payload` | PASS | 0.017 |
| `sitl_command_link_readiness` | PASS | 0.028 |
| `runtime_readiness_policy` | PASS | 0.020 |
| `gui_backend_selection` | PASS | 0.023 |
| `gui_readiness_contract` | PASS | 0.024 |
| `gui_initial_depth_contract` | PASS | 0.021 |
| `initial_hold_pose` | PASS | 0.083 |
| `real_start_measurements` | PASS | 0.043 |
| `physics_contract_geometry` | PASS | 0.039 |
| `physics_runtime_hydrostatic` | PASS | 0.048 |
| `thruster_param_loader` | PASS | 0.046 |
| `odometry_publish_builders` | PASS | 0.042 |
| `ros2_sitl_command_override` | PASS | 0.015 |
| `source_contract_audit` | PASS | 0.494 |
| `dev_os_compat_headless` | PASS | 0.800 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
