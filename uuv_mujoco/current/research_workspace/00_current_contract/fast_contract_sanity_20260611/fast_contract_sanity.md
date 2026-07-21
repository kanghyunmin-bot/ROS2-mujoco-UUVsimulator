# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 16
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.188 |
| `rc_frame_contract` | PASS | 0.022 |
| `ros2_command_payload` | PASS | 0.019 |
| `sitl_command_link_readiness` | PASS | 0.033 |
| `runtime_readiness_policy` | PASS | 0.022 |
| `gui_backend_selection` | PASS | 0.026 |
| `gui_readiness_contract` | PASS | 0.027 |
| `gui_initial_depth_contract` | PASS | 0.023 |
| `initial_hold_pose` | PASS | 0.166 |
| `real_start_measurements` | PASS | 0.047 |
| `physics_contract_geometry` | PASS | 0.043 |
| `physics_runtime_hydrostatic` | PASS | 0.069 |
| `thruster_param_loader` | PASS | 0.067 |
| `odometry_publish_builders` | PASS | 0.046 |
| `ros2_sitl_command_override` | PASS | 0.016 |
| `source_contract_audit` | PASS | 0.387 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
