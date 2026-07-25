# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 18
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.180 |
| `rc_frame_contract` | PASS | 0.023 |
| `ros2_command_payload` | PASS | 0.179 |
| `bridge_live_imports` | PASS | 0.135 |
| `sitl_command_link_readiness` | PASS | 0.033 |
| `runtime_readiness_policy` | PASS | 0.043 |
| `gui_backend_selection` | PASS | 0.026 |
| `gui_readiness_contract` | PASS | 0.030 |
| `gui_initial_depth_contract` | PASS | 0.031 |
| `initial_hold_pose` | PASS | 0.071 |
| `real_start_measurements` | PASS | 0.066 |
| `physics_contract_geometry` | PASS | 0.061 |
| `model_runtime_setup` | PASS | 0.554 |
| `physics_runtime_hydrostatic` | PASS | 0.056 |
| `thruster_param_loader` | PASS | 0.059 |
| `odometry_publish_builders` | PASS | 0.054 |
| `ros2_sitl_command_override` | PASS | 0.015 |
| `source_contract_audit` | PASS | 0.542 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
