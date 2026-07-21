# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 18
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.180 |
| `rc_frame_contract` | PASS | 0.025 |
| `ros2_command_payload` | PASS | 0.168 |
| `bridge_live_imports` | PASS | 0.142 |
| `sitl_command_link_readiness` | PASS | 0.032 |
| `runtime_readiness_policy` | PASS | 0.025 |
| `gui_backend_selection` | PASS | 0.028 |
| `gui_readiness_contract` | PASS | 0.032 |
| `gui_initial_depth_contract` | PASS | 0.025 |
| `initial_hold_pose` | PASS | 0.059 |
| `real_start_measurements` | PASS | 0.059 |
| `physics_contract_geometry` | PASS | 0.062 |
| `model_runtime_setup` | PASS | 0.666 |
| `physics_runtime_hydrostatic` | PASS | 0.058 |
| `thruster_param_loader` | PASS | 0.060 |
| `odometry_publish_builders` | PASS | 0.053 |
| `ros2_sitl_command_override` | PASS | 0.015 |
| `source_contract_audit` | PASS | 0.501 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
