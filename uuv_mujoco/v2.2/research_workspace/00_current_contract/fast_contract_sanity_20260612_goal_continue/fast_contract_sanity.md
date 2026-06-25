# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 18
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.414 |
| `rc_frame_contract` | PASS | 0.021 |
| `ros2_command_payload` | PASS | 0.184 |
| `bridge_live_imports` | PASS | 0.131 |
| `sitl_command_link_readiness` | PASS | 0.029 |
| `runtime_readiness_policy` | PASS | 0.022 |
| `gui_backend_selection` | PASS | 0.026 |
| `gui_readiness_contract` | PASS | 0.028 |
| `gui_initial_depth_contract` | PASS | 0.023 |
| `initial_hold_pose` | PASS | 0.054 |
| `real_start_measurements` | PASS | 0.055 |
| `physics_contract_geometry` | PASS | 0.053 |
| `model_runtime_setup` | PASS | 0.494 |
| `physics_runtime_hydrostatic` | PASS | 0.067 |
| `thruster_param_loader` | PASS | 0.072 |
| `odometry_publish_builders` | PASS | 0.057 |
| `ros2_sitl_command_override` | PASS | 0.016 |
| `source_contract_audit` | PASS | 0.546 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
