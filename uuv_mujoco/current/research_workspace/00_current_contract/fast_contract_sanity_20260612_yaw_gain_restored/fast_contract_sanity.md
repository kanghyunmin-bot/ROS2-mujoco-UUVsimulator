# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 19
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.214 |
| `rc_frame_contract` | PASS | 0.029 |
| `ros2_command_payload` | PASS | 0.028 |
| `bridge_live_imports` | PASS | 0.249 |
| `sitl_command_link_readiness` | PASS | 0.030 |
| `runtime_readiness_policy` | PASS | 0.022 |
| `gui_backend_selection` | PASS | 0.026 |
| `gui_readiness_contract` | PASS | 0.029 |
| `gui_initial_depth_contract` | PASS | 0.023 |
| `initial_hold_pose` | PASS | 0.068 |
| `real_start_measurements` | PASS | 0.053 |
| `physics_contract_geometry` | PASS | 0.049 |
| `model_runtime_setup` | PASS | 0.557 |
| `physics_runtime_hydrostatic` | PASS | 0.065 |
| `thruster_param_loader` | PASS | 0.060 |
| `odometry_publish_builders` | PASS | 0.060 |
| `ros2_sitl_command_override` | PASS | 0.019 |
| `source_contract_audit` | PASS | 0.461 |
| `dev_os_compat_headless` | PASS | 0.297 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
