# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 19
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.196 |
| `rc_frame_contract` | PASS | 0.024 |
| `ros2_command_payload` | PASS | 0.031 |
| `bridge_live_imports` | PASS | 0.228 |
| `sitl_command_link_readiness` | PASS | 0.030 |
| `runtime_readiness_policy` | PASS | 0.023 |
| `gui_backend_selection` | PASS | 0.036 |
| `gui_readiness_contract` | PASS | 0.029 |
| `gui_initial_depth_contract` | PASS | 0.023 |
| `initial_hold_pose` | PASS | 0.069 |
| `real_start_measurements` | PASS | 0.049 |
| `physics_contract_geometry` | PASS | 0.047 |
| `model_runtime_setup` | PASS | 0.498 |
| `physics_runtime_hydrostatic` | PASS | 0.057 |
| `thruster_param_loader` | PASS | 0.056 |
| `odometry_publish_builders` | PASS | 0.052 |
| `ros2_sitl_command_override` | PASS | 0.020 |
| `source_contract_audit` | PASS | 0.466 |
| `dev_os_compat_headless` | PASS | 0.301 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
