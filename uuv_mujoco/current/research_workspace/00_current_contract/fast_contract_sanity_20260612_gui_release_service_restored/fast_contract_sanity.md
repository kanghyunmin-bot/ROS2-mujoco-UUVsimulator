# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 19
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.339 |
| `rc_frame_contract` | PASS | 0.056 |
| `ros2_command_payload` | PASS | 0.043 |
| `bridge_live_imports` | PASS | 0.450 |
| `sitl_command_link_readiness` | PASS | 0.082 |
| `runtime_readiness_policy` | PASS | 0.067 |
| `gui_backend_selection` | PASS | 0.063 |
| `gui_readiness_contract` | PASS | 0.073 |
| `gui_initial_depth_contract` | PASS | 0.075 |
| `initial_hold_pose` | PASS | 0.187 |
| `real_start_measurements` | PASS | 0.146 |
| `physics_contract_geometry` | PASS | 0.121 |
| `model_runtime_setup` | PASS | 0.805 |
| `physics_runtime_hydrostatic` | PASS | 0.108 |
| `thruster_param_loader` | PASS | 0.103 |
| `odometry_publish_builders` | PASS | 0.093 |
| `ros2_sitl_command_override` | PASS | 0.034 |
| `source_contract_audit` | PASS | 0.839 |
| `dev_os_compat_headless` | PASS | 0.450 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
