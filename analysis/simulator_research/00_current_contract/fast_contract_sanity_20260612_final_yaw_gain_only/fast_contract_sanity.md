# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 19
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.204 |
| `rc_frame_contract` | PASS | 0.025 |
| `ros2_command_payload` | PASS | 0.019 |
| `bridge_live_imports` | PASS | 0.259 |
| `sitl_command_link_readiness` | PASS | 0.033 |
| `runtime_readiness_policy` | PASS | 0.025 |
| `gui_backend_selection` | PASS | 0.028 |
| `gui_readiness_contract` | PASS | 0.030 |
| `gui_initial_depth_contract` | PASS | 0.026 |
| `initial_hold_pose` | PASS | 0.062 |
| `real_start_measurements` | PASS | 0.053 |
| `physics_contract_geometry` | PASS | 0.050 |
| `model_runtime_setup` | PASS | 0.552 |
| `physics_runtime_hydrostatic` | PASS | 0.065 |
| `thruster_param_loader` | PASS | 0.065 |
| `odometry_publish_builders` | PASS | 0.060 |
| `ros2_sitl_command_override` | PASS | 0.020 |
| `source_contract_audit` | PASS | 0.456 |
| `dev_os_compat_headless` | PASS | 0.330 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
