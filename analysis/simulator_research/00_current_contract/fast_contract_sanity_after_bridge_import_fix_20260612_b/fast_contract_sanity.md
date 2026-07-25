# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 19
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.151 |
| `rc_frame_contract` | PASS | 0.022 |
| `ros2_command_payload` | PASS | 0.016 |
| `bridge_live_imports` | PASS | 0.104 |
| `sitl_command_link_readiness` | PASS | 0.028 |
| `runtime_readiness_policy` | PASS | 0.019 |
| `gui_backend_selection` | PASS | 0.022 |
| `gui_readiness_contract` | PASS | 0.024 |
| `gui_initial_depth_contract` | PASS | 0.020 |
| `initial_hold_pose` | PASS | 0.059 |
| `real_start_measurements` | PASS | 0.049 |
| `physics_contract_geometry` | PASS | 0.038 |
| `model_runtime_setup` | PASS | 0.406 |
| `physics_runtime_hydrostatic` | PASS | 0.048 |
| `thruster_param_loader` | PASS | 0.045 |
| `odometry_publish_builders` | PASS | 0.042 |
| `ros2_sitl_command_override` | PASS | 0.015 |
| `source_contract_audit` | PASS | 0.463 |
| `dev_os_compat_headless` | PASS | 0.312 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
