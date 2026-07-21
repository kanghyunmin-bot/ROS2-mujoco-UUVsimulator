# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 24
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.191 |
| `rc_frame_contract` | PASS | 0.025 |
| `ros2_command_payload` | PASS | 0.024 |
| `bridge_live_imports` | PASS | 0.222 |
| `sitl_command_link_readiness` | PASS | 0.030 |
| `runtime_readiness_policy` | PASS | 0.023 |
| `gui_backend_selection` | PASS | 0.025 |
| `gui_readiness_contract` | PASS | 0.036 |
| `gui_ros_python_contract` | PASS | 0.585 |
| `gui_arm_mode_command_contract` | PASS | 0.022 |
| `gui_initial_depth_contract` | PASS | 0.026 |
| `gui_pilot_toggle_contract` | PASS | 0.030 |
| `axis_rc_health_contract` | PASS | 0.023 |
| `initial_hold_pose` | PASS | 0.073 |
| `real_start_measurements` | PASS | 0.052 |
| `physics_contract_geometry` | PASS | 0.048 |
| `model_runtime_setup` | PASS | 0.501 |
| `physics_runtime_hydrostatic` | PASS | 0.055 |
| `thruster_param_loader` | PASS | 0.051 |
| `odometry_publish_builders` | PASS | 0.049 |
| `ros2_sitl_command_override` | PASS | 0.018 |
| `ping360_stl_io` | PASS | 0.022 |
| `source_contract_audit` | PASS | 0.428 |
| `dev_os_compat_headless` | PASS | 0.296 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
