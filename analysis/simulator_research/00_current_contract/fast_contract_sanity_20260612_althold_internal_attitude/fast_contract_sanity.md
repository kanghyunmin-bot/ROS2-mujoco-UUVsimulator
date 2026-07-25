# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 26
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.216 |
| `rc_frame_contract` | PASS | 0.026 |
| `ros2_command_payload` | PASS | 0.020 |
| `bridge_live_imports` | PASS | 0.234 |
| `sitl_command_link_readiness` | PASS | 0.030 |
| `runtime_readiness_policy` | PASS | 0.023 |
| `gui_backend_selection` | PASS | 0.026 |
| `gui_readiness_contract` | PASS | 0.027 |
| `gui_ros_python_contract` | PASS | 0.984 |
| `gui_arm_mode_command_contract` | PASS | 0.021 |
| `gui_initial_depth_contract` | PASS | 0.023 |
| `initial_depth_auto_release_contract` | PASS | 0.019 |
| `gui_pilot_toggle_contract` | PASS | 0.028 |
| `axis_rc_health_contract` | PASS | 0.022 |
| `mavros_rcout_publish_policy` | PASS | 0.017 |
| `initial_hold_pose` | PASS | 0.067 |
| `real_start_measurements` | PASS | 0.049 |
| `physics_contract_geometry` | PASS | 0.044 |
| `model_runtime_setup` | PASS | 0.460 |
| `physics_runtime_hydrostatic` | PASS | 0.055 |
| `thruster_param_loader` | PASS | 0.054 |
| `odometry_publish_builders` | PASS | 0.050 |
| `ros2_sitl_command_override` | PASS | 0.018 |
| `ping360_stl_io` | PASS | 0.023 |
| `source_contract_audit` | PASS | 0.428 |
| `dev_os_compat_headless` | PASS | 0.295 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
