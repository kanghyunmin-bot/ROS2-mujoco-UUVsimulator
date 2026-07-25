# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 26
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.202 |
| `rc_frame_contract` | PASS | 0.037 |
| `ros2_command_payload` | PASS | 0.021 |
| `bridge_live_imports` | PASS | 0.234 |
| `sitl_command_link_readiness` | PASS | 0.031 |
| `runtime_readiness_policy` | PASS | 0.029 |
| `gui_backend_selection` | PASS | 0.031 |
| `gui_readiness_contract` | PASS | 0.032 |
| `gui_ros_python_contract` | PASS | 1.013 |
| `gui_arm_mode_command_contract` | PASS | 0.024 |
| `gui_initial_depth_contract` | PASS | 0.030 |
| `initial_depth_auto_release_contract` | PASS | 0.023 |
| `gui_pilot_toggle_contract` | PASS | 0.033 |
| `axis_rc_health_contract` | PASS | 0.028 |
| `mavros_rcout_publish_policy` | PASS | 0.022 |
| `initial_hold_pose` | PASS | 0.069 |
| `real_start_measurements` | PASS | 0.051 |
| `physics_contract_geometry` | PASS | 0.046 |
| `model_runtime_setup` | PASS | 0.497 |
| `physics_runtime_hydrostatic` | PASS | 0.062 |
| `thruster_param_loader` | PASS | 0.059 |
| `odometry_publish_builders` | PASS | 0.051 |
| `ros2_sitl_command_override` | PASS | 0.018 |
| `ping360_stl_io` | PASS | 0.022 |
| `source_contract_audit` | PASS | 0.438 |
| `dev_os_compat_headless` | PASS | 0.300 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
