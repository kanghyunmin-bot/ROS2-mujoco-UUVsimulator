# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 26
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.205 |
| `rc_frame_contract` | PASS | 0.037 |
| `ros2_command_payload` | PASS | 0.020 |
| `bridge_live_imports` | PASS | 0.232 |
| `sitl_command_link_readiness` | PASS | 0.030 |
| `runtime_readiness_policy` | PASS | 0.024 |
| `gui_backend_selection` | PASS | 0.027 |
| `gui_readiness_contract` | PASS | 0.030 |
| `gui_ros_python_contract` | PASS | 0.901 |
| `gui_arm_mode_command_contract` | PASS | 0.031 |
| `gui_initial_depth_contract` | PASS | 0.031 |
| `initial_depth_auto_release_contract` | PASS | 0.028 |
| `gui_pilot_toggle_contract` | PASS | 0.044 |
| `axis_rc_health_contract` | PASS | 0.030 |
| `mavros_rcout_publish_policy` | PASS | 0.022 |
| `initial_hold_pose` | PASS | 0.079 |
| `real_start_measurements` | PASS | 0.054 |
| `physics_contract_geometry` | PASS | 0.048 |
| `model_runtime_setup` | PASS | 0.513 |
| `physics_runtime_hydrostatic` | PASS | 0.066 |
| `thruster_param_loader` | PASS | 0.060 |
| `odometry_publish_builders` | PASS | 0.056 |
| `ros2_sitl_command_override` | PASS | 0.020 |
| `ping360_stl_io` | PASS | 0.023 |
| `source_contract_audit` | PASS | 0.436 |
| `dev_os_compat_headless` | PASS | 0.260 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
