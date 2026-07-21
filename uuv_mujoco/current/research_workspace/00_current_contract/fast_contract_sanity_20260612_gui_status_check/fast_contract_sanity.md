# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 26
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.383 |
| `rc_frame_contract` | PASS | 0.049 |
| `ros2_command_payload` | PASS | 0.042 |
| `bridge_live_imports` | PASS | 0.480 |
| `sitl_command_link_readiness` | PASS | 0.084 |
| `runtime_readiness_policy` | PASS | 0.067 |
| `gui_backend_selection` | PASS | 0.069 |
| `gui_readiness_contract` | PASS | 0.088 |
| `gui_ros_python_contract` | PASS | 1.538 |
| `gui_arm_mode_command_contract` | PASS | 0.038 |
| `gui_initial_depth_contract` | PASS | 0.044 |
| `initial_depth_auto_release_contract` | PASS | 0.035 |
| `gui_pilot_toggle_contract` | PASS | 0.051 |
| `axis_rc_health_contract` | PASS | 0.041 |
| `mavros_rcout_publish_policy` | PASS | 0.034 |
| `initial_hold_pose` | PASS | 0.116 |
| `real_start_measurements` | PASS | 0.097 |
| `physics_contract_geometry` | PASS | 0.089 |
| `model_runtime_setup` | PASS | 1.014 |
| `physics_runtime_hydrostatic` | PASS | 0.100 |
| `thruster_param_loader` | PASS | 0.103 |
| `odometry_publish_builders` | PASS | 0.092 |
| `ros2_sitl_command_override` | PASS | 0.034 |
| `ping360_stl_io` | PASS | 0.043 |
| `source_contract_audit` | PASS | 1.054 |
| `dev_os_compat_headless` | PASS | 0.430 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
