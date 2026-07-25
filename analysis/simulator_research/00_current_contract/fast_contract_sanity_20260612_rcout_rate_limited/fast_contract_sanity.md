# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 26
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.373 |
| `rc_frame_contract` | PASS | 0.051 |
| `ros2_command_payload` | PASS | 0.042 |
| `bridge_live_imports` | PASS | 0.449 |
| `sitl_command_link_readiness` | PASS | 0.082 |
| `runtime_readiness_policy` | PASS | 0.069 |
| `gui_backend_selection` | PASS | 0.073 |
| `gui_readiness_contract` | PASS | 0.092 |
| `gui_ros_python_contract` | PASS | 1.616 |
| `gui_arm_mode_command_contract` | PASS | 0.039 |
| `gui_initial_depth_contract` | PASS | 0.046 |
| `initial_depth_auto_release_contract` | PASS | 0.040 |
| `gui_pilot_toggle_contract` | PASS | 0.056 |
| `axis_rc_health_contract` | PASS | 0.046 |
| `mavros_rcout_publish_policy` | PASS | 0.038 |
| `initial_hold_pose` | PASS | 0.129 |
| `real_start_measurements` | PASS | 0.107 |
| `physics_contract_geometry` | PASS | 0.094 |
| `model_runtime_setup` | PASS | 0.989 |
| `physics_runtime_hydrostatic` | PASS | 0.095 |
| `thruster_param_loader` | PASS | 0.097 |
| `odometry_publish_builders` | PASS | 0.095 |
| `ros2_sitl_command_override` | PASS | 0.037 |
| `ping360_stl_io` | PASS | 0.049 |
| `source_contract_audit` | PASS | 1.081 |
| `dev_os_compat_headless` | PASS | 0.417 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
