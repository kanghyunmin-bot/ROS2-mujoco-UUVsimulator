# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 24
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.195 |
| `rc_frame_contract` | PASS | 0.024 |
| `ros2_command_payload` | PASS | 0.020 |
| `bridge_live_imports` | PASS | 0.232 |
| `sitl_command_link_readiness` | PASS | 0.029 |
| `runtime_readiness_policy` | PASS | 0.023 |
| `gui_backend_selection` | PASS | 0.025 |
| `gui_readiness_contract` | PASS | 0.037 |
| `gui_ros_python_contract` | PASS | 0.923 |
| `gui_arm_mode_command_contract` | PASS | 0.021 |
| `gui_initial_depth_contract` | PASS | 0.032 |
| `gui_pilot_toggle_contract` | PASS | 0.032 |
| `axis_rc_health_contract` | PASS | 0.025 |
| `initial_hold_pose` | PASS | 0.071 |
| `real_start_measurements` | PASS | 0.051 |
| `physics_contract_geometry` | PASS | 0.050 |
| `model_runtime_setup` | PASS | 0.480 |
| `physics_runtime_hydrostatic` | PASS | 0.057 |
| `thruster_param_loader` | PASS | 0.062 |
| `odometry_publish_builders` | PASS | 0.051 |
| `ros2_sitl_command_override` | PASS | 0.018 |
| `ping360_stl_io` | PASS | 0.022 |
| `source_contract_audit` | PASS | 0.426 |
| `dev_os_compat_headless` | PASS | 0.309 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
