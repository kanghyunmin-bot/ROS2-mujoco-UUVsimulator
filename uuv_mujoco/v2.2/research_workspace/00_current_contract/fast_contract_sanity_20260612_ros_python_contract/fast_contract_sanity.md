# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 23
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.191 |
| `rc_frame_contract` | PASS | 0.028 |
| `ros2_command_payload` | PASS | 0.026 |
| `bridge_live_imports` | PASS | 0.223 |
| `sitl_command_link_readiness` | PASS | 0.031 |
| `runtime_readiness_policy` | PASS | 0.023 |
| `gui_backend_selection` | PASS | 0.026 |
| `gui_readiness_contract` | PASS | 0.032 |
| `gui_ros_python_contract` | PASS | 0.844 |
| `gui_initial_depth_contract` | PASS | 0.026 |
| `gui_pilot_toggle_contract` | PASS | 0.029 |
| `axis_rc_health_contract` | PASS | 0.024 |
| `initial_hold_pose` | PASS | 0.071 |
| `real_start_measurements` | PASS | 0.053 |
| `physics_contract_geometry` | PASS | 0.048 |
| `model_runtime_setup` | PASS | 0.515 |
| `physics_runtime_hydrostatic` | PASS | 0.055 |
| `thruster_param_loader` | PASS | 0.053 |
| `odometry_publish_builders` | PASS | 0.050 |
| `ros2_sitl_command_override` | PASS | 0.018 |
| `ping360_stl_io` | PASS | 0.023 |
| `source_contract_audit` | PASS | 0.423 |
| `dev_os_compat_headless` | PASS | 0.321 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
