# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 20
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.341 |
| `rc_frame_contract` | PASS | 0.045 |
| `ros2_command_payload` | PASS | 0.040 |
| `bridge_live_imports` | PASS | 0.434 |
| `sitl_command_link_readiness` | PASS | 0.076 |
| `runtime_readiness_policy` | PASS | 0.059 |
| `gui_backend_selection` | PASS | 0.070 |
| `gui_readiness_contract` | PASS | 0.069 |
| `gui_initial_depth_contract` | PASS | 0.054 |
| `axis_rc_health_contract` | PASS | 0.066 |
| `initial_hold_pose` | PASS | 0.165 |
| `real_start_measurements` | PASS | 0.136 |
| `physics_contract_geometry` | PASS | 0.128 |
| `model_runtime_setup` | PASS | 1.039 |
| `physics_runtime_hydrostatic` | PASS | 0.100 |
| `thruster_param_loader` | PASS | 0.103 |
| `odometry_publish_builders` | PASS | 0.095 |
| `ros2_sitl_command_override` | PASS | 0.033 |
| `source_contract_audit` | PASS | 0.920 |
| `dev_os_compat_headless` | PASS | 0.426 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
