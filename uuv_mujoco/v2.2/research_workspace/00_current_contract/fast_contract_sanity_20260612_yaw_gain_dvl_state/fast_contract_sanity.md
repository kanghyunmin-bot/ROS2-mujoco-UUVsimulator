# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 19
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.176 |
| `rc_frame_contract` | PASS | 0.024 |
| `ros2_command_payload` | PASS | 0.019 |
| `bridge_live_imports` | PASS | 0.222 |
| `sitl_command_link_readiness` | PASS | 0.031 |
| `runtime_readiness_policy` | PASS | 0.034 |
| `gui_backend_selection` | PASS | 0.030 |
| `gui_readiness_contract` | PASS | 0.040 |
| `gui_initial_depth_contract` | PASS | 0.036 |
| `initial_hold_pose` | PASS | 0.064 |
| `real_start_measurements` | PASS | 0.055 |
| `physics_contract_geometry` | PASS | 0.050 |
| `model_runtime_setup` | PASS | 0.559 |
| `physics_runtime_hydrostatic` | PASS | 0.063 |
| `thruster_param_loader` | PASS | 0.058 |
| `odometry_publish_builders` | PASS | 0.056 |
| `ros2_sitl_command_override` | PASS | 0.028 |
| `source_contract_audit` | PASS | 0.452 |
| `dev_os_compat_headless` | PASS | 0.343 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
