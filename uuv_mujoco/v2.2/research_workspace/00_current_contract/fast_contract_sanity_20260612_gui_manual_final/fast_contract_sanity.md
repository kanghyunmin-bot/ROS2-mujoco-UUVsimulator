# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 18
- fail: 1

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.214 |
| `rc_frame_contract` | PASS | 0.031 |
| `ros2_command_payload` | FAIL | 0.088 |
| `bridge_live_imports` | PASS | 0.236 |
| `sitl_command_link_readiness` | PASS | 0.031 |
| `runtime_readiness_policy` | PASS | 0.024 |
| `gui_backend_selection` | PASS | 0.028 |
| `gui_readiness_contract` | PASS | 0.030 |
| `gui_initial_depth_contract` | PASS | 0.024 |
| `initial_hold_pose` | PASS | 0.049 |
| `real_start_measurements` | PASS | 0.052 |
| `physics_contract_geometry` | PASS | 0.047 |
| `model_runtime_setup` | PASS | 0.502 |
| `physics_runtime_hydrostatic` | PASS | 0.057 |
| `thruster_param_loader` | PASS | 0.055 |
| `odometry_publish_builders` | PASS | 0.053 |
| `ros2_sitl_command_override` | PASS | 0.019 |
| `source_contract_audit` | PASS | 0.412 |
| `dev_os_compat_headless` | PASS | 0.211 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
