# ROS2 State And Actuator Wrench Split

Date: 2026-06-07

Scope:

- `bridge/ros2_state_estimation.py`
- `tools/actuator_wrench_audit.py`

Change:

- Split ROS2 bridge state-estimation helpers into:
  - `bridge/ros2_state_vertical.py`
  - `bridge/ros2_state_kinematics.py`
  - `bridge/ros2_state_sensors.py`
  - `bridge/ros2_state_setpoint.py`
- Kept `bridge/ros2_state_estimation.py` as the compatibility facade consumed
  by `bridge/ros2_bridge.py`.
- Split actuator wrench audit logic into:
  - `tools/actuator_wrench_common.py`
  - `tools/actuator_wrench_model.py`
  - `tools/actuator_wrench_calc.py`
  - `tools/actuator_wrench_report.py`
- Kept `tools/actuator_wrench_audit.py` as the CLI entry point.

Contract:

- No controller-parity observation point changed.
- No plant input semantics changed.
- Bar30/static-pressure, IMU, DVL, and MAVROS setpoint helper method names used
  by `Ros2Bridge` are preserved.
- The actuator wrench audit still reports the same `axes` JSON shape and writes
  the same Markdown table format.

Verification:

```text
python3 -m py_compile \
  uuv_mujoco/current/bridge/ros2_state_estimation.py \
  uuv_mujoco/current/bridge/ros2_state_vertical.py \
  uuv_mujoco/current/bridge/ros2_state_kinematics.py \
  uuv_mujoco/current/bridge/ros2_state_sensors.py \
  uuv_mujoco/current/bridge/ros2_state_setpoint.py

source ./.uuv_mujoco_env.sh && PYTHONPATH=uuv_mujoco/current "$MJ311_PYTHON" <Ros2Bridge binding smoke>

python3 -m py_compile \
  uuv_mujoco/current/tools/actuator_wrench_audit.py \
  uuv_mujoco/current/tools/actuator_wrench_common.py \
  uuv_mujoco/current/tools/actuator_wrench_model.py \
  uuv_mujoco/current/tools/actuator_wrench_calc.py \
  uuv_mujoco/current/tools/actuator_wrench_report.py

source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" \
  uuv_mujoco/current/tools/actuator_wrench_audit.py \
  --out-json /private/tmp/actuator_wrench_after.json \
  --out-md /private/tmp/actuator_wrench_after.md

python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
git diff --check
```

Result:

- `bridge/ros2_state_estimation.py`: `325 LOC / 54` branches -> `38 LOC / 0`
  branches.
- `tools/actuator_wrench_audit.py`: `330 LOC / 35` branches -> `54 LOC / 3`
  branches.
- Both files dropped out of the top 35 hotspot inventory.
