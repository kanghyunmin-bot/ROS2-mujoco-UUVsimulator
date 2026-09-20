#!/usr/bin/env bash
set -euo pipefail
cd /workspace
set +u
source /opt/ros/humble/setup.bash
source /opt/uuv-mavros/install/setup.bash
source rospkg/install/setup.bash
set -u
.venv/bin/python -c 'import mujoco,numpy,cv2,rclpy; print("MuJoCo",mujoco.__version__,"NumPy",numpy.__version__,"OpenCV",cv2.__version__)'
for pkg in hit25_auv_ros2 hit25_auv_ros2_msg mavros kmu26_auv_vla_data_collector kmu26_auv_vla_policy auv_mavros_dvl_plugin; do ros2 pkg prefix "$pkg"; done
test -x ardupilot_sub_stable/build/sitl/bin/ardusub
python3 setup/patch_ardusub_json_clock.py --ardupilot_dir ardupilot_sub_stable
MUJOCO_GL=egl .venv/bin/python release/render_smoke.py
python3 -m unittest discover -s tools/vla_gui -p 'test_*.py'
echo 'Simulation, ROS collector/policy, camera rendering and GUI checks passed.'
