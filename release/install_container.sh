#!/usr/bin/env bash
set -euo pipefail
cd /workspace
export PIP_CONSTRAINT=/workspace/release/constraints.txt
./setup/03_setup_uuv_mujoco.sh --venv-root /workspace/.venv
.venv/bin/python -m pip install -r rospkg/src/auv_vla_data_collector/requirements-export.txt scipy pytest==7.4.4 lark==1.2.2
python3 setup/patch_ardusub_json_clock.py --ardupilot_dir /workspace/ardupilot_sub_stable --apply
(cd ardupilot_sub_stable && ./waf configure --board sitl && ./waf build --target bin/ardusub -j2)
cd /workspace/rospkg
KMU26_BUILD_JOBS=2 ./build_safe.sh --packages-up-to kmu26_auv_vla_policy auv_mavros_dvl_plugin dvl_msgs --cmake-args -DBUILD_TESTING=OFF
