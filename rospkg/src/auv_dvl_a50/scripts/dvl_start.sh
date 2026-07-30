#!/usr/bin/env bash
set -euo pipefail

IP="${1:-192.168.194.95}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1) DVL Dead Reckoning 리셋 1회
python3 "$SCRIPT_DIR/dvl_reset.py" --ip "$IP"

# 안정화 약간 대기(필요시 조정)
sleep 0.1

# 2) DVL → MAVROS 포즈 퍼블리셔 실행
# exec 로 교체해서 roslaunch가 이 프로세스를 모니터링하도록 함

# pose 데이터만
#exec rosrun waterlinked_a50_ros_driver dvl2mavros_pose.py _ip:="$IP"

# speed 데이터
#exec rosrun waterlinked_a50_ros_driver dvl2mavros_speed.py _ip:="$IP"

# posdelta 데이터
exec rosrun waterlinked_a50_ros_driver dvl2mavros_posdelta.py _ip:="$IP"


# odom 데이터
#exec rosrun waterlinked_a50_ros_driver dvl2mavros_odom.py _ip:="$IP"

