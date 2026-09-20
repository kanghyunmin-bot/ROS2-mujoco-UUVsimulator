#!/usr/bin/env bash
set -eo pipefail
# Public, pinned sources avoid missing binary packages in ROS apt rebuilds.
apt-get update
apt-get install -y --no-install-recommends libasio-dev libgeographic-dev geographiclib-tools libconsole-bridge-dev ros-humble-eigen-stl-containers ros-humble-geographic-msgs ros-humble-diagnostic-updater ros-humble-eigen3-cmake-module python3-lxml
mkdir -p /opt/uuv-mavros/src
cd /opt/uuv-mavros/src
fetch() {
  git init "$1"
  git -C "$1" fetch --depth 1 "$2" "$3"
  git -C "$1" checkout --detach FETCH_HEAD
}
fetch mavlink https://github.com/ros2-gbp/mavlink-gbp-release.git 45ec1a19169e3be7d3859ef5fb69e2d68feebbdc
fetch mavros https://github.com/mavlink/mavros.git c655e6343ec81687d51e7185bcba7a651e361fcd
source /opt/ros/humble/setup.bash
cd /opt/uuv-mavros
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --packages-up-to mavros mavros_extras --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
geographiclib-get-geoids egm96-5
test -f /usr/share/GeographicLib/geoids/egm96-5.pgm
rm -rf build log
