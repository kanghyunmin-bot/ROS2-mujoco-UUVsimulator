#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERSION="$(tr -d '[:space:]' <"${ROOT_DIR}/VERSION")"
DEB_PATH="${1:-${ROOT_DIR}/documentary/release/builds/${VERSION}/kmu-auv-simulator_${VERSION}_amd64.deb}"

command -v docker >/dev/null 2>&1 || {
  echo "[clean-jammy] docker is required" >&2
  exit 1
}
[[ -s "$DEB_PATH" ]] || {
  echo "[clean-jammy] DEB missing: $DEB_PATH" >&2
  exit 1
}
DEB_PATH="$(cd "$(dirname "$DEB_PATH")" && pwd)/$(basename "$DEB_PATH")"

echo "[clean-jammy] testing ${DEB_PATH} as a regular user on ubuntu:22.04"
docker run --rm -i \
  -e "KMU_AUV_TEST_VERSION=${VERSION}" \
  -v "${DEB_PATH}:/input/kmu-auv.deb:ro" \
  ubuntu:22.04 bash -s <<'CONTAINER'
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y sudo ca-certificates
dpkg-deb -x /input/kmu-auv.deb /package
useradd -m -s /bin/bash kim
printf '%s\n' 'kim ALL=(ALL) NOPASSWD:ALL' >/etc/sudoers.d/kim
chmod 0440 /etc/sudoers.d/kim

INSTALLER=/package/opt/kmu-auv-simulator/bundle/install_uuv_sim_current_ubuntu22.sh
INSTALL_ROOT=/home/kim/.local/share/kmu-auv-simulator/current
COMMON_ENV="HOME=/home/kim UUV_DIST_BUILD_JOBS=2 UUV_SIM_DIST_VERSION=${KMU_AUV_TEST_VERSION}"

# Fresh install: install Jammy dependencies, clone and patch the pinned
# ArduPilot checkout, and produce the actual ArduSub SITL binary. ROS, QGC and
# the large ML venv are independently covered by the normal release verifier.
su -s /bin/bash kim -c \
  "env ${COMMON_ENV} ${INSTALLER} --install-root ${INSTALL_ROOT} --without-ros2 --skip-qgc --skip-python-env --noninteractive"

test -x "${INSTALL_ROOT}/sim/ardupilot/build/sitl/bin/ardusub"
su -s /bin/bash kim -c \
  'test -r /package/opt/kmu-auv-simulator/bundle/sim/current/assets/yolo/best.pt'

# Repair retry: reuse the patched checkout and prove that the legacy
# non-idempotent prerequisite helper is not re-entered.
su -s /bin/bash kim -c \
  "env ${COMMON_ENV} ${INSTALLER} --install-root ${INSTALL_ROOT} --without-ros2 --skip-apt --skip-qgc --skip-python-env --skip-ardupilot-build --noninteractive"

test "$(tr -d '[:space:]' <"${INSTALL_ROOT}/.uuv_sim_current_version")" = \
  "${KMU_AUV_TEST_VERSION}"
printf '%s\n' '[clean-jammy] PASS: fresh install, ArduSub build, model access and repair retry'
CONTAINER
