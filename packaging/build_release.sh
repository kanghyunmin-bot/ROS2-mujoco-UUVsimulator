#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERSION="$(tr -d '[:space:]' <"${ROOT_DIR}/VERSION")"
ARCH="${ARCH:-$(dpkg --print-architecture)}"
OUT_ROOT="${OUT_ROOT:-${ROOT_DIR}/documentary/release/builds}"
OUT_DIR="${OUT_ROOT}/${VERSION}"
PACKAGE_NAME="kmu-auv-simulator"
DEB_NAME="${PACKAGE_NAME}_${VERSION}_${ARCH}.deb"
DEB_PATH="${OUT_DIR}/${DEB_NAME}"
BUNDLE_ZIP="${OUT_DIR}/KMU_AUV_Simulator_Bundle_${VERSION}.zip"
INSTALLER_ZIP="${OUT_DIR}/KMU_AUV_Simulator_Installer_${VERSION}.zip"

for cmd in rsync zip unzip zipinfo dpkg-deb sha256sum awk sed; do
  command -v "$cmd" >/dev/null 2>&1 || {
    echo "[release] required command missing: $cmd" >&2
    exit 1
  }
done
[[ "$ARCH" == "amd64" ]] || {
  echo "[release] this release currently targets amd64, got: $ARCH" >&2
  exit 1
}
[[ -s "${ROOT_DIR}/sim/current/assets/yolo/best.pt" ]] || {
  echo "[release] YOLO model missing" >&2
  exit 1
}

TMP_DIR="$(mktemp -d /tmp/kmu-auv-release.XXXXXX)"
trap 'rm -rf "$TMP_DIR"' EXIT
mkdir -p "$OUT_DIR"

RSYNC_EXCLUDES=(
  --exclude='.git/'
  --exclude='__pycache__/'
  --exclude='*.pyc'
  --exclude='*.pyo'
  --exclude='.pytest_cache/'
  --exclude='build/'
  --exclude='install/'
  --exclude='log/'
  --exclude='logs/'
  --exclude='generated/'
  --exclude='private/'
  --exclude='real_robot_ros_bag/'
  --exclude='*.bag'
  --exclude='*.db3'
  --exclude='*.mcap'
  --exclude='core'
  --exclude='core.*'
  --exclude='*.bak'
  --exclude='*.orig'
  --exclude='*~'
)

copy_clean() {
  local source="$1"
  local destination="$2"
  mkdir -p "$destination"
  rsync -a "${RSYNC_EXCLUDES[@]}" "${source}/" "${destination}/"
}

copy_clean_follow_links() {
  local source="$1"
  local destination="$2"
  mkdir -p "$destination"
  rsync -aL "${RSYNC_EXCLUDES[@]}" "${source}/" "${destination}/"
}

zip_entry() {
  local parent="$1"
  local entry="$2"
  local output="$3"
  rm -f "$output"
  (cd "$parent" && zip -qr -9 "$output" "$entry")
}

echo "[release] staging MuJoCo runtime ${VERSION}"
RUNTIME_STAGE="${TMP_DIR}/runtime/sim"
mkdir -p "${RUNTIME_STAGE}/current"
rsync -a "${RSYNC_EXCLUDES[@]}" \
  --exclude='current/' --exclude='ardupilot/' \
  "${ROOT_DIR}/sim/" "${RUNTIME_STAGE}/"
copy_clean_follow_links "${ROOT_DIR}/sim/current" "${RUNTIME_STAGE}/current"
printf '%s\n' "$VERSION" >"${RUNTIME_STAGE}/current/.uuv_runtime_payload_version"
zip_entry "${TMP_DIR}/runtime" sim "${TMP_DIR}/uuv_mujoco.zip"

echo "[release] staging ROS source packages"
ROS_ZIP_DIR="${TMP_DIR}/ros-zips"
mkdir -p "$ROS_ZIP_DIR"
make_ros_zip() {
  local source="$1"
  local root_name="$2"
  local zip_name="$3"
  local stage_parent="${TMP_DIR}/ros-stage"
  local stage="${stage_parent}/${root_name}"
  rm -rf "$stage"
  copy_clean "$source" "$stage"
  [[ -s "${stage}/package.xml" ]] || {
    echo "[release] package.xml missing after staging: ${root_name}" >&2
    exit 1
  }
  if [[ "$zip_name" == "kmu26_auv_buoy_vision_control" ]]; then
    mkdir -p "${stage}/models"
    cp -a "${ROOT_DIR}/sim/current/assets/yolo/best.pt" "${stage}/models/best.pt"
  fi
  zip_entry "$stage_parent" "$root_name" "${ROS_ZIP_DIR}/${zip_name}.zip"
  rm -rf "$stage"
}

make_ros_zip "${ROOT_DIR}/rospkg/src/dvl_msgs" dvl_msgs dvl_msgs
make_ros_zip "${ROOT_DIR}/rospkg/src/auv_dvl_a50_msg" auv_dvl_a50_msg auv_dvl_a50_msg
make_ros_zip "${ROOT_DIR}/rospkg/src/ping360_sonar_msgs" ping360_sonar_msgs ping360_sonar_msgs
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_msg" kmu26_auv_msg kmu26_auv_msg
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv" kmu26_auv kmu26_auv
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_hydrophone/audio_common_msgs" audio_common_msgs audio_common_msgs
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_hydrophone/audio_common" audio_common audio_common
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_hydrophone/audio_capture" audio_capture audio_capture
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_hydrophone/hydrophone_ctrl" hydrophone_ctrl hydrophone_ctrl
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_buoy_vision_control" kmu26_auv_buoy_vision_control kmu26_auv_buoy_vision_control
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_vision_control/auv_lane_vision_control" auv_lane_vision_control auv_lane_vision_control
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_surface_buoy_mission" kmu26_auv_surface_buoy_mission kmu26_auv_surface_buoy_mission
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_web_gui" kmu26_auv_web_gui kmu26_auv_web_gui
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_pinger_homing" kmu26_pinger_homing kmu26_pinger_homing
make_ros_zip "${ROOT_DIR}/rospkg/src/robot_localization" robot_localization robot_localization

echo "[release] assembling self-contained installer bundle"
BUNDLE_ROOT="${TMP_DIR}/bundle"
mkdir -p "${BUNDLE_ROOT}/rospkg" "${BUNDLE_ROOT}/sim/current/assets/yolo" \
  "${BUNDLE_ROOT}/sim/ardupilot_patches"
cp -a "${ROOT_DIR}/VERSION" "${ROOT_DIR}/run_control_gui.sh" "$BUNDLE_ROOT/"
cp -a "${ROOT_DIR}/sim/current/tools/install/install_uuv_sim_current_ubuntu22.sh" "$BUNDLE_ROOT/"
cp -a "${ROOT_DIR}/sim/current/tools/install/preflight_uuv_sim_current.sh" "$BUNDLE_ROOT/"
cp -a "${ROOT_DIR}/sim/current/tools/install/setup_kmu26_hydrophone.sh" "$BUNDLE_ROOT/"
cp -a "${ROOT_DIR}/sim/current/tools/install/uuv_control_gui.py" "$BUNDLE_ROOT/"
cp -a "${ROOT_DIR}/packaging/ROS_PACKAGES.md" "${BUNDLE_ROOT}/rospkg/README.md"
cp -a "${ROOT_DIR}/documentary/release/README_FIRST.md" "$BUNDLE_ROOT/"
cp -a "${ROOT_DIR}/documentary/release/PORTABILITY_CHECKLIST.md" "$BUNDLE_ROOT/"
cp -a "${ROOT_DIR}/documentary/release/BUNDLE_CONTENTS.txt" "$BUNDLE_ROOT/"
cp -a "${ROOT_DIR}/documentary/release/RELEASE_MANIFEST.txt" "$BUNDLE_ROOT/"
cp -a "${TMP_DIR}/uuv_mujoco.zip" "$BUNDLE_ROOT/"
cp -a "${ROS_ZIP_DIR}/"*.zip "${BUNDLE_ROOT}/rospkg/"
cp -a "${ROOT_DIR}/sim/current/assets/yolo/best.pt" \
  "${BUNDLE_ROOT}/sim/current/assets/yolo/best.pt"
cp -a "${ROOT_DIR}/sim/ardupilot_patches/0001-current-althold-stabilize.patch" \
  "${BUNDLE_ROOT}/sim/ardupilot_patches/"
# Source files can come from a private developer checkout (for example a
# Git-LFS model with mode 0600). Everything below /opt is read by the regular
# desktop user after dpkg installs it as root, so normalize bundle access here.
chmod -R a+rX "$BUNDLE_ROOT"
chmod +x "${BUNDLE_ROOT}/"*.sh "${BUNDLE_ROOT}/"*.py
zip_entry "$TMP_DIR" bundle "$BUNDLE_ZIP"

echo "[release] building native Debian package"
DEB_ROOT="${TMP_DIR}/deb-root"
mkdir -p "${DEB_ROOT}/DEBIAN" "${DEB_ROOT}/opt/kmu-auv-simulator" \
  "${DEB_ROOT}/usr/bin" "${DEB_ROOT}/usr/share/applications" \
  "${DEB_ROOT}/usr/share/icons/hicolor/scalable/apps"
cp -a "$BUNDLE_ROOT" "${DEB_ROOT}/opt/kmu-auv-simulator/bundle"
cp -a "${ROOT_DIR}/VERSION" "${DEB_ROOT}/opt/kmu-auv-simulator/VERSION"
cp -a "${ROOT_DIR}/packaging/bin/kmu-auv-simulator" "${DEB_ROOT}/usr/bin/"
cp -a "${ROOT_DIR}/packaging/bin/kmu-auv-simulator-installer" "${DEB_ROOT}/usr/bin/"
cp -a "${ROOT_DIR}/packaging/bin/kmu-auv-simulator-uninstall" "${DEB_ROOT}/usr/bin/"
cp -a "${ROOT_DIR}/packaging/applications/"*.desktop "${DEB_ROOT}/usr/share/applications/"
cp -a "${ROOT_DIR}/packaging/icons/kmu-auv-simulator.svg" \
  "${DEB_ROOT}/usr/share/icons/hicolor/scalable/apps/"
cp -a "${ROOT_DIR}/packaging/debian/postinst" "${DEB_ROOT}/DEBIAN/postinst"
cp -a "${ROOT_DIR}/packaging/debian/prerm" "${DEB_ROOT}/DEBIAN/prerm"
cp -a "${ROOT_DIR}/packaging/debian/postrm" "${DEB_ROOT}/DEBIAN/postrm"
chmod 0755 "${DEB_ROOT}/DEBIAN/postinst" "${DEB_ROOT}/DEBIAN/prerm" \
  "${DEB_ROOT}/DEBIAN/postrm" \
  "${DEB_ROOT}/usr/bin/kmu-auv-simulator" \
  "${DEB_ROOT}/usr/bin/kmu-auv-simulator-installer" \
  "${DEB_ROOT}/usr/bin/kmu-auv-simulator-uninstall"
INSTALLED_KIB="$(du -sk "$DEB_ROOT" | awk '{print $1}')"
cat >"${DEB_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${VERSION}
Section: science
Priority: optional
Architecture: ${ARCH}
Installed-Size: ${INSTALLED_KIB}
Maintainer: KMU Underwater Robot Team <robot@example.com>
Depends: bash, python3, sudo, unzip, ca-certificates, curl, gnupg, lsb-release, zenity
Recommends: xterm
Description: KMU AUV MuJoCo, ArduSub SITL and ROS 2 simulator
 Installs a graphical first-run bootstrapper and a versioned competition
 simulator payload for Ubuntu 22.04 and ROS 2 Humble.
EOF
dpkg-deb --build --root-owner-group -Zxz "$DEB_ROOT" "$DEB_PATH" >/dev/null

echo "[release] building double-click delivery archive"
DELIVERY_ROOT="${TMP_DIR}/delivery/KMU_AUV_Simulator_Installer_${VERSION}"
mkdir -p "$DELIVERY_ROOT"
cp -a "$DEB_PATH" "$DELIVERY_ROOT/"
cat >"${DELIVERY_ROOT}/README_설치방법.txt" <<EOF
KMU AUV Simulator ${VERSION} / Ubuntu 22.04 amd64

1. ${DEB_NAME} 파일을 더블클릭합니다.
2. Ubuntu 앱 센터에서 설치를 누릅니다.
3. 앱 목록에서 'KMU AUV Simulator'를 실행합니다.
4. 첫 실행 설치 창에서 '설치 시작'을 누르고 관리자 암호를 입력합니다.

완전 삭제는 앱 목록의 'KMU AUV 시뮬레이터 완전 제거'를 실행합니다.
작업공간, ArduPilot, QGroundControl, ROS 빌드 결과, 전용 Python 환경과 로그가
삭제된 뒤 Debian 패키지도 제거됩니다.

인터넷 연결이 필요합니다. 프로그램 데이터는 사용자 홈의
~/.local/share/kmu-auv-simulator/current 아래에 설치됩니다.
EOF
(cd "$DELIVERY_ROOT" && sha256sum "$DEB_NAME" >SHA256SUMS.txt)
zip_entry "${TMP_DIR}/delivery" "KMU_AUV_Simulator_Installer_${VERSION}" "$INSTALLER_ZIP"

"${ROOT_DIR}/packaging/verify_release.sh" "$DEB_PATH"
(cd "$OUT_DIR" && sha256sum "$DEB_NAME" "$(basename "$BUNDLE_ZIP")" \
  "$(basename "$INSTALLER_ZIP")" >SHA256SUMS.txt)

echo "[release] complete"
echo "  deb:       ${DEB_PATH}"
echo "  installer: ${INSTALLER_ZIP}"
echo "  bundle:    ${BUNDLE_ZIP}"
echo "  checksums: ${OUT_DIR}/SHA256SUMS.txt"
