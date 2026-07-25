#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
VERSION="$(tr -d '[:space:]' <"${ROOT_DIR}/VERSION")"
PACKAGE_ROOT_NAME="uuv_sim_current_ubuntu22.04"
OUT_ROOT="${OUT_ROOT:-${ROOT_DIR}/documentary/release/builds}"
OUT_DIR="${OUT_ROOT}/${VERSION}"
LATEST_DIR="${OUT_ROOT}/latest"
DOWNLOAD_DIR="${DOWNLOAD_DIR:-${HOME}/Downloads}"
TEMPLATE_DEB="${TEMPLATE_DEB:-${LATEST_DIR}/uuv-sim-current_2026.07.01-dist2_amd64.deb}"
TEMPLATE_WRAPPER="${TEMPLATE_WRAPPER:-${HOME}/Downloads/UUV_Sim_Install_and_Run_2026.07.01-dist2.zip}"

DIRECT_ZIP="${OUT_DIR}/${PACKAGE_ROOT_NAME}.zip"
DEB_NAME="uuv-sim-current_${VERSION}_amd64.deb"
DEB_PATH="${OUT_DIR}/${DEB_NAME}"
WRAPPER_NAME="UUV_Sim_Install_and_Run_${VERSION}"
WRAPPER_ZIP="${OUT_DIR}/${WRAPPER_NAME}.zip"

for cmd in rsync zip unzip zipinfo dpkg-deb sha256sum sed awk; do
  command -v "$cmd" >/dev/null 2>&1 || {
    echo "[package-dist] command missing: ${cmd}" >&2
    exit 1
  }
done

if [[ ! -f "$TEMPLATE_DEB" && -d "$LATEST_DIR" ]]; then
  TEMPLATE_DEB="$(find "$LATEST_DIR" -maxdepth 1 -type f \
    -name 'uuv-sim-current_*_amd64.deb' -print | sort | tail -n 1)"
fi
[[ -f "$TEMPLATE_DEB" ]] || {
  echo "[package-dist] DEB template missing: ${TEMPLATE_DEB}" >&2
  exit 1
}
[[ -f "$TEMPLATE_WRAPPER" ]] || {
  echo "[package-dist] installer wrapper template missing: ${TEMPLATE_WRAPPER}" >&2
  exit 1
}
[[ -s "${ROOT_DIR}/sim/current/assets/yolo/best.pt" ]] || {
  echo "[package-dist] sim/current/assets/yolo/best.pt missing" >&2
  exit 1
}
[[ -s "${ROOT_DIR}/rospkg/src/kmu26_auv_buoy_vision_control/models/best.pt" ]] || {
  echo "[package-dist] ROS vision models/best.pt missing" >&2
  exit 1
}
cmp -s "${ROOT_DIR}/sim/current/assets/yolo/best.pt" \
  "${ROOT_DIR}/rospkg/src/kmu26_auv_buoy_vision_control/models/best.pt" || {
  echo "[package-dist] GUI and ROS best.pt files differ" >&2
  exit 1
}

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR" "$LATEST_DIR" "$DOWNLOAD_DIR"

RSYNC_EXCLUDES=(
  --exclude='.git/'
  --exclude='__pycache__/'
  --exclude='*.pyc'
  --exclude='*.pyo'
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
  --exclude='best (1).pt'
  --exclude='MUJOCO_LOG.TXT'
  --exclude='core'
  --exclude='core.*'
  --exclude='*.bak'
  --exclude='*.bak_*'
  --exclude='*.orig'
  --exclude='*~'
)

clean_copy() {
  local source="$1"
  local destination="$2"
  mkdir -p "$destination"
  rsync -a "${RSYNC_EXCLUDES[@]}" "${source}/" "${destination}/"
}

clean_copy_follow_links() {
  local source="$1"
  local destination="$2"
  mkdir -p "$destination"
  # Source compatibility links may point to another package in the developer
  # workspace. A release payload must remain self-contained after extraction.
  # The three Python pinger aliases are retained locally only as archive
  # breadcrumbs; the distribution executes the C++ controller exclusively.
  rsync -aL "${RSYNC_EXCLUDES[@]}" \
    --exclude='gui/single_hydrophone_homing_math.py' \
    --exclude='gui/single_hydrophone_homing_controller.py' \
    --exclude='tools/check_single_hydrophone_homing_math.py' \
    "${source}/" "${destination}/"
}

zip_tree() {
  local parent="$1"
  local entry="$2"
  local output="$3"
  rm -f "$output"
  (
    cd "$parent"
    zip -qr -9 "$output" "$entry"
  )
}

echo "[package-dist] staging runtime ${VERSION}"
RUNTIME_STAGE="${TMP_DIR}/runtime/sim"
mkdir -p "${RUNTIME_STAGE}/current"
rsync -a "${RSYNC_EXCLUDES[@]}" \
  --exclude='current/' --exclude='ardupilot/' \
  "${ROOT_DIR}/sim/" "${RUNTIME_STAGE}/"
clean_copy_follow_links "${ROOT_DIR}/sim/current" "${RUNTIME_STAGE}/current"
printf '%s\n' "$VERSION" >"${RUNTIME_STAGE}/.uuv_runtime_payload_version"
zip_tree "${TMP_DIR}/runtime" sim "${TMP_DIR}/uuv_mujoco.zip"

echo "[package-dist] staging ROS source packages"
ROS_ZIP_DIR="${TMP_DIR}/ros-zips"
mkdir -p "$ROS_ZIP_DIR"
make_ros_zip() {
  local source="$1"
  local root_name="$2"
  local zip_name="$3"
  local stage="${TMP_DIR}/ros-stage/${root_name}"
  rm -rf "$stage"
  clean_copy "$source" "$stage"
  [[ -s "${stage}/package.xml" ]] || {
    echo "[package-dist] package.xml missing after staging: ${root_name}" >&2
    exit 1
  }
  zip_tree "${TMP_DIR}/ros-stage" "$root_name" "${ROS_ZIP_DIR}/${zip_name}.zip"
  rm -rf "$stage"
}

make_ros_zip "${ROOT_DIR}/rospkg/src/dvl_msgs" dvl_msgs dvl_msgs
make_ros_zip "${ROOT_DIR}/rospkg/src/ping360_sonar_msgs" ping360_sonar_msgs ping360_sonar_msgs
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_msg" kmu26_auv_msg kmu26_auv_msg
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv" kmu26_auv kmu26_auv
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_hydrophone/audio_common_msgs" audio_common_msgs audio_common_msgs
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_hydrophone/audio_common" audio_common audio_common
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_hydrophone/audio_capture" audio_capture audio_capture
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_buoy_vision_control" \
  kmu26_auv_buoy_vision_control kmu26_auv_buoy_vision_control
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_mission_fsm" kmu26_mission_fsm kmu26_mission_fsm
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_auv_web_gui" \
  kmu26_auv_web_gui kmu26_auv_web_gui
make_ros_zip "${ROOT_DIR}/rospkg/src/kmu26_control_packages/kmu26_pinger_homing" \
  kmu26_pinger_homing kmu26_pinger_homing
make_ros_zip "${ROOT_DIR}/rospkg/src/robot_localization" \
  robot_localization robot_localization

echo "[package-dist] building direct extraction archive"
DIRECT_ROOT="${TMP_DIR}/direct/${PACKAGE_ROOT_NAME}"
mkdir -p "${DIRECT_ROOT}/sim/current/assets/yolo" "${DIRECT_ROOT}/rospkg"
cp -a "${ROOT_DIR}/VERSION" "${ROOT_DIR}/run_control_gui.sh" "${DIRECT_ROOT}/"
INSTALL_TOOL_DIR="${ROOT_DIR}/sim/current/tools/install"
for file in install_uuv_sim_current_ubuntu22.sh preflight_uuv_sim_current.sh \
  install_and_run_web.sh uuv_control_gui.py setup_kmu26_hydrophone.sh; do
  cp -a "${INSTALL_TOOL_DIR}/${file}" "${DIRECT_ROOT}/${file}"
done
for file in README_FIRST.md PORTABILITY_CHECKLIST.md BUNDLE_CONTENTS.txt RELEASE_MANIFEST.txt; do
  cp -a "${ROOT_DIR}/documentary/release/${file}" "${DIRECT_ROOT}/${file}"
done
cp -a "${ROOT_DIR}/sim/current/tools/verify_distribution.sh" \
  "${DIRECT_ROOT}/verify_current_dist.sh"
cp -a "${ROOT_DIR}/sim/current/tools/cleanup_workspace.sh" \
  "${DIRECT_ROOT}/cleanup_generated_artifacts.sh"
cp -a "${TMP_DIR}/uuv_mujoco.zip" "${DIRECT_ROOT}/uuv_mujoco.zip"
cp -a "${ROOT_DIR}/sim/current/assets/yolo/best.pt" "${DIRECT_ROOT}/sim/current/assets/yolo/best.pt"
cp -a "${ROS_ZIP_DIR}/"*.zip "${DIRECT_ROOT}/rospkg/"
cp -a "${ROOT_DIR}/rospkg/src/README.md" "${DIRECT_ROOT}/rospkg/README.md"
chmod +x "${DIRECT_ROOT}/"*.sh "${DIRECT_ROOT}/uuv_control_gui.py"
zip_tree "${TMP_DIR}/direct" "$PACKAGE_ROOT_NAME" "$DIRECT_ZIP"

"${ROOT_DIR}/sim/current/tools/verify_distribution.sh" "$DIRECT_ZIP"

echo "[package-dist] building Debian installer"
DEB_ROOT="${TMP_DIR}/deb-root"
dpkg-deb -R "$TEMPLATE_DEB" "$DEB_ROOT"
cp -a "$DIRECT_ZIP" "${DEB_ROOT}/opt/uuv-sim-current/uuv_sim_current_ubuntu22.04.zip"
cp -a "${ROOT_DIR}/VERSION" "${DEB_ROOT}/opt/uuv-sim-current/VERSION"
cp -a "${ROOT_DIR}/sim/current/tools/install/preflight_uuv_sim_current.sh" \
  "${DEB_ROOT}/opt/uuv-sim-current/preflight_uuv_sim_current.sh"
# DIST2's launchers skipped the ROS source/build refresh during a version
# change. Keep the same one-click surface, but make every launcher perform a
# complete in-place workspace upgrade before it starts the requested UI.
for launcher in uuv-sim-current-web uuv-sim-current-gui uuv-sim-current-headless; do
  launcher_path="${DEB_ROOT}/usr/bin/${launcher}"
  [[ -f "$launcher_path" ]] || {
    echo "[package-dist] DEB template launcher missing: ${launcher}" >&2
    exit 1
  }
  sed -i \
    -e 's#\\.uuv_mujoco_env\\.sh#sim/environment.sh#g' \
    -e 's/ --force-reextract//g' \
    -e 's/ --skip-rospkg-build//g' \
    -e 's/if \[\[ -f "$ENV_FILE" && -n "$payload_version" && "$target_version" != "$payload_version" \]\]; then/if [[ -f "$ENV_FILE" \&\& ( "${UUV_SIM_FORCE_REFRESH:-0}" == "1" || ( -n "$payload_version" \&\& "$target_version" != "$payload_version" ) ) ]]; then/' \
    -e 's/elif \[\[ -n "$payload_version" && "$target_version" != "$payload_version" \]\]; then/elif [[ "${UUV_SIM_FORCE_REFRESH:-0}" == "1" || ( -n "$payload_version" \&\& "$target_version" != "$payload_version" ) ]]; then/' \
    "$launcher_path"
done
sed -i -E "s/^Version:.*/Version: ${VERSION}/" "${DEB_ROOT}/DEBIAN/control"
installed_kib="$(du -sk "$DEB_ROOT" | awk '{print $1}')"
if grep -q '^Installed-Size:' "${DEB_ROOT}/DEBIAN/control"; then
  sed -i -E "s/^Installed-Size:.*/Installed-Size: ${installed_kib}/" "${DEB_ROOT}/DEBIAN/control"
else
  sed -i "/^Architecture:/a Installed-Size: ${installed_kib}" "${DEB_ROOT}/DEBIAN/control"
fi
dpkg-deb -b -Zxz "$DEB_ROOT" "$DEB_PATH" >/dev/null

echo "[package-dist] building one-click installer archive"
TEMPLATE_DIR="${TMP_DIR}/wrapper-template"
unzip -q "$TEMPLATE_WRAPPER" -d "$TEMPLATE_DIR"
OLD_WRAPPER_ROOT="$(find "$TEMPLATE_DIR" -mindepth 1 -maxdepth 1 -type d -print -quit)"
[[ -n "$OLD_WRAPPER_ROOT" ]] || {
  echo "[package-dist] invalid wrapper template" >&2
  exit 1
}
WRAPPER_ROOT="${TMP_DIR}/wrapper/${WRAPPER_NAME}"
mkdir -p "${WRAPPER_ROOT}/payload"
cp -a "$DEB_PATH" "${WRAPPER_ROOT}/payload/${DEB_NAME}"
if [[ -f "${OLD_WRAPPER_ROOT}/Install and Run UUV Sim" ]]; then
  cp -a "${OLD_WRAPPER_ROOT}/Install and Run UUV Sim" "${WRAPPER_ROOT}/Install and Run UUV Sim"
fi
[[ -f "${WRAPPER_ROOT}/Install and Run UUV Sim" ]] || {
  echo "[package-dist] double-click launcher missing from dist2 wrapper template" >&2
  exit 1
}
chmod +x "${WRAPPER_ROOT}/Install and Run UUV Sim"
sed \
  -e "s/2026\.07\.01-dist2/${VERSION}/g" \
  -e "s/uuv-sim-current_[^\"]*_amd64\.deb/${DEB_NAME}/g" \
  -e 's/env DEBIAN_FRONTEND=noninteractive apt install -y "$deb"/env DEBIAN_FRONTEND=noninteractive apt install -y --reinstall "$deb"/' \
  -e 's/if \[\[ "$installed_version" != "$target_version" \]\]; then/if true; then/' \
  -e '/message info "패키지 설치가 완료되었습니다/i\
  workspace_root="${UUV_SIM_INSTALL_ROOT:-${HOME}/uuv_sim_current}"\
  rm -f "${workspace_root}/.uuv_sim_current_version"' \
  -e "s/launch_cmd='uuv-sim-current-web/launch_cmd='UUV_SIM_FORCE_REFRESH=1 uuv-sim-current-web/" \
  "${OLD_WRAPPER_ROOT}/install_and_run_uuv_sim.sh" \
  >"${WRAPPER_ROOT}/install_and_run_uuv_sim.sh"
chmod +x "${WRAPPER_ROOT}/install_and_run_uuv_sim.sh"
cat >"${WRAPPER_ROOT}/README.txt" <<EOF
UUV Sim ${VERSION} (Ubuntu 22.04 x86_64)

1. 압축을 완전히 풉니다.
2. 폴더 안의 "Install and Run UUV Sim"을 더블클릭합니다.
3. Ubuntu가 물으면 "Allow Launching" / "신뢰하고 실행"을 선택합니다.
4. 요청되면 관리자 비밀번호를 입력합니다.
5. 설치 완료 창에서 "웹 GUI 실행"을 선택합니다.

터미널에서는 아래 명령으로 같은 설치를 실행할 수 있습니다.
   ./install_and_run_uuv_sim.sh
설치 후 웹 GUI는 uuv-sim-current-web 명령으로 다시 실행할 수 있습니다.
이전 버전이 설치되어 있으면 같은 ~/uuv_sim_current 위치에서 코드와 ROS 패키지를
자동 업그레이드하며, 수조 배치/소나/물리 설정과 로그는 보존합니다.
같은 dist4 버전의 수정 ZIP을 다시 실행해도 DEB와 작업공간을 강제로 새로 반영합니다.

포함 모델:
  sim/current/assets/yolo/best.pt
  ROS auv_buoy_vision_control/models/best.pt
  SHA-256: $(sha256sum "${ROOT_DIR}/sim/current/assets/yolo/best.pt" | awk '{print $1}')

ArduPilot, ROS 2 및 Python 의존성은 설치 과정에서 내려받으므로 인터넷 연결이 필요합니다.
EOF
cp -a "${ROOT_DIR}/RELEASE_MANIFEST.txt" "${WRAPPER_ROOT}/RELEASE_NOTES_${VERSION}.txt"
(
  cd "$WRAPPER_ROOT"
  sha256sum "payload/${DEB_NAME}" "Install and Run UUV Sim" \
    install_and_run_uuv_sim.sh README.txt \
    "RELEASE_NOTES_${VERSION}.txt" >SHA256SUMS.txt
)
zip_tree "${TMP_DIR}/wrapper" "$WRAPPER_NAME" "$WRAPPER_ZIP"

echo "[package-dist] validating generated installers"
unzip -tq "$WRAPPER_ZIP" >/dev/null
WRAPPER_VERIFY="${TMP_DIR}/wrapper-verify"
unzip -q "$WRAPPER_ZIP" -d "$WRAPPER_VERIFY"
WRAPPER_VERIFY_ROOT="${WRAPPER_VERIFY}/${WRAPPER_NAME}"
[[ -x "${WRAPPER_VERIFY_ROOT}/Install and Run UUV Sim" ]] || {
  echo "[package-dist] ZIP did not preserve the double-click launcher executable bit" >&2
  exit 1
}
[[ -x "${WRAPPER_VERIFY_ROOT}/install_and_run_uuv_sim.sh" ]] || {
  echo "[package-dist] ZIP did not preserve the installer script executable bit" >&2
  exit 1
}
bash -n "${WRAPPER_VERIFY_ROOT}/install_and_run_uuv_sim.sh"
grep -Fq 'apt install -y --reinstall "$deb"' \
  "${WRAPPER_VERIFY_ROOT}/install_and_run_uuv_sim.sh" || {
  echo "[package-dist] same-version DEB reinstall contract missing" >&2
  exit 1
}
grep -Fq 'UUV_SIM_FORCE_REFRESH=1 uuv-sim-current-web --open-browser' \
  "${WRAPPER_VERIFY_ROOT}/install_and_run_uuv_sim.sh" || {
  echo "[package-dist] one-click launcher does not force same-version workspace refresh" >&2
  exit 1
}
grep -Fq 'rm -f "${workspace_root}/.uuv_sim_current_version"' \
  "${WRAPPER_VERIFY_ROOT}/install_and_run_uuv_sim.sh" || {
  echo "[package-dist] DEB reinstall does not invalidate the workspace payload marker" >&2
  exit 1
}
printed_script="$(
  cd "$WRAPPER_VERIFY_ROOT"
  ./Install\ and\ Run\ UUV\ Sim --print-script
)"
[[ "$printed_script" == "${WRAPPER_VERIFY_ROOT}/install_and_run_uuv_sim.sh" ]] || {
  echo "[package-dist] double-click launcher does not resolve the bundled installer script" >&2
  exit 1
}
(
  cd "$WRAPPER_VERIFY_ROOT"
  sha256sum -c SHA256SUMS.txt >/dev/null
)
dpkg-deb -I "$DEB_PATH" >/dev/null
DEB_VERIFY="${TMP_DIR}/deb-verify"
dpkg-deb -x "$DEB_PATH" "$DEB_VERIFY"
cmp -s "$DIRECT_ZIP" "${DEB_VERIFY}/opt/uuv-sim-current/uuv_sim_current_ubuntu22.04.zip" || {
  echo "[package-dist] DEB payload differs from verified direct archive" >&2
  exit 1
}
for launcher in uuv-sim-current-web uuv-sim-current-gui uuv-sim-current-headless; do
  launcher_path="${DEB_VERIFY}/usr/bin/${launcher}"
  grep -Fq 'workspace refresh required' "$launcher_path" || {
    echo "[package-dist] ${launcher} lost automatic workspace upgrade detection" >&2
    exit 1
  }
  if grep -Eq -- '--force-reextract|--skip-rospkg-build' "$launcher_path"; then
    echo "[package-dist] ${launcher} still skips part of the in-place upgrade" >&2
    exit 1
  fi
  grep -Fq 'UUV_SIM_FORCE_REFRESH' "$launcher_path" || {
    echo "[package-dist] ${launcher} lost explicit same-version refresh support" >&2
    exit 1
  }
done
grep -Fq 'PORT="${UUV_WEB_PORT:-8878}"' "${DEB_VERIFY}/usr/bin/uuv-sim-current-web" || {
  echo "[package-dist] web launcher lost the 8878 default port contract" >&2
  exit 1
}
grep -Fq 'run_control_gui.sh" --web --host "$HOST" --port "$PORT"' \
  "${DEB_VERIFY}/usr/bin/uuv-sim-current-web" || {
  echo "[package-dist] web launcher does not forward the configured 8878 endpoint" >&2
  exit 1
}

(
  cd "$OUT_DIR"
  sha256sum "$(basename "$DIRECT_ZIP")" "$DEB_NAME" "$(basename "$WRAPPER_ZIP")" \
    >SHA256SUMS.txt
)

rm -f "${LATEST_DIR}/uuv_sim_current_ubuntu22.04.zip" \
  "${LATEST_DIR}"/uuv-sim-current_*_amd64.deb \
  "${LATEST_DIR}"/UUV_Sim_Install_and_Run_*.zip \
  "${LATEST_DIR}/SHA256SUMS.txt"
cp -a "$DIRECT_ZIP" "$DEB_PATH" "$WRAPPER_ZIP" "${OUT_DIR}/SHA256SUMS.txt" "$LATEST_DIR/"
cp -a "$DIRECT_ZIP" "$DEB_PATH" "$WRAPPER_ZIP" "${OUT_DIR}/SHA256SUMS.txt" "$DOWNLOAD_DIR/"

echo "[package-dist] complete"
echo "  direct:  ${DIRECT_ZIP}"
echo "  deb:     ${DEB_PATH}"
echo "  wrapper: ${WRAPPER_ZIP}"
echo "  copied:  ${DOWNLOAD_DIR}"
