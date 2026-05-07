#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/Users/kanghyunmin/Desktop/uuv_sim"
DOC_DIR="${ROOT_DIR}/document"
DOCSRC_DIR="${DOC_DIR}/docsource"
SIM_DIR="${ROOT_DIR}/uuv_mujoco/v2.2"
FIG_DIR="${DOCSRC_DIR}/figures_v30"
FLUID_MODEL="${1:-current}"
PROFILE="${2:-${FLUID_MODEL}}"
ENGINE_LABEL="${3:-${FLUID_MODEL}}"
OUT_DIR="${DOCSRC_DIR}/measurements/mavros_step_test_${ENGINE_LABEL}_$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${OUT_DIR}/bag"
LAUNCH_LOG="${OUT_DIR}/launcher.log"

mkdir -p "${OUT_DIR}"
mkdir -p "${FIG_DIR}"

set +u
source "${HOME}/miniconda3/etc/profile.d/conda.sh"
unset PYTHONPATH PYTHONHOME
conda activate ros2_h311
set -u

cleanup() {
  if [[ -n "${BAG_PID:-}" ]]; then
    kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
    wait "${BAG_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${LAUNCHER_PID:-}" ]]; then
    kill -INT "${LAUNCHER_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCHER_PID}" >/dev/null 2>&1 || true
  fi
  "${SIM_DIR}/reset_uuv_sim.sh" --with-qgc-stop >/dev/null 2>&1 || true
}
trap cleanup EXIT

cd "${SIM_DIR}"
"${SIM_DIR}/start_sitl_mujoco_mj311.sh" \
  --with-qgc-stop \
  --no-wait-ready \
  --ros2 \
  --sitl-no-rebuild \
  -- \
  --headless \
  --no-qgc-video \
  --tank-549x274x132 \
  --sitl-mavlink-endpoint "udpin:0.0.0.0:14551" \
  --fluid-model "${FLUID_MODEL}" \
  --profile "${PROFILE}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCHER_PID=$!

READY=0
for _ in $(seq 1 90); do
  if ! kill -0 "${LAUNCHER_PID}" >/dev/null 2>&1; then
    echo "[measurement] launcher exited early"
    tail -n 200 "${LAUNCH_LOG}" || true
    exit 1
  fi
  if [[ -f /tmp/ArduSub.log ]]; then
    if rg -Fq "JSON received:" /tmp/ArduSub.log \
      && rg -Fq "UDP connection 127.0.0.1:14551" /tmp/ArduSub.log; then
      READY=1
      break
    fi
  fi
  sleep 1
done

if [[ "${READY}" -ne 1 ]]; then
  echo "[measurement] launcher readiness timeout"
  tail -n 200 "${LAUNCH_LOG}" || true
  exit 1
fi

ros2 bag record \
  -o "${BAG_DIR}" \
  /measurement/phase \
  /mavros/state \
  /mavros/rc/override \
  /mavros/local_position/odom \
  /mavros/imu/data \
  /mavros/vfr_hud \
  /rovio/odometry \
  /dvl/odometry \
  /depth \
  /mujoco/ground_truth/pose \
  >"${OUT_DIR}/bag_record.log" 2>&1 &
BAG_PID=$!

sleep 3

python "${DOCSRC_DIR}/measure_mavros_rc_sequence.py" --output-dir "${OUT_DIR}"

kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
wait "${BAG_PID}" >/dev/null 2>&1 || true
unset BAG_PID

python "${DOCSRC_DIR}/analyze_mavros_bag.py" --bag-dir "${BAG_DIR}" --output-dir "${OUT_DIR}"

cp "${OUT_DIR}/plots/measured_step_responses.png" "${FIG_DIR}/actual_${ENGINE_LABEL}_mavros_step_responses.png"
cp "${OUT_DIR}/plots/measured_xy_track.png" "${FIG_DIR}/actual_${ENGINE_LABEL}_mavros_xy_track.png"
cp "${OUT_DIR}/plots/measured_mode_comparison.png" "${FIG_DIR}/actual_${ENGINE_LABEL}_mavros_mode_comparison.png"
cp "${OUT_DIR}/measurement_summary.json" "${DOCSRC_DIR}/measurement_summary_${ENGINE_LABEL}_latest_v30.json"

echo "[measurement] outputs saved in ${OUT_DIR}"
echo "[measurement] latest figures copied to ${FIG_DIR}/"
