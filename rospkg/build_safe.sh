#!/usr/bin/env bash
# Resource-bounded ROS 2 build for the robot laptop.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_JOBS="${KMU26_BUILD_JOBS:-1}"
MIN_AVAILABLE_MB="${KMU26_MIN_AVAILABLE_MB:-3500}"
ABORT_AVAILABLE_MB="${KMU26_ABORT_AVAILABLE_MB:-1200}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RESOURCE_LOG="${ROOT}/log/resource_build_${STAMP}.log"

available_mb() {
  awk '/MemAvailable:/ {printf "%d", $2 / 1024}' /proc/meminfo
}

if ! [[ "${BUILD_JOBS}" =~ ^[1-4]$ ]]; then
  echo "[build_safe] KMU26_BUILD_JOBS must be 1..4" >&2
  exit 2
fi

mkdir -p "${ROOT}/log"
AVAILABLE_MB="$(available_mb)"
if (( AVAILABLE_MB < MIN_AVAILABLE_MB )); then
  echo "[build_safe] refusing to build: ${AVAILABLE_MB} MiB available; ${MIN_AVAILABLE_MB} MiB required" >&2
  echo "[build_safe] stop duplicate detector/viewer jobs, but do not kill VS Code or MAVROS." >&2
  exit 75
fi

set +u
source /opt/ros/humble/setup.bash
set -u
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  set +u
  source "${ROOT}/install/setup.bash"
  set -u
fi

export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"
export MAKEFLAGS="-j${BUILD_JOBS}"
export NINJAFLAGS="-j${BUILD_JOBS}"
# Bypass the system ccache compiler wrappers. On this laptop their cache lock
# can stall a one-job ROS build indefinitely after an interrupted session.
export CC="${CC:-/usr/bin/gcc}"
export CXX="${CXX:-/usr/bin/g++}"
export CCACHE_DISABLE=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1

monitor_resources() {
  local build_pid="$1" current
  while kill -0 "${build_pid}" 2>/dev/null; do
    current="$(available_mb)"
    printf '%s available_mb=%s load=' "$(date --iso-8601=seconds)" "${current}"
    cut -d' ' -f1-3 /proc/loadavg
    if (( current < ABORT_AVAILABLE_MB )); then
      echo "[build_safe] aborting build before OOM: ${current} MiB available" >&2
      kill -INT -- "-${build_pid}" 2>/dev/null || true
      sleep 2
      kill -TERM -- "-${build_pid}" 2>/dev/null || true
      return 0
    fi
    sleep 5
  done
}
cleanup() {
  if [[ -n "${MONITOR_PID:-}" ]]; then
    kill "${MONITOR_PID}" 2>/dev/null || true
    wait "${MONITOR_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[build_safe] jobs=${BUILD_JOBS}, available=${AVAILABLE_MB} MiB, log=${RESOURCE_LOG}"
echo "[build_safe] physical package root=${ROOT}/src"

# --cmake-clean-cache is intentional after the one-time move into rospkg/src;
# object files remain reusable, while stale source paths in CMakeCache are removed.
setsid nice -n 10 ionice -c 2 -n 7 \
  colcon --log-base "${ROOT}/log" build \
    --base-paths "${ROOT}/src" \
    --build-base "${ROOT}/build" \
    --install-base "${ROOT}/install" \
    --executor sequential \
    --cmake-clean-cache \
    "$@" &
BUILD_PID=$!
monitor_resources "${BUILD_PID}" >>"${RESOURCE_LOG}" 2>&1 &
MONITOR_PID=$!
set +e
wait "${BUILD_PID}"
BUILD_RESULT=$?
set -e
cleanup
trap - EXIT INT TERM
exit "${BUILD_RESULT}"
