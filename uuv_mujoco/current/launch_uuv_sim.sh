#!/bin/bash
# Launch MuJoCo UUV simulation
# Usage:
#   ./launch_uuv_sim.sh [--headless] [--sitl] [--images] [--no-ros2] [--ros2] [--ros2-real-pkg-compat] [--qgc-video] [--force-clean] [--scene <path>] [--profile <name>] [--tank-35x30x11] [--fluid-model <name>]
#   ./launch_uuv_sim.sh --sitl
#   ./launch_uuv_sim.sh --sitl --ros2 --ros2-real-pkg-compat
# Note:
#   ROS2 is enabled by default.
#   QGC direct video is disabled by default. Enable it with --qgc-video only when needed.
#   --images is kept as a legacy compatibility flag, but the lightweight
#   real-robot ROS2 bridge does not publish /stereo/* topics.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$SCRIPT_DIR"
HOST_OS="$(uname -s)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
CONTROLLER_PARITY_LOCK="${UUV_MUJOCO_CONTROLLER_PARITY_LOCK:-/tmp/uuv_mujoco_controller_parity.lock}"

controller_parity_lock_pid() {
    python3 - "$1" <<'PY'
import json
import sys
from pathlib import Path

try:
    data = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    print(int(data.get("pid", 0)))
except Exception:
    print("")
PY
}

guard_controller_parity_lock() {
    if [[ "${UUV_RESET_IGNORE_CONTROLLER_PARITY_LOCK:-0}" == "1" ]]; then
        return 0
    fi
    if [[ ! -f "$CONTROLLER_PARITY_LOCK" ]]; then
        return 0
    fi

    local lock_owner_pid
    lock_owner_pid="$(controller_parity_lock_pid "$CONTROLLER_PARITY_LOCK")"
    if [[ -n "$lock_owner_pid" ]] && kill -0 "$lock_owner_pid" 2>/dev/null; then
        echo "[launch] refusing to start/reset while controller-parity run owns MuJoCo/SITL lock"
        echo "[launch] lock: ${CONTROLLER_PARITY_LOCK}"
        echo "[launch] owner pid: ${lock_owner_pid}"
        echo "[launch] set UUV_RESET_IGNORE_CONTROLLER_PARITY_LOCK=1 only for intentional manual override"
        exit 75
    fi
}

guard_controller_parity_lock
UUV_RUNTIME_PROFILE="${UUV_RUNTIME_PROFILE:-balanced}"
case "${UUV_RUNTIME_PROFILE}" in
    low)
        PROFILE_SENSOR_HZ=30
        PROFILE_THRUSTER_HZ=60
        PROFILE_VIEWER_FPS=20
        ;;
    balanced|"")
        UUV_RUNTIME_PROFILE="balanced"
        PROFILE_SENSOR_HZ=100
        PROFILE_THRUSTER_HZ=140
        PROFILE_VIEWER_FPS=30
        ;;
    high)
        PROFILE_SENSOR_HZ=140
        PROFILE_THRUSTER_HZ=160
        PROFILE_VIEWER_FPS=30
        ;;
    *)
        echo "[warn] unknown UUV_RUNTIME_PROFILE=${UUV_RUNTIME_PROFILE}; using balanced"
        UUV_RUNTIME_PROFILE="balanced"
        PROFILE_SENSOR_HZ=100
        PROFILE_THRUSTER_HZ=140
        PROFILE_VIEWER_FPS=30
        ;;
esac
if [[ -n "${MJ311_ROOT:-}" ]]; then
    DEFAULT_MJ311_ROOT="${MJ311_ROOT}"
elif [[ -x "$HOME/.venvs/uuv_mujoco/bin/python" ]]; then
    DEFAULT_MJ311_ROOT="$HOME/.venvs/uuv_mujoco"
elif [[ -x "$HOME/.venvs/mujoco311/bin/python" ]]; then
    DEFAULT_MJ311_ROOT="$HOME/.venvs/mujoco311"
else
    DEFAULT_MJ311_ROOT="$HOME/.venvs/uuv_mujoco"
fi
MJ311_ROOT="${MJ311_ROOT:-$DEFAULT_MJ311_ROOT}"
MJ311_PYTHON="${MJ311_PYTHON:-}"
MJ311_MJPYTHON="${MJ311_MJPYTHON:-}"
ROS_ENV_SETUP="${ROS_ENV_SETUP:-}"
if [[ -z "${ROS_WORKSPACE_SETUP:-}" ]]; then
    if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
        ROS_WORKSPACE_SETUP="${PROJECT_ROOT}/install/setup.bash"
    else
        ROS_WORKSPACE_SETUP="${PROJECT_ROOT}/rospkg/install/setup.bash"
    fi
fi

resolve_python() {
    local candidate
    if [[ -n "$MJ311_PYTHON" ]]; then
        if [[ -x "$MJ311_PYTHON" ]]; then
            printf '%s\n' "$MJ311_PYTHON"
            return 0
        fi
        echo "[error] MJ311_PYTHON is not executable: $MJ311_PYTHON" >&2
        return 1
    fi
    if [[ -n "$MJ311_ROOT" && -x "$MJ311_ROOT/bin/python" ]]; then
        printf '%s\n' "$MJ311_ROOT/bin/python"
        return 0
    fi
    for candidate in python3 python; do
        if command -v "$candidate" >/dev/null 2>&1; then
            command -v "$candidate"
            return 0
        fi
    done
    echo "[error] no usable Python interpreter found." >&2
    echo "        Set MJ311_PYTHON or install python3." >&2
    return 1
}

resolve_mjpython() {
    local candidate python_bin
    if [[ -n "$MJ311_MJPYTHON" ]]; then
        if [[ -x "$MJ311_MJPYTHON" ]]; then
            printf '%s\n' "$MJ311_MJPYTHON"
            return 0
        fi
        echo "[error] MJ311_MJPYTHON is not executable: $MJ311_MJPYTHON" >&2
        return 1
    fi
    if [[ -n "$MJ311_ROOT" && -x "$MJ311_ROOT/bin/mjpython" ]]; then
        printf '%s\n' "$MJ311_ROOT/bin/mjpython"
        return 0
    fi
    python_bin="$(resolve_python)" || return 1
    candidate="$(cd "$(dirname "$python_bin")" && pwd)/mjpython"
    if [[ -x "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
    fi
    if command -v mjpython >/dev/null 2>&1; then
        command -v mjpython
        return 0
    fi
    if [[ "$(uname -s)" != "Darwin" ]]; then
        printf '%s\n' "$python_bin"
        return 0
    fi
    echo "[error] mjpython is required on macOS for the passive MuJoCo viewer." >&2
    echo "        Set MJ311_MJPYTHON or install mujoco into a Python env that provides mjpython." >&2
    return 1
}

# Parse arguments
HEADLESS=false
IMAGES=""
ROS2_REQUESTED=true
REAL_PKG_COMPAT=false
HEADLESS_ARG=""
SITL_ARG=""
SITL_PORT="9002"
SITL_SEND_PORT="9003"
EXTRA_ARGS=()
PROFILE=""
FORCE_CLEAN=false
QGC_VIDEO_AUTO=off
SCENE_PATH="scenes/tank_current_scene.xml"
FLUID_MODEL="current"
SITL_MAVLINK_TARGET_SYSID=1
SITL_MAVLINK_TARGET_COMPID=1
SITL_MAVLINK_SOURCE_SYSID="${SITL_MAVLINK_SOURCE_SYSID:-255}"
SITL_MAVLINK_SOURCE_COMPID="${SITL_MAVLINK_SOURCE_COMPID:-190}"

require_option_value() {
    local opt="$1"
    local argc="$2"
    if (( argc < 2 )); then
        echo "Missing value for ${opt}"
        exit 2
    fi
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --scene)
            require_option_value "$1" "$#"
            SCENE_PATH="$2"
            shift 2
            ;;
        --scene=*)
            SCENE_PATH="${1#*=}"
            shift
            ;;
        --tank-35x30x11|--tank-549x274x132)
            SCENE_PATH="scenes/tank_current_scene.xml"
            shift
            ;;
        --legacy-scene)
            echo "[launch] Unsupported legacy scene option: $1"
            echo "        Use --current-scene / --ellipsoid-scene so MuJoCo ellipsoid fluidcoef is active."
            exit 2
            ;;
        --current-scene)
            SCENE_PATH="scenes/tank_current_scene.xml"
            shift
            ;;
        --custom-scene)
            echo "[launch] Unsupported custom scene option: $1"
            echo "        Use --current-scene / --ellipsoid-scene so MuJoCo ellipsoid fluidcoef is active."
            exit 2
            ;;
        --ellipsoid-scene)
            SCENE_PATH="scenes/tank_current_scene.xml"
            shift
            ;;
        --fluid-model)
            require_option_value "$1" "$#"
            FLUID_MODEL="$2"
            shift 2
            ;;
        --fluid-model=*)
            FLUID_MODEL="${1#*=}"
            shift
            ;;
        --headless)
            HEADLESS=true
            shift
            ;;
        --images)
            IMAGES="--ros2-images"
            ROS2_REQUESTED=true
            shift
            ;;
        --no-ros2)
            ROS2_REQUESTED=false
            REAL_PKG_COMPAT=false
            shift
            ;;
        --sitl)
            SITL_ARG="--sitl"
            shift
            ;;
        --ros2)
            ROS2_REQUESTED=true
            REAL_PKG_COMPAT=false
            EXTRA_ARGS+=("--ros2")
            shift
            ;;
        --ros2-real-pkg-compat)
            ROS2_REQUESTED=true
            REAL_PKG_COMPAT=true
            EXTRA_ARGS+=("--ros2-real-pkg-compat")
            shift
            ;;
        --sitl-port)
            require_option_value "$1" "$#"
            SITL_PORT="$2"
            shift 2
            ;;
        --sitl-send-port)
            require_option_value "$1" "$#"
            SITL_SEND_PORT="$2"
            shift 2
            ;;
        --sitl-servo-source|--sitl-servo-source=*|--allow-mavros-rc-in-sitl|--hover-stable)
            echo "[launch] Unsupported legacy option: $1"
            echo "        Current standard path uses MAVLink SITL control and no internal hover helper."
            exit 2
            ;;
        --force-clean)
            FORCE_CLEAN=true
            shift
            ;;
        --qgc-video)
            QGC_VIDEO_AUTO=on
            EXTRA_ARGS+=("--qgc-video")
            shift
            ;;
        --no-qgc-video)
            QGC_VIDEO_AUTO=off
            shift
            ;;
        --profile)
            require_option_value "$1" "$#"
            PROFILE="$2"
            EXTRA_ARGS+=("--profile" "$2")
            shift 2
            ;;
        --thruster-voltage)
            require_option_value "$1" "$#"
            EXTRA_ARGS+=("--thruster-voltage" "$2")
            shift 2
            ;;
        --calib-left)
            require_option_value "$1" "$#"
            EXTRA_ARGS+=("--ros2-camera-calib-left" "$2")
            shift 2
            ;;
        --calib-right)
            require_option_value "$1" "$#"
            EXTRA_ARGS+=("--ros2-camera-calib-right" "$2")
            shift 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ "$REAL_PKG_COMPAT" == "true" ]]; then
    case "$(printf '%s' "${UUV_REAL_PKG_CAMERA_ENABLE:-1}" | tr '[:upper:]' '[:lower:]')" in
        1|true|yes|on|enable|enabled)
            IMAGES="--ros2-images"
            ;;
    esac
fi

resolve_scene_path() {
    local requested="$1"
    if [[ -z "$requested" ]]; then
        echo "[error] empty scene path requested." >&2
        return 1
    fi
    case "$requested" in
        tank_legacy_scene.xml)
            echo "[error] legacy scene removed from runtime path; use scenes/tank_current_scene.xml" >&2
            return 1
            ;;
        tank_current_scene.xml)
            requested="scenes/tank_current_scene.xml"
            ;;
        tank_custom_scene.xml)
            echo "[error] custom scene removed from runtime path; use scenes/tank_current_scene.xml" >&2
            return 1
            ;;
        tank_ellipsoid_scene.xml)
            requested="scenes/tank_current_scene.xml"
            ;;
    esac
    if [[ -f "$requested" ]]; then
        printf '%s\n' "$requested"
        return 0
    fi
    if [[ -f "$SCRIPT_DIR/$requested" ]]; then
        printf '%s\n' "$requested"
        return 0
    fi
    echo "[error] scene file not found: $requested" >&2
    echo "        Looked for: $requested and $SCRIPT_DIR/$requested" >&2
    return 1
}

case "$FLUID_MODEL" in
    ellipsoid|builtin-ellipsoid)
        FLUID_MODEL="current"
        ;;
    distributed|patch)
        FLUID_MODEL="distributed"
        ;;
    current|legacy|custom)
        ;;
    *)
        echo "[error] unknown --fluid-model: ${FLUID_MODEL}" >&2
        echo "        expected one of: current, ellipsoid, builtin-ellipsoid, distributed, legacy, custom" >&2
        exit 2
        ;;
esac

if [[ "$(basename "$SCENE_PATH")" == "tank_legacy_scene.xml" ]]; then
    echo "[error] legacy scene removed from runtime path; use scenes/tank_current_scene.xml" >&2
    exit 2
fi

SCENE_PATH="$(resolve_scene_path "$SCENE_PATH")"
SCENE_LABEL="$(basename "$SCENE_PATH")"

if [ "$HEADLESS" = true ]; then
    PY_LAUNCHER="$(resolve_python)"
else
    PY_LAUNCHER="$(resolve_mjpython)"
fi

run_dev_os_preflight() {
    local skip_raw skip_lc check_script check_python
    skip_raw="${UUV_SKIP_DEV_OS_COMPAT_CHECK:-0}"
    skip_lc="$(printf '%s' "$skip_raw" | tr '[:upper:]' '[:lower:]')"
    case "$skip_lc" in
        1|true|yes|on|enable|enabled)
            echo "[launch] dev OS compatibility preflight skipped by UUV_SKIP_DEV_OS_COMPAT_CHECK=${skip_raw}"
            return 0
            ;;
    esac

    check_script="${SCRIPT_DIR}/tools/check_dev_os_compat.py"
    if [[ ! -f "$check_script" ]]; then
        echo "[launch] dev OS compatibility preflight unavailable: ${check_script}"
        return 0
    fi

    check_python="$(resolve_python)" || return 1
    local check_args=("--python" "$check_python")
    if [ "$HEADLESS" = true ]; then
        check_args+=("--headless")
    else
        check_args+=("--require-viewer" "--mjpython" "$PY_LAUNCHER")
    fi

    local strict_lc
    strict_lc="$(printf '%s' "${UUV_DEV_OS_COMPAT_STRICT:-0}" | tr '[:upper:]' '[:lower:]')"
    case "$strict_lc" in
        1|true|yes|on|enable|enabled)
            check_args+=("--strict")
            ;;
    esac

    echo "[launch] dev OS compatibility preflight: ${check_script}"
    if ! "$check_python" "$check_script" "${check_args[@]}"; then
        echo "[error] dev OS compatibility preflight failed." >&2
        echo "        Fix the reported Python/MuJoCo/SITL host contract or set UUV_SKIP_DEV_OS_COMPAT_CHECK=1 for intentional debugging." >&2
        exit 78
    fi
}

run_dev_os_preflight

extra_arg_present() {
    local needle="$1"
    local token
    for token in "${EXTRA_ARGS[@]}"; do
        if [[ "$token" == "$needle" || "$token" == "$needle="* ]]; then
            return 0
        fi
    done
    return 1
}

extra_arg_value() {
    local needle="$1"
    local idx next_idx
    for idx in "${!EXTRA_ARGS[@]}"; do
        if [[ "${EXTRA_ARGS[$idx]}" == "$needle="* ]]; then
            printf '%s\n' "${EXTRA_ARGS[$idx]#*=}"
            return 0
        fi
        if [[ "${EXTRA_ARGS[$idx]}" == "$needle" ]]; then
            next_idx=$((idx + 1))
            if (( next_idx < ${#EXTRA_ARGS[@]} )); then
                printf '%s\n' "${EXTRA_ARGS[$next_idx]}"
                return 0
            fi
        fi
    done
    return 1
}

append_extra_arg_if_missing() {
    local needle="$1"
    shift
    if ! extra_arg_present "$needle"; then
        EXTRA_ARGS+=("$@")
        return 0
    fi
    return 1
}

replace_extra_arg_value() {
    local needle="$1"
    local value="$2"
    local idx next_idx
    for idx in "${!EXTRA_ARGS[@]}"; do
        if [[ "${EXTRA_ARGS[$idx]}" == "$needle="* ]]; then
            EXTRA_ARGS[$idx]="${needle}=${value}"
            return 0
        fi
        if [[ "${EXTRA_ARGS[$idx]}" == "$needle" ]]; then
            next_idx=$((idx + 1))
            if (( next_idx < ${#EXTRA_ARGS[@]} )); then
                EXTRA_ARGS[$next_idx]="$value"
                return 0
            fi
        fi
    done
    EXTRA_ARGS+=("$needle" "$value")
}

env_flag_enabled() {
    local raw="${1:-0}"
    raw="$(printf '%s' "$raw" | tr '[:upper:]' '[:lower:]')"
    case "$raw" in
        1|true|yes|on|enable|enabled)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

real_start_state_default_csv() {
    printf '%s\n' "${SCRIPT_DIR}/debug/controller_parity_412/real_20260401_feedback/real_controller_feedback_20hz.csv"
}

apply_real_start_state_args() {
    local enabled_raw="${UUV_REAL_START_STATE:-0}"
    if ! env_flag_enabled "$enabled_raw"; then
        return 0
    fi

    local csv_path="${UUV_REAL_START_STATE_CSV:-$(real_start_state_default_csv)}"
    local start_s="${UUV_REAL_START_STATE_T_S:-69.35}"
    local helper="${SCRIPT_DIR}/tools/real_start_state.py"
    if [[ ! -f "$csv_path" ]]; then
        echo "[launch] UUV_REAL_START_STATE requested but CSV not found: ${csv_path}" >&2
        exit 2
    fi
    if [[ ! -f "$helper" ]]; then
        echo "[launch] real-state helper missing: ${helper}" >&2
        exit 2
    fi

    local state_shell
    if ! state_shell="$("$PY_LAUNCHER" "$helper" --csv "$csv_path" --start "$start_s" --format shell)"; then
        echo "[launch] failed to derive real initial state from ${csv_path}" >&2
        exit 2
    fi
    eval "$state_shell"
    export UUV_REAL_START_STATE_APPLIED=1
    export UUV_REAL_START_STATE_CSV="$csv_path"
    export UUV_REAL_START_STATE_T_S="$start_s"
    export UUV_REAL_START_STATE_AUTO_RELEASE="${UUV_REAL_START_STATE_AUTO_RELEASE:-1}"
    export UUV_REAL_START_SOURCE_T_S
    export UUV_REAL_START_DEPTH_M
    export UUV_REAL_START_BASE_DEPTH_M
    export UUV_REAL_START_BASE_X_M
    export UUV_REAL_START_BASE_Y_M
    export UUV_REAL_START_ROLL_RAD
    export UUV_REAL_START_PITCH_RAD
    export UUV_REAL_START_YAW_RAD
    export UUV_REAL_START_BODY_VX_MPS
    export UUV_REAL_START_BODY_VY_MPS
    export UUV_REAL_START_BODY_VZ_MPS
    export UUV_REAL_START_BODY_WX_RADPS
    export UUV_REAL_START_BODY_WY_RADPS
    export UUV_REAL_START_BODY_WZ_RADPS
    export UUV_REAL_START_MODE
    export UUV_REAL_START_ARMED
    export UUV_REAL_START_DEPTH_SOURCE
    export UUV_REAL_START_BASE_DEPTH_SOURCE
    export UUV_REAL_START_BASE_XY_SOURCE
    export UUV_REAL_START_ATTITUDE_SOURCE
    export UUV_REAL_START_VELOCITY_SOURCE
    export UUV_REAL_START_ANGULAR_VELOCITY_SOURCE
    export UUV_REAL_START_STATIC_PRESSURE_PA
    export UUV_REAL_START_BAR30_SURFACE_PRESSURE_PA
    export UUV_REAL_START_BARO_REAL_GND_PRESSURE_PA
    export UUV_REAL_START_BARO_REAL_GND_SOURCE
    export UUV_REAL_START_BARO_SITL_GND_PRESSURE_PA
    export UUV_REAL_START_BARO_JSON_DEPTH_M
    export UUV_REAL_START_BARO_FRONTEND_DEPTH_M

    if [[ "${UUV_BAR30_SURFACE_PRESSURE_USER_SET:-0}" != "1" && -n "${UUV_REAL_START_BAR30_SURFACE_PRESSURE_PA:-}" ]]; then
        export ROS2_UUV_BAR30_SURFACE_PRESSURE_PA="$UUV_REAL_START_BAR30_SURFACE_PRESSURE_PA"
    fi
    export ROS2_UUV_BARO_REAL_GND_PRESSURE_PA="${ROS2_UUV_BARO_REAL_GND_PRESSURE_PA:-$UUV_REAL_START_BARO_REAL_GND_PRESSURE_PA}"
    export ROS2_UUV_BARO_SITL_GND_PRESSURE_PA="${ROS2_UUV_BARO_SITL_GND_PRESSURE_PA:-$UUV_REAL_START_BARO_SITL_GND_PRESSURE_PA}"
    export ROS2_UUV_SITL_BARO_DEPTH_CONTRACT="${ROS2_UUV_SITL_BARO_DEPTH_CONTRACT:-frontend_match}"

    if ! extra_arg_present "--initial-depth-m" && ! extra_arg_present "--initial-bar30-depth-m"; then
        real_start_geometry_depth_source="${UUV_REAL_START_GEOMETRY_DEPTH_SOURCE:-base}"
        real_start_geometry_depth_source="$(printf '%s' "$real_start_geometry_depth_source" | tr '[:upper:]' '[:lower:]')"
        case "$real_start_geometry_depth_source" in
            base|base_link|local_pose)
                EXTRA_ARGS+=("--initial-depth-m" "$UUV_REAL_START_BASE_DEPTH_M")
                ;;
            bar30|depth|depth_topic|pressure)
                EXTRA_ARGS+=("--initial-bar30-depth-m" "$UUV_REAL_START_DEPTH_M")
                ;;
            *)
                echo "[launch] invalid UUV_REAL_START_GEOMETRY_DEPTH_SOURCE=${UUV_REAL_START_GEOMETRY_DEPTH_SOURCE}; expected bar30 or base" >&2
                exit 2
                ;;
        esac
    fi
    if ! extra_arg_present "--initial-rpy-rad"; then
        EXTRA_ARGS+=(
            "--initial-rpy-rad"
            "$UUV_REAL_START_ROLL_RAD"
            "$UUV_REAL_START_PITCH_RAD"
            "$UUV_REAL_START_YAW_RAD"
        )
    fi
    if ! extra_arg_present "--initial-position-xy"; then
        EXTRA_ARGS+=(
            "--initial-position-xy"
            "$UUV_REAL_START_BASE_X_M"
            "$UUV_REAL_START_BASE_Y_M"
        )
    fi
    if ! extra_arg_present "--release-linear-velocity-body"; then
        EXTRA_ARGS+=(
            "--release-linear-velocity-body"
            "$UUV_REAL_START_BODY_VX_MPS"
            "$UUV_REAL_START_BODY_VY_MPS"
            "$UUV_REAL_START_BODY_VZ_MPS"
        )
    fi
    if ! extra_arg_present "--release-angular-velocity-body"; then
        EXTRA_ARGS+=(
            "--release-angular-velocity-body"
            "$UUV_REAL_START_BODY_WX_RADPS"
            "$UUV_REAL_START_BODY_WY_RADPS"
            "$UUV_REAL_START_BODY_WZ_RADPS"
        )
    fi

    local hold_raw="${UUV_REAL_START_STATE_HOLD_UNTIL_RELEASE:-1}"
    if env_flag_enabled "$hold_raw" && ! extra_arg_present "--hold-initial-depth-until-release"; then
        EXTRA_ARGS+=("--hold-initial-depth-until-release")
    fi

    echo "[launch] real start state: csv=${csv_path}, t=${UUV_REAL_START_SOURCE_T_S}s, base_xy=${UUV_REAL_START_BASE_X_M},${UUV_REAL_START_BASE_Y_M} (${UUV_REAL_START_BASE_XY_SOURCE}), base_depth=${UUV_REAL_START_BASE_DEPTH_M}m (${UUV_REAL_START_BASE_DEPTH_SOURCE}), depth_topic=${UUV_REAL_START_DEPTH_M}m (${UUV_REAL_START_DEPTH_SOURCE}), rpy=${UUV_REAL_START_ROLL_RAD},${UUV_REAL_START_PITCH_RAD},${UUV_REAL_START_YAW_RAD} (${UUV_REAL_START_ATTITUDE_SOURCE}), body_v=${UUV_REAL_START_BODY_VX_MPS},${UUV_REAL_START_BODY_VY_MPS},${UUV_REAL_START_BODY_VZ_MPS} (${UUV_REAL_START_VELOCITY_SOURCE}), body_w=${UUV_REAL_START_BODY_WX_RADPS},${UUV_REAL_START_BODY_WY_RADPS},${UUV_REAL_START_BODY_WZ_RADPS} (${UUV_REAL_START_ANGULAR_VELOCITY_SOURCE})"
    echo "[launch] real Bar30 contract: static_pressure=${UUV_REAL_START_STATIC_PRESSURE_PA}Pa, initial_surface=${ROS2_UUV_BAR30_SURFACE_PRESSURE_PA}Pa, real_gnd=${ROS2_UUV_BARO_REAL_GND_PRESSURE_PA}Pa (${UUV_REAL_START_BARO_REAL_GND_SOURCE}), sitl_gnd=${ROS2_UUV_BARO_SITL_GND_PRESSURE_PA}Pa, frontend_depth_at_start=${UUV_REAL_START_BARO_FRONTEND_DEPTH_M}m, json_depth_at_start=${UUV_REAL_START_BARO_JSON_DEPTH_M}m"
}

source_setup_bash_safely() {
    local setup_file="$1"
    local restore_nounset=0
    if [[ $- == *u* ]]; then
        restore_nounset=1
        set +u
    fi
    # shellcheck disable=SC1090
    source "$setup_file"
    if [[ "$restore_nounset" -eq 1 ]]; then
        set -u
    fi
}

resolve_ros_setup_for_bash() {
    local candidate base_dir

    for candidate in \
        "$ROS_ENV_SETUP" \
        "$HOME/miniconda3/envs/ros2_h311/setup.bash" \
        "$HOME/miniconda3/envs/ros2_h311/setup.sh" \
        "$HOME/miniconda3/envs/ros2/setup.bash" \
        "$HOME/miniconda3/envs/ros2/setup.sh" \
        "/opt/ros/${ROS_DISTRO}/setup.bash" \
        "/opt/ros/${ROS_DISTRO}/setup.sh" \
        "${CONDA_PREFIX:-}/setup.bash" \
        "${CONDA_PREFIX:-}/setup.sh" \
        "/opt/homebrew/opt/ros/${ROS_DISTRO}/setup.bash" \
        "/usr/local/opt/ros/${ROS_DISTRO}/setup.bash"
    do
        [[ -n "$candidate" ]] || continue
        if [[ -f "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
        if [[ "$candidate" == *.zsh ]]; then
            base_dir="$(cd "$(dirname "$candidate")" && pwd)"
            if [[ -f "$base_dir/setup.bash" ]]; then
                printf '%s\n' "$base_dir/setup.bash"
                return 0
            fi
            if [[ -f "$base_dir/setup.sh" ]]; then
                printf '%s\n' "$base_dir/setup.sh"
                return 0
            fi
        fi
    done

    return 1
}

env_flag_disabled() {
    local value
    value="$(printf '%s' "${1:-}" | tr '[:upper:]' '[:lower:]')"
    case "$value" in
        0|false|no|off|disable|disabled)
            return 0
            ;;
    esac
    return 1
}

mujoco_system_glfw_library() {
    local candidate
    for candidate in \
        "/lib/x86_64-linux-gnu/libglfw.so.3" \
        "/usr/lib/x86_64-linux-gnu/libglfw.so.3" \
        "/lib/aarch64-linux-gnu/libglfw.so.3" \
        "/usr/lib/aarch64-linux-gnu/libglfw.so.3"
    do
        if [[ -f "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
    if command -v ldconfig >/dev/null 2>&1; then
        ldconfig -p 2>/dev/null | awk '/libglfw\.so\.3[[:space:]]/ {print $NF; exit}'
    fi
}

configure_mujoco_viewer_window_backend() {
    if [[ "$HOST_OS" != "Linux" || "$HEADLESS" == true ]]; then
        return
    fi
    if [[ -z "${DISPLAY:-}" ]]; then
        return
    fi
    local wayland_session=0
    case "$(printf '%s' "${XDG_SESSION_TYPE:-}" | tr '[:upper:]' '[:lower:]')" in
        wayland)
            wayland_session=1
            ;;
    esac
    if [[ -n "${WAYLAND_DISPLAY:-}" || "${GLFW_PLATFORM:-}" == "x11" ]]; then
        wayland_session=1
    fi
    if [[ "$wayland_session" -ne 1 ]]; then
        return
    fi
    if env_flag_disabled "${UUV_GUI_MUJOCO_XWAYLAND:-1}"; then
        echo "[launch] Wayland session: native MuJoCo GLFW backend requested (UUV_GUI_MUJOCO_XWAYLAND=0)."
        return
    fi

    export GLFW_PLATFORM="${GLFW_PLATFORM:-x11}"
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    export GDK_BACKEND="${GDK_BACKEND:-x11}"
    export SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-x11}"
    if [[ -z "${PYGLFW_LIBRARY:-}" ]]; then
        local glfw_library
        glfw_library="$(mujoco_system_glfw_library || true)"
        if [[ -n "$glfw_library" ]]; then
            export PYGLFW_LIBRARY="$glfw_library"
        fi
    fi
    unset WAYLAND_DISPLAY
    echo "[launch] Wayland session: forcing MuJoCo GLFW viewer through XWayland (DISPLAY=${DISPLAY}, PYGLFW_LIBRARY=${PYGLFW_LIBRARY:-auto})."
}

collect_existing_mujoco_pids() {
    pgrep -f "run_uuv_mujoco.py" || true
    pgrep -f "run_urdf_full.py" || true
}

kill_pid_list() {
    local pids="$1"
    [[ -z "$pids" ]] && return 0
    # shellcheck disable=SC2086
    kill $pids 2>/dev/null || true
    sleep 0.6
    local alive=""
    # shellcheck disable=SC2086
    for pid in $pids; do
        if kill -0 "$pid" 2>/dev/null; then
            alive+=" $pid"
        fi
    done
    if [[ -n "$alive" ]]; then
        # shellcheck disable=SC2086
        kill -9 $alive 2>/dev/null || true
    fi
}

EXISTING_MJ_PIDS="$(collect_existing_mujoco_pids | xargs)"
if [[ -n "$EXISTING_MJ_PIDS" ]]; then
    if [[ "$FORCE_CLEAN" == true ]]; then
        echo "[launch] --force-clean: stopping existing MuJoCo runtimes:$EXISTING_MJ_PIDS"
        kill_pid_list "$EXISTING_MJ_PIDS"
    else
        echo "[error] existing MuJoCo runtime detected:$EXISTING_MJ_PIDS"
        echo "        Stop old process first or rerun with --force-clean."
        echo "        Example: pkill -f 'run_uuv_mujoco.py|run_urdf_full.py'"
        exit 1
    fi
fi

# Set display for headless mode
if [ "$HEADLESS" = true ]; then
    HEADLESS_ARG="--headless"
    export DISPLAY=""
    case "$HOST_OS" in
        Darwin)
            export MUJOCO_GL=cgl
            echo "[launch] Running in headless mode (CGL rendering on macOS)"
            ;;
        Linux)
            export MUJOCO_GL=egl
            echo "[launch] Running in headless mode (EGL rendering)"
            ;;
        *)
            unset MUJOCO_GL
            echo "[launch] Running in headless mode (default MuJoCo GL backend)"
            ;;
    esac
fi

configure_mujoco_viewer_window_backend

if [[ -n "$SITL_ARG" ]]; then
    # ArduSub/sim2real run-mode contract. Do not patch ArduSub controller
    # behavior here: choose only which actuator stream owns the MuJoCo plant.
    UUV_RUN_MODE="$(printf '%s' "${UUV_RUN_MODE:-closed_loop}" | tr '[:upper:]' '[:lower:]')"
    case "$UUV_RUN_MODE" in
        closed_loop|plant_replay)
            ;;
        *)
            echo "[launch] invalid UUV_RUN_MODE=${UUV_RUN_MODE}; expected closed_loop or plant_replay" >&2
            exit 2
            ;;
    esac
    export UUV_RUN_MODE

    # Keep live GUI/QGC-style pilot input on ArduSub's raw RC override path.
    # MANUAL_CONTROL remains available as an explicit diagnostic backend.
    export UUV_GUI_PILOT_CONTROL_MODE="${UUV_GUI_PILOT_CONTROL_MODE:-rc_override}"
    export ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND="${ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND:-rc_override}"
    export UUV_GUI_RC_PWM_SPAN="${UUV_GUI_RC_PWM_SPAN:-300}"
    export ROS2_UUV_MAVROS_RC_PWM_SPAN="${ROS2_UUV_MAVROS_RC_PWM_SPAN:-300}"
    export ROS2_UUV_SITL_ALLOW_DIRECT_CMD="${ROS2_UUV_SITL_ALLOW_DIRECT_CMD:-0}"
    export ROS2_UUV_SITL_CMD_VEL_SETPOINT_ENABLE="${ROS2_UUV_SITL_CMD_VEL_SETPOINT_ENABLE:-0}"
    export ROS2_UUV_MAVROS_SETPOINT_ENABLE="${ROS2_UUV_MAVROS_SETPOINT_ENABLE:-0}"
    export ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK="${ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK:-0}"
    export UUV_SITL_INITIAL_BAR30_DEPTH_M="${UUV_SITL_INITIAL_BAR30_DEPTH_M:-auto}"
    export SITL_DEDICATED_COMMAND_MAVLINK="${SITL_DEDICATED_COMMAND_MAVLINK:-1}"
    export SITL_COMMAND_MAV_PORT="${SITL_COMMAND_MAV_PORT:-14661}"
    export SITL_TCP_MAVLINK_PORT="${SITL_TCP_MAVLINK_PORT:-5760}"
    if [[ "${SITL_DEDICATED_COMMAND_MAVLINK}" == "1" ]]; then
        export ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT="${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT:-udpin:0.0.0.0:${SITL_COMMAND_MAV_PORT}}"
    else
        export ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT="${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT:-}"
    fi
    # Real ArduSub-4.1.2 controller/runtime timing contract.
    # The hardware dump uses SCHED_LOOP_RATE=400, and controller-parity replay
    # showed that 100Hz SITL changes RCOU shape. Keep sensor feed and plant
    # force updates on the same default cadence unless explicitly overridden.
    export SITL_SCHED_LOOP_RATE="${SITL_SCHED_LOOP_RATE:-400}"
    export SITL_SENSOR_HZ_DEFAULT="${SITL_SENSOR_HZ_DEFAULT:-${SITL_SCHED_LOOP_RATE}}"
    export SITL_THRUSTER_LOOP_HZ_DEFAULT="${SITL_THRUSTER_LOOP_HZ_DEFAULT:-${SITL_SCHED_LOOP_RATE}}"
    export UUV_ROS2_SENSOR_HZ="${UUV_ROS2_SENSOR_HZ:-${SITL_SENSOR_HZ_DEFAULT}}"
    export UUV_THRUSTER_LOOP_HZ="${UUV_THRUSTER_LOOP_HZ:-${SITL_THRUSTER_LOOP_HZ_DEFAULT}}"
    # BARO_PRIMARY=1 in the real dump selects the second Bar30 ground pressure.
    # Use that as the default simulated surface pressure so AP_Baro_SITL sees
    # the same pressure datum as the real-controller replay path.
    if [[ -n "${ROS2_UUV_BAR30_SURFACE_PRESSURE_PA+x}" ]]; then
        UUV_BAR30_SURFACE_PRESSURE_USER_SET=1
    else
        UUV_BAR30_SURFACE_PRESSURE_USER_SET=0
    fi
    export UUV_BAR30_SURFACE_PRESSURE_USER_SET
    export ROS2_UUV_BAR30_SURFACE_PRESSURE_PA="${ROS2_UUV_BAR30_SURFACE_PRESSURE_PA:-101640.0}"
    if [[ "$UUV_RUN_MODE" == "plant_replay" ]]; then
        export ROS2_UUV_SITL_JSON_SERVO_FALLBACK="${ROS2_UUV_SITL_JSON_SERVO_FALLBACK:-0}"
        export ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE="${ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE:-1}"
        echo "[mode] plant_replay: recorded actuator PWM/RCOU is authoritative plant input"
    elif [[ "${SITL_DIRECT_MAVLINK:-0}" == "1" ]]; then
        export ROS2_UUV_SITL_JSON_SERVO_FALLBACK="${ROS2_UUV_SITL_JSON_SERVO_FALLBACK:-0}"
        export ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE="${ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE:-0}"
        echo "[mode] closed_loop/native-direct: MAVLink SERVO_OUTPUT_RAW is authoritative plant input"
    else
        export ROS2_UUV_SITL_JSON_SERVO_FALLBACK="${ROS2_UUV_SITL_JSON_SERVO_FALLBACK:-1}"
        export ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE="${ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE:-0}"
        echo "[mode] closed_loop: raw ArduSub JSON servo is authoritative plant input"
    fi
    # Default GUI/live-control contract is Bar30 POSZ plus JSON IMU only.
    # The real 4.1.2 ExternalNav/DVL parity contract is still available by
    # explicitly setting UUV_EKF_CONTRACT=real_param_parity or poshold_extnav.
    export UUV_EKF_CONTRACT="${UUV_EKF_CONTRACT:-althold_baro}"
    LIVE_EXTNAV_HZ_DEFAULT=15
    if [[ -n "${ROS2_UUV_SITL_SENSOR_REPLAY_PREVIEW_CSV:-}" || -n "${ROS2_UUV_SITL_SENSOR_REPLAY_VPD_CSV:-}" ]]; then
        LIVE_EXTNAV_HZ_DEFAULT=10
    fi
    case "$UUV_EKF_CONTRACT" in
        real_param_parity)
            export UUV_EKF_CONTRACT="real_param_parity"
            export SITL_EKF3_EXTNAV=1
            export SITL_EKF3_EXTNAV_POSZ=1
            export SITL_EKF3_EXTNAV_VELZ=6
            export SITL_AHRS_EKF_TYPE="${SITL_AHRS_EKF_TYPE:-3}"
            export ROS2_UUV_SITL_EXTNAV_ENABLE=1
            export ROS2_UUV_SITL_EXTNAV_HZ="${ROS2_UUV_SITL_EXTNAV_HZ:-$LIVE_EXTNAV_HZ_DEFAULT}"
            export ROS2_UUV_REQUIRE_EXTNAV_TX=1
            export ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE="${ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE:-1}"
            ;;
        poshold_extnav|poshold_extnav_412|real-ekf|real_ekf|extnav)
            export SITL_EKF3_EXTNAV=1
            export SITL_EKF3_EXTNAV_POSZ=1
            export SITL_EKF3_EXTNAV_VELZ=6
            export SITL_AHRS_EKF_TYPE="${SITL_AHRS_EKF_TYPE:-3}"
            export ROS2_UUV_SITL_EXTNAV_ENABLE=1
            export ROS2_UUV_SITL_EXTNAV_HZ="${ROS2_UUV_SITL_EXTNAV_HZ:-$LIVE_EXTNAV_HZ_DEFAULT}"
            export ROS2_UUV_REQUIRE_EXTNAV_TX=1
            export ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE="${ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE:-1}"
            ;;
        althold_baro|baro|baro-ekf|depthhold_baro)
            export UUV_EKF_CONTRACT="althold_baro"
            export SITL_EKF3_EXTNAV=0
            export SITL_EKF3_EXTNAV_POSZ=1
            export SITL_EKF3_EXTNAV_VELZ=0
            export SITL_AHRS_EKF_TYPE="${SITL_AHRS_EKF_TYPE:-3}"
            export ROS2_UUV_SITL_EXTNAV_ENABLE=0
            export ROS2_UUV_REQUIRE_EXTNAV_TX=0
            export ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE="${ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE:-0}"
            ;;
        *)
            echo "[launch] unknown UUV_EKF_CONTRACT=$UUV_EKF_CONTRACT" >&2
            exit 2
            ;;
    esac
    if [[ "${ROS2_UUV_SITL_BRIDGE_EXTNAV_DISABLE:-0}" == "1" ]]; then
        # Controller-parity direct-MAVLink mode owns the native VPD stream.
        # Keep ArduSub EKF ExternalNav parameters as requested by
        # UUV_EKF_CONTRACT, but do not let the MuJoCo bridge synthesize a
        # second VISION_POSITION_DELTA stream from live plant state.
        export ROS2_UUV_SITL_EXTNAV_ENABLE=0
        export ROS2_UUV_REQUIRE_EXTNAV_TX=0
        echo "[launch] bridge ExternalNav TX disabled by ROS2_UUV_SITL_BRIDGE_EXTNAV_DISABLE=1"
    fi
    export ROS2_UUV_EXTNAV_MIN_TX_HZ="${ROS2_UUV_EXTNAV_MIN_TX_HZ:-10}"
    export ROS2_UUV_EXTNAV_TX_GRACE_S="${ROS2_UUV_EXTNAV_TX_GRACE_S:-6}"
    export ROS2_UUV_EXTNAV_MAX_STALE_S="${ROS2_UUV_EXTNAV_MAX_STALE_S:-2.0}"
    # Start SITL with the pressure sensor submerged by default. This sets the
    # launch pose before any sensor packet is published; it is not a runtime
    # hold, guard, or ALT_HOLD shim.
    # The bridge owns the ALT_HOLD vertical feedback contract: JSON position.z
    # is Bar30-derived positive-down depth, IMU comes from base_link, and the
    # real-param-parity profile sends body-frame VPD instead of VISION_SPEED.
    export ROS2_UUV_ARM_MODE_BOOT_GUARD_S="${ROS2_UUV_ARM_MODE_BOOT_GUARD_S:-4}"
fi

apply_real_start_state_args

echo "[launch] Starting MuJoCo UUV Simulation"
echo "[launch] Scene: ${SCENE_LABEL}"
echo "[launch] Fluid model: ${FLUID_MODEL}"
echo "[launch] Python launcher: ${PY_LAUNCHER}"
if [[ "$FLUID_MODEL" == "current" ]]; then
    HYDRO_DESCRIPTION="MuJoCo ellipsoid fluidcoef"
else
    HYDRO_DESCRIPTION="Python-owned hydrodynamic wrenches"
fi
if [ "$ROS2_REQUESTED" = true ]; then
    echo "[launch] Source ROS2 environment for --ros2 topics."
    ROS_SETUP_FILE=""
    if ROS_SETUP_FILE="$(resolve_ros_setup_for_bash)"; then
        echo "[launch] ROS2 setup: ${ROS_SETUP_FILE}"
        source_setup_bash_safely "$ROS_SETUP_FILE"
    else
        echo "[launch] ROS2 setup file not found; relying on current shell environment."
    fi
    if [[ -f "$ROS_WORKSPACE_SETUP" && "$ROS_WORKSPACE_SETUP" != "$ROS_SETUP_FILE" ]]; then
        echo "[launch] ROS2 workspace setup: ${ROS_WORKSPACE_SETUP}"
        source_setup_bash_safely "$ROS_WORKSPACE_SETUP"
    fi
    if [[ "$HOST_OS" == "Darwin" && -z "${RMW_IMPLEMENTATION:-}" ]]; then
        # The MuJoCo runtime executes with a non-conda Python while ROS2
        # message/service type support is loaded from the conda Humble env.
        # FastDDS can discover those services but rejects MAVROS request
        # payloads at runtime; CycloneDDS preserves the MAVROS service facade.
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        echo "[launch] ROS2 RMW default: ${RMW_IMPLEMENTATION} (macOS deterministic bridge)"
    fi
fi
echo "[launch] Bridge:"
if [ "$ROS2_REQUESTED" = true ]; then
    echo "  ROS2 Transport: enabled"
    if [[ -n "$SITL_ARG" ]]; then
        echo "    Input:  /mavros/rc/override -> ArduSub closed-loop"
        echo "            GUI pilot input mode: ${UUV_GUI_PILOT_CONTROL_MODE}"
        echo "            set ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK=1 only for direct MuJoCo smoke/debug"
        echo "            /cmd_vel is ignored in SITL by default; set ROS2_UUV_SITL_CMD_VEL_SETPOINT_ENABLE=1 only for guided-setpoint smoke tests"
        echo "            /mavros/setpoint_raw/local enabled only with ROS2_UUV_MAVROS_SETPOINT_ENABLE=1"
        echo "            /uuv_mujoco/rc/out_override enabled only with ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE=1"
        echo "    Run mode: ${UUV_RUN_MODE}"
        echo "    Hydrodynamics: ${HYDRO_DESCRIPTION} (profile=${PROFILE:-runtime default})"
        echo "    RCOUT plant override: ${ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE}"
        echo "    Command MAVLink: ${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT}"
        echo "    SITL scheduler: ${SITL_SCHED_LOOP_RATE} Hz"
        echo "    SITL sensor feed: ${SITL_SENSOR_HZ_DEFAULT} Hz"
        echo "    SITL thruster loop: ${SITL_THRUSTER_LOOP_HZ_DEFAULT} Hz"
        echo "    Bar30 surface pressure: ${ROS2_UUV_BAR30_SURFACE_PRESSURE_PA} Pa"
        if [[ "${UUV_REAL_START_STATE_APPLIED:-0}" == "1" ]]; then
            echo "    SITL start: real-state CSV ${UUV_REAL_START_STATE_CSV} @ ${UUV_REAL_START_SOURCE_T_S}s"
            echo "                base xy ${UUV_REAL_START_BASE_X_M}, ${UUV_REAL_START_BASE_Y_M} m; base depth ${UUV_REAL_START_BASE_DEPTH_M} m, depth topic ${UUV_REAL_START_DEPTH_M} m, mode=${UUV_REAL_START_MODE}, armed=${UUV_REAL_START_ARMED}"
        else
            echo "    SITL start: Bar30 initial depth default ${UUV_SITL_INITIAL_BAR30_DEPTH_M} m"
        fi
        echo "    SITL vertical feedback: Bar30 depth + base_link/Pixhawk IMU"
        echo "    SITL EKF3 ExternalNav: ${ROS2_UUV_SITL_EXTNAV_ENABLE}"
        echo "    SITL DVL rangefinder payload: ${ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE}"
    else
        echo "    Input:  /cmd_vel (TwistStamped), /mavros/rc/override direct fallback"
    fi
    echo "    Output: /imu/data, /dvl/velocity, /dvl/twist, /dvl/odometry, /dvl/altitude"
    echo "            /dvl/data, /dvl/position, /depth, /depth/pose, /bar30/pressure_pa"
    echo "            /rovio/odometry, /sim/odom, /tf, /tf_static, /robot_description"
    echo "            /ping360/image, /ping360/scan_image, /ping360/scan, /ping360/scan_echo, /ping360/echo"
    echo "    Debug:  /mujoco/ground_truth/pose, /mujoco/course_buoys/status"
    if [[ "$REAL_PKG_COMPAT" == true ]]; then
        echo "    MAVROS: external node owns commands/state/local position and /mavros/imu/data (FCU AHRS)"
        echo "            bridge owns delivery-driven /mavros/imu/data_raw and /mavros/imu/static_pressure"
        if [[ -n "$SITL_ARG" ]]; then
            echo "            use hit25_auv_ros2 with fcu_url:=udp://0.0.0.0:14551@"
            echo "            (listen for the ArduSub/MAVProxy output and learn its UDP peer)"
        fi
    else
        echo "    MAVROS: full lightweight surface enabled"
        echo "            /mavros/vision_pose/pose, /mavros/battery, /mavros/rc/*"
    fi
    if [ -n "$IMAGES" ]; then
        echo "    Images: disabled in lightweight real-robot bridge (legacy flag ignored)"
    fi
else
    echo "  ROS2 Transport: disabled (--no-ros2; SITL UDP JSON only)"
fi
if [[ -n "$SITL_ARG" ]]; then
    echo "[launch] SITL UDP JSON:"
    echo "  listen port  = ${SITL_PORT}  (ArduPilot --sim-port-out, where servo packets are sent)"
    echo "  send target = ${SITL_SEND_PORT}  (ArduPilot --sim-port-in, where sensor packets are sent)"
fi
echo ""

if [[ -n "$SITL_ARG" ]]; then
    SITL_SERVO_SOURCE="json"
    SITL_MAVLINK_ENDPOINT_VALUE=""
    if [[ "$SITL_PORT" == "$SITL_SEND_PORT" ]]; then
        echo "[warn] --sitl-port and --sitl-send-port are identical (${SITL_PORT}); SITL usually requires distinct direction-specific UDP ports."
    fi
    # Intentionally do not force extra stabilization/depth-hold for SITL.
    # ArduPilot/QGC depth and attitude modes should own these loops.
    if ! extra_arg_present "--initial-depth-m" && ! extra_arg_present "--initial-bar30-depth-m"; then
        initial_bar30_depth_raw="${UUV_SITL_INITIAL_BAR30_DEPTH_M:-}"
        drop_start_raw="${UUV_SITL_DROP_START_ABOVE_WATER:-0}"
        drop_start_lc="$(printf '%s' "$drop_start_raw" | tr '[:upper:]' '[:lower:]')"
        initial_bar30_depth_lc="$(printf '%s' "$initial_bar30_depth_raw" | tr '[:upper:]' '[:lower:]')"
        case "$drop_start_lc" in
            1|true|yes|on|enable|enabled)
                echo "[launch] SITL mode: drop-start requested; not injecting initial Bar30 depth."
                ;;
            *)
                case "$initial_bar30_depth_lc" in
                    ""|off|none|false)
                        echo "[launch] SITL mode: initial Bar30 depth disabled by UUV_SITL_INITIAL_BAR30_DEPTH_M=${initial_bar30_depth_raw:-<empty>}."
                        ;;
                    *)
                        EXTRA_ARGS+=("--initial-bar30-depth-m" "$initial_bar30_depth_raw")
                        echo "[launch] SITL mode: starting with Bar30 depth ${initial_bar30_depth_raw} m."
                        ;;
                esac
                ;;
        esac
    fi
    if [[ -z "$PROFILE" ]]; then
        PROFILE="current"
        replace_extra_arg_value --profile "$PROFILE"
    fi
    SITL_SENSOR_HZ_DEFAULT="${SITL_SENSOR_HZ_DEFAULT:-${PROFILE_SENSOR_HZ}}"
    SITL_THRUSTER_LOOP_HZ_DEFAULT="${SITL_THRUSTER_LOOP_HZ_DEFAULT:-${PROFILE_THRUSTER_HZ}}"
    SITL_MAVLINK_SERVO_HZ_DEFAULT="${SITL_MAVLINK_SERVO_HZ_DEFAULT:-50}"
    MUJOCO_VIEWER_FPS_DEFAULT="${UUV_MUJOCO_VIEWER_FPS:-${PROFILE_VIEWER_FPS}}"
    echo "[launch] runtime profile: ${UUV_RUNTIME_PROFILE} (sensor=${SITL_SENSOR_HZ_DEFAULT}Hz, thruster=${SITL_THRUSTER_LOOP_HZ_DEFAULT}Hz, viewer=${MUJOCO_VIEWER_FPS_DEFAULT}Hz)"
    # Keep the default profile responsive without saturating CPU/ROS on native desktops.
    if [[ "$HOST_OS" == "Darwin" ]]; then
        if append_extra_arg_if_missing "--ros2-sensor-hz" --ros2-sensor-hz "${SITL_SENSOR_HZ_DEFAULT}"; then
            echo "[launch] SITL mode: using sensor publish rate ${SITL_SENSOR_HZ_DEFAULT} Hz (override with --ros2-sensor-hz)."
        fi
    elif append_extra_arg_if_missing "--ros2-sensor-hz" --ros2-sensor-hz "${SITL_SENSOR_HZ_DEFAULT}"; then
        echo "[launch] SITL mode: using sensor publish rate ${SITL_SENSOR_HZ_DEFAULT} Hz (override with --ros2-sensor-hz)."
    fi
    if append_extra_arg_if_missing "--thruster-loop-hz" --thruster-loop-hz "${SITL_THRUSTER_LOOP_HZ_DEFAULT}"; then
        echo "[launch] SITL mode: using thruster loop rate ${SITL_THRUSTER_LOOP_HZ_DEFAULT} Hz (override with --thruster-loop-hz)."
    fi
    if [[ "$HEADLESS" != true ]]; then
        append_extra_arg_if_missing "--viewer-fps" --viewer-fps "${MUJOCO_VIEWER_FPS_DEFAULT}" >/dev/null || true
    fi
    if extra_arg_present "--sitl-mavlink-endpoint"; then
        SITL_MAVLINK_ENDPOINT_VALUE="$(extra_arg_value "--sitl-mavlink-endpoint" || true)"
    else
        SITL_MAVLINK_ENDPOINT_VALUE="udpin:0.0.0.0:14660"
    fi
    if [[ "$UUV_RUN_MODE" == "plant_replay" ]]; then
        echo "[launch] SITL mode: plant_replay uses recorded actuator PWM/RCOU as plant input."
        echo "[launch] SITL mode: JSON/MAVLink live servo streams remain telemetry/estimator context only."
    elif [[ "${SITL_DIRECT_MAVLINK:-0}" == "1" ]]; then
        echo "[launch] SITL mode: using MAVLink SERVO_OUTPUT_RAW as the native/direct plant input."
        echo "[launch] SITL mode: JSON servo packets remain telemetry/sensor-reply context only."
    else
        echo "[launch] SITL mode: using servo source json (standard ArduPilot SITL UDP servo packets)."
        echo "[launch] SITL mode: MAVLink SERVO_OUTPUT_RAW kept for heartbeat/telemetry only."
    fi
    append_extra_arg_if_missing "--sitl-mavlink-target-sysid" --sitl-mavlink-target-sysid "${SITL_MAVLINK_TARGET_SYSID}" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-target-compid" --sitl-mavlink-target-compid "${SITL_MAVLINK_TARGET_COMPID}" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-source-sysid" --sitl-mavlink-source-sysid "${SITL_MAVLINK_SOURCE_SYSID}" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-source-compid" --sitl-mavlink-source-compid "${SITL_MAVLINK_SOURCE_COMPID}" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-endpoint" --sitl-mavlink-endpoint "udpin:0.0.0.0:14660" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-servo-hz" --sitl-mavlink-servo-hz "${SITL_MAVLINK_SERVO_HZ_DEFAULT}" >/dev/null || true
    # Legacy polynomial/gain tuned mode used:
    # append_extra_arg_if_missing "--sitl-servo-scale" --sitl-servo-scale "0.58" >/dev/null || true
    append_extra_arg_if_missing "--sitl-servo-scale" --sitl-servo-scale "${SITL_SERVO_SCALE_DEFAULT:-1.0}" >/dev/null || true
    # Final ArduSub PWM drives the measured T200/Basic ESC surface exactly once.
    # Voltage is configurable and can follow a recorded ESC-bus voltage trace.
    SITL_THRUSTER_FORCE_MODEL="${UUV_SITL_THRUSTER_FORCE_MODEL:-t200}"
    if extra_arg_present "--disable-thruster-perf" && extra_arg_present "--thruster-perf-direct"; then
        echo "[launch] ERROR: --disable-thruster-perf and --thruster-perf-direct are mutually exclusive." >&2
        exit 2
    fi
    if extra_arg_present "--disable-thruster-perf"; then
        SITL_THRUSTER_FORCE_MODEL="polynomial"
    elif extra_arg_present "--thruster-perf-direct"; then
        SITL_THRUSTER_FORCE_MODEL="t200"
    fi
    case "${SITL_THRUSTER_FORCE_MODEL}" in
        polynomial)
            append_extra_arg_if_missing "--disable-thruster-perf" --disable-thruster-perf >/dev/null || true
            echo "[launch] SITL mode: using configured polynomial thruster model (fixed-voltage T200 curve disabled)."
            ;;
        t200)
            append_extra_arg_if_missing "--thruster-perf-direct" --thruster-perf-direct >/dev/null || true
            echo "[launch] SITL mode: measured T200/Basic ESC PWM model; bus voltage comes from profile/CLI or voltage trace."
            ;;
        *)
            echo "[launch] ERROR: UUV_SITL_THRUSTER_FORCE_MODEL must be polynomial or t200 (got ${SITL_THRUSTER_FORCE_MODEL})." >&2
            exit 2
            ;;
    esac
    if [[ -n "${UUV_THRUSTER_BUS_VOLTAGE_V:-}" ]]; then
        append_extra_arg_if_missing "--thruster-voltage" --thruster-voltage "${UUV_THRUSTER_BUS_VOLTAGE_V}" >/dev/null || true
    fi
    if [[ -n "${UUV_THRUSTER_VOLTAGE_TRACE:-}" ]]; then
        append_extra_arg_if_missing "--thruster-voltage-trace" --thruster-voltage-trace "${UUV_THRUSTER_VOLTAGE_TRACE}" >/dev/null || true
    fi
    # Lower input latency defaults for SITL command loops.
    : "${ROS2_UUV_CMD_DEADBAND:=0.0}"
    : "${ROS2_UUV_CMD_SLEW_RATE:=200.0}"
    : "${ROS2_UUV_DVL_LPF_ALPHA:=1.0}"
    : "${ROS2_UUV_BAR30_NOISE_PA_STD:=0.0}"
    : "${ROS2_UUV_CMD_TIMEOUT_S:=0.25}"
    : "${ROS2_UUV_SPIN_TIMEOUT_S:=0.001}"
    : "${ROS2_UUV_SITL_MAVLINK_TIMEOUT_S:=1.5}"
    export ROS2_UUV_CMD_DEADBAND ROS2_UUV_CMD_SLEW_RATE ROS2_UUV_DVL_LPF_ALPHA ROS2_UUV_BAR30_NOISE_PA_STD ROS2_UUV_CMD_TIMEOUT_S ROS2_UUV_SPIN_TIMEOUT_S ROS2_UUV_SITL_MAVLINK_TIMEOUT_S
    echo "[launch] SITL mode: simple sensor path deadband=${ROS2_UUV_CMD_DEADBAND}, slew=${ROS2_UUV_CMD_SLEW_RATE}/s, dvl_alpha=${ROS2_UUV_DVL_LPF_ALPHA}, bar30_noise=${ROS2_UUV_BAR30_NOISE_PA_STD}Pa, timeout=${ROS2_UUV_CMD_TIMEOUT_S}s, spin_timeout=${ROS2_UUV_SPIN_TIMEOUT_S}s, mavlink_timeout=${ROS2_UUV_SITL_MAVLINK_TIMEOUT_S}s"
    if [[ "$QGC_VIDEO_AUTO" != "off" ]] && ! extra_arg_present "--qgc-video"; then
        EXTRA_ARGS+=("--qgc-video")
        echo "[launch] SITL mode: enabling direct QGC video stream."
    fi
    if extra_arg_present "--qgc-video" && [[ "$HOST_OS" == "Darwin" ]]; then
        if append_extra_arg_if_missing "--qgc-video-fps" --qgc-video-fps 8; then
            echo "[launch] macOS: lowering QGC video FPS to 8 by default (override with --qgc-video-fps)."
        fi
        if append_extra_arg_if_missing "--qgc-video-width" --qgc-video-width 512; then
            echo "[launch] macOS: lowering QGC video width to 512 by default (override with --qgc-video-width)."
        fi
        if append_extra_arg_if_missing "--qgc-video-height" --qgc-video-height 288; then
            echo "[launch] macOS: lowering QGC video height to 288 by default (override with --qgc-video-height)."
        fi
        if append_extra_arg_if_missing "--qgc-video-bitrate-kbps" --qgc-video-bitrate-kbps 900; then
            echo "[launch] macOS: lowering QGC video bitrate to 900 kbps by default (override with --qgc-video-bitrate-kbps)."
        fi
    fi
fi

if [ "$ROS2_REQUESTED" = true ] && ! extra_arg_present "--ros2"; then
    EXTRA_ARGS+=(--ros2)
fi

case "$PROFILE" in
    custom)
        PROFILE="legacy"
        replace_extra_arg_value --profile "$PROFILE"
        ;;
    ellipsoid)
        PROFILE="current"
        replace_extra_arg_value --profile "$PROFILE"
        ;;
esac

if [[ "$FLUID_MODEL" == "distributed" ]]; then
    case "$PROFILE" in
        ""|current|legacy|research_pool)
            echo "[error] --fluid-model distributed requires an explicit profile with distributed_hydrodynamics enabled." >&2
            echo "        Recommended: --profile research_pool_distributed" >&2
            exit 2
            ;;
    esac
fi

if [[ -n "$PROFILE" ]]; then
    echo "[launch] Simulation profile: $PROFILE"
fi

RUN_ARGS=(--scene "$SCENE_PATH" --fluid-model "$FLUID_MODEL")
if [[ -n "$IMAGES" ]]; then
    RUN_ARGS+=("$IMAGES")
fi
if [[ -n "$SITL_ARG" ]]; then
    RUN_ARGS+=("$SITL_ARG" --sitl-port "$SITL_PORT" --sitl-send-port "$SITL_SEND_PORT")
fi
if [[ -n "$HEADLESS_ARG" ]]; then
    RUN_ARGS+=("$HEADLESS_ARG")
fi
if ((${#EXTRA_ARGS[@]})); then
    RUN_ARGS+=("${EXTRA_ARGS[@]}")
fi

"$PY_LAUNCHER" run_uuv_mujoco.py "${RUN_ARGS[@]}"
