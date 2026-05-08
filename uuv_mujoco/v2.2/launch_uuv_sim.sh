#!/bin/bash
# Launch MuJoCo UUV simulation
# Usage:
#   ./launch_uuv_sim.sh [--headless] [--sitl] [--images] [--no-ros2] [--ros2] [--ros2-real-pkg-compat] [--qgc-video] [--force-clean] [--scene <path>] [--tank-549x274x132] [--fluid-model <name>]
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
ROS_WORKSPACE_SETUP="${ROS_WORKSPACE_SETUP:-${PROJECT_ROOT}/rospkg/install/setup.bash}"

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

MJ311_MJPYTHON="$(resolve_mjpython)"
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
SITL_MAVLINK_SOURCE_SYSID=200
SITL_MAVLINK_SOURCE_COMPID=190

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
        --tank-549x274x132)
            SCENE_PATH="scenes/tank_legacy_scene.xml"
            shift
            ;;
        --legacy-scene)
            SCENE_PATH="scenes/tank_legacy_scene.xml"
            shift
            ;;
        --current-scene)
            SCENE_PATH="scenes/tank_current_scene.xml"
            shift
            ;;
        --custom-scene)
            SCENE_PATH="scenes/tank_legacy_scene.xml"
            shift
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
            echo "        v2.2 standard path uses mavlink SITL control and no internal hover helper."
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

resolve_scene_path() {
    local requested="$1"
    if [[ -z "$requested" ]]; then
        echo "[error] empty scene path requested." >&2
        return 1
    fi
    case "$requested" in
        tank_legacy_scene.xml)
            requested="scenes/tank_legacy_scene.xml"
            ;;
        tank_current_scene.xml)
            requested="scenes/tank_current_scene.xml"
            ;;
        tank_custom_scene.xml)
            requested="scenes/tank_legacy_scene.xml"
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
    legacy|custom|current|ellipsoid|builtin-ellipsoid)
        ;;
    *)
        echo "[error] unknown --fluid-model: ${FLUID_MODEL}" >&2
        echo "        expected one of: legacy, current" >&2
        exit 2
        ;;
esac

case "$FLUID_MODEL" in
    custom)
        FLUID_MODEL="legacy"
        ;;
    ellipsoid|builtin-ellipsoid)
        FLUID_MODEL="current"
        ;;
esac

if [[ "$(basename "$SCENE_PATH")" == "tank_legacy_scene.xml" && "$FLUID_MODEL" == "current" ]]; then
    SCENE_PATH="scenes/tank_current_scene.xml"
fi

SCENE_PATH="$(resolve_scene_path "$SCENE_PATH")"
SCENE_LABEL="$(basename "$SCENE_PATH")"

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

collect_existing_mujoco_pids() {
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
        echo "        Example: pkill -f 'run_urdf_full.py'"
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

if [[ -n "$SITL_ARG" ]]; then
    # In SITL mode, /mavros/rc/override must pass through ArduSub first.  The
    # standalone MuJoCo fallback is useful without SITL, but it corrupts closed-loop
    # ALT_HOLD tests by injecting the same RC command directly into the plant.
    export ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK="${ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK:-0}"
fi

echo "[launch] Starting MuJoCo UUV Simulation"
echo "[launch] Scene: ${SCENE_LABEL}"
echo "[launch] Fluid model: ${FLUID_MODEL}"
echo "[launch] Python launcher: ${MJ311_MJPYTHON}"
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
fi
echo "[launch] Bridge:"
if [ "$ROS2_REQUESTED" = true ]; then
    echo "  ROS2 Transport: enabled"
    if [[ -n "$SITL_ARG" ]]; then
        echo "    Input:  /mavros/rc/override -> ArduSub closed-loop"
        echo "            /cmd_vel remains available for standalone bridge tests"
    else
        echo "    Input:  /cmd_vel (TwistStamped), /mavros/rc/override direct fallback"
    fi
    echo "    Output: /imu/data, /dvl/velocity, /dvl/twist, /dvl/odometry, /dvl/altitude"
    echo "            /dvl/data, /dvl/position, /depth, /depth/pose, /bar30/pressure_pa"
    echo "            /rovio/odometry, /sim/odom, /tf, /tf_static, /robot_description"
    echo "            /ping360/image, /ping360/scan_image, /ping360/scan, /ping360/scan_echo, /ping360/echo"
    echo "    Debug:  /mujoco/ground_truth/pose"
    if [[ "$REAL_PKG_COMPAT" == true ]]; then
        echo "    MAVROS: compat-only in simulator (/mavros/vfr_hud only; external MAVROS expected)"
        if [[ -n "$SITL_ARG" ]]; then
            echo "            use hit25_auv_ros2 with fcu_url:=udp://:14551@127.0.0.1:14551"
            echo "            (ArduSub direct MAVLink serial2 output; MAVProxy not required)"
        fi
    else
        echo "    MAVROS: full lightweight surface enabled"
        echo "            /mavros/state, /mavros/imu/*, /mavros/local_position/*"
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
    SITL_SERVO_SOURCE="mavlink"
    SITL_MAVLINK_ENDPOINT_VALUE=""
    if [[ "$SITL_PORT" == "$SITL_SEND_PORT" ]]; then
        echo "[warn] --sitl-port and --sitl-send-port are identical (${SITL_PORT}); SITL usually requires distinct direction-specific UDP ports."
    fi
    # Intentionally do not force extra stabilization/depth-hold for SITL.
    # ArduPilot/QGC depth and attitude modes should own these loops.
    if [[ -z "$PROFILE" ]]; then
        if [[ "$FLUID_MODEL" == "current" ]]; then
            PROFILE="current"
        else
            PROFILE="legacy"
        fi
        replace_extra_arg_value --profile "$PROFILE"
    fi
    # Raise SITL sensor feed rate by default to reduce EKF lag in stabilize mode.
    if [[ "$HOST_OS" == "Darwin" ]]; then
        if append_extra_arg_if_missing "--ros2-sensor-hz" --ros2-sensor-hz 120; then
            echo "[launch] SITL mode: using macOS-friendly sensor publish rate 120 Hz (override with --ros2-sensor-hz)."
        fi
    elif append_extra_arg_if_missing "--ros2-sensor-hz" --ros2-sensor-hz 300; then
        echo "[launch] SITL mode: overriding sensor publish rate to 300 Hz (use --ros2-sensor-hz to set manually)."
    fi
    if append_extra_arg_if_missing "--thruster-loop-hz" --thruster-loop-hz 150; then
        echo "[launch] SITL mode: overriding thruster loop rate to 150 Hz (use --thruster-loop-hz to set manually)."
    fi
    if extra_arg_present "--sitl-mavlink-endpoint"; then
        SITL_MAVLINK_ENDPOINT_VALUE="$(extra_arg_value "--sitl-mavlink-endpoint" || true)"
    else
        SITL_MAVLINK_ENDPOINT_VALUE="udpin:0.0.0.0:14660"
    fi
    case "$(printf '%s' "$SITL_MAVLINK_ENDPOINT_VALUE" | tr '[:upper:]' '[:lower:]')" in
        none|off|disabled|disable)
            SITL_SERVO_SOURCE="json"
            ;;
    esac
    if [[ "$SITL_SERVO_SOURCE" == "mavlink" ]]; then
        echo "[launch] SITL mode: using servo source mavlink (SERVO_OUTPUT_RAW)."
    else
        echo "[launch] SITL mode: using servo source json (standard ArduPilot SITL UDP servo packets)."
    fi
    append_extra_arg_if_missing "--sitl-mavlink-target-sysid" --sitl-mavlink-target-sysid "${SITL_MAVLINK_TARGET_SYSID}" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-target-compid" --sitl-mavlink-target-compid "${SITL_MAVLINK_TARGET_COMPID}" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-source-sysid" --sitl-mavlink-source-sysid "${SITL_MAVLINK_SOURCE_SYSID}" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-source-compid" --sitl-mavlink-source-compid "${SITL_MAVLINK_SOURCE_COMPID}" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-endpoint" --sitl-mavlink-endpoint "udpin:0.0.0.0:14660" >/dev/null || true
    append_extra_arg_if_missing "--sitl-mavlink-servo-hz" --sitl-mavlink-servo-hz 50 >/dev/null || true
    append_extra_arg_if_missing "--sitl-servo-scale" --sitl-servo-scale "0.58" >/dev/null || true
    # Lower input latency defaults for SITL command loops.
    : "${ROS2_UUV_CMD_DEADBAND:=0.0}"
    : "${ROS2_UUV_CMD_SLEW_RATE:=200.0}"
    : "${ROS2_UUV_DVL_LPF_ALPHA:=1.0}"
    : "${ROS2_UUV_BAR30_NOISE_PA_STD:=0.0}"
    : "${ROS2_UUV_CMD_TIMEOUT_S:=0.45}"
    : "${ROS2_UUV_SITL_MAVLINK_TIMEOUT_S:=1.5}"
    export ROS2_UUV_CMD_DEADBAND ROS2_UUV_CMD_SLEW_RATE ROS2_UUV_DVL_LPF_ALPHA ROS2_UUV_BAR30_NOISE_PA_STD ROS2_UUV_CMD_TIMEOUT_S ROS2_UUV_SITL_MAVLINK_TIMEOUT_S
    echo "[launch] SITL mode: simple sensor path deadband=${ROS2_UUV_CMD_DEADBAND}, slew=${ROS2_UUV_CMD_SLEW_RATE}/s, dvl_alpha=${ROS2_UUV_DVL_LPF_ALPHA}, bar30_noise=${ROS2_UUV_BAR30_NOISE_PA_STD}Pa, timeout=${ROS2_UUV_CMD_TIMEOUT_S}s, mavlink_timeout=${ROS2_UUV_SITL_MAVLINK_TIMEOUT_S}s"
    if append_extra_arg_if_missing "--disable-thruster-perf" --disable-thruster-perf; then
        echo "[launch] SITL mode: disabling thruster performance curve for simple model."
    fi
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

if [[ -n "$PROFILE" && "$PROFILE" != "legacy" ]]; then
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

"$MJ311_MJPYTHON" run_urdf_full.py "${RUN_ARGS[@]}"
