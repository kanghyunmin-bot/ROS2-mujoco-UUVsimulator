#!/usr/bin/env bash
set -Eeuo pipefail

# One-command competition-map SNR homing run:
#   default spawn -> A-side center -> 1.3 m RC circle -> SNR homing -> disarm
#
# The script intentionally does not start the Web GUI.  Its generic Stack Start
# uses the shallow 0.5 m vision spawn, while this test requires the validated
# 8.3 m hydrophone spawn.  MuJoCo's native viewer and follow-RViz are started.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${SCRIPT_DIR}/sim/environment.sh" ]]; then
    # Use the same desktop-local MuJoCo/ArduPilot paths as the normal GUI
    # entrypoint.  Without this, a stale ~/.venvs/uuv_mujoco from another
    # machine or notebook can silently win the launcher's fallback search.
    set +u
    # shellcheck source=/dev/null
    source "${SCRIPT_DIR}/sim/environment.sh"
    set -u
fi
ROS_WORKSPACE="${SCRIPT_DIR}/rospkg"
SIM_START="${SCRIPT_DIR}/sim/current/tools/start_snr_homing_sim.sh"
SIM_RESET="${SCRIPT_DIR}/sim/current/reset_uuv_sim.sh"
AUDIO_RELAY="${SCRIPT_DIR}/sim/current/tools/audio_stamped_relay.py"
PID_FILE="/tmp/uuv_hydrophone_competition.pids"
LOCK_FILE="/tmp/uuv_hydrophone_competition.lock"
MISSION_TIMEOUT_S=360
STOP_ONLY=0

usage() {
    cat <<'EOF'
Usage:
  ./run_hydrophone_competition.sh
  ./run_hydrophone_competition.sh --timeout SECONDS
  ./run_hydrophone_competition.sh --stop

Default run:
  1. Stops only the UUV sim/ROS/hydrophone/GUI processes from older runs.
  2. Starts the validated competition hydrophone simulator at 8.3 m depth.
  3. Starts MAVROS, PCM timestamp relay, SNR controller, MuJoCo viewer and RViz.
  4. Selects STABILIZE, arms, and publishes the start frame.
  5. Waits for center -> circle -> SNR homing -> SUCCESS, then disarms.

Press Ctrl+C to stop and clean up this run. No video is recorded.
EOF
}

while (($# > 0)); do
    case "$1" in
        --timeout)
            MISSION_TIMEOUT_S="$2"
            shift 2
            ;;
        --stop)
            STOP_ONLY=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[hydrophone] unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if ! [[ "${MISSION_TIMEOUT_S}" =~ ^[0-9]+$ ]] || ((MISSION_TIMEOUT_S < 30)); then
    echo "[hydrophone] --timeout must be an integer >= 30" >&2
    exit 2
fi
if [[ ! -x "${SIM_START}" || ! -x "${SIM_RESET}" || ! -x "${AUDIO_RELAY}" ]]; then
    echo "[hydrophone] required launcher is missing or not executable" >&2
    exit 1
fi
if [[ ! -f "${ROS_WORKSPACE}/install/setup.bash" ]]; then
    echo "[hydrophone] ROS workspace is not built: ${ROS_WORKSPACE}/install/setup.bash" >&2
    exit 1
fi
if ((STOP_ONLY == 0)); then
    if ! command -v flock >/dev/null 2>&1; then
        echo "[hydrophone] flock is required to prevent concurrent runs" >&2
        exit 1
    fi
    exec 9>"${LOCK_FILE}"
    if ! flock -n 9; then
        echo "[hydrophone] another competition hydrophone run is active" >&2
        exit 75
    fi
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROS_WORKSPACE}/install/setup.bash"
set -u

export ROS_LOCALHOST_ONLY=0
export ROS_DISABLE_DAEMON=1
export UUV_MUJOCO_SKIP_FRESHNESS_CHECK=1
export QT_AUTO_SCREEN_SCALE_FACTOR=0
export QT_SCALE_FACTOR=0.75
export QT_FONT_DPI=72

LOG_ROOT="${SCRIPT_DIR}/logs"
RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${LOG_ROOT}/hydrophone_competition_${RUN_STAMP}"

SIM_PGID=""
ROS_PGID=""
RELAY_PGID=""
HOMING_PGID=""
START_FRAME_PGID=""
RUN_ACTIVE=0
SUCCESS_REACHED=0
CLEANING=0
LAST_GROUP_PID=""

lineage_pids() {
    local lineage_pid
    lineage_pid="$$"
    while [[ -n "${lineage_pid}" && "${lineage_pid}" =~ ^[0-9]+$ &&
             "${lineage_pid}" != "0" ]]; do
        printf '%s\n' "${lineage_pid}"
        lineage_pid="$(
            ps -o ppid= -p "${lineage_pid}" 2>/dev/null |
                tr -d '[:space:]' || true
        )"
    done
}

is_own_lineage() {
    local candidate_pid="$1"
    local own_pid
    while read -r own_pid; do
        if [[ "${candidate_pid}" == "${own_pid}" ]]; then
            return 0
        fi
    done < <(lineage_pids)
    return 1
}

stop_pattern() {
    local label="$1"
    local pattern="$2"
    local candidate_pid
    local -a targets=()

    while read -r candidate_pid; do
        [[ -z "${candidate_pid}" ]] && continue
        if ! is_own_lineage "${candidate_pid}"; then
            targets+=("${candidate_pid}")
        fi
    done < <(pgrep -f -- "${pattern}" 2>/dev/null || true)

    if ((${#targets[@]} == 0)); then
        return 0
    fi

    echo "[hydrophone] stopping stale ${label}: ${targets[*]}"
    kill -INT "${targets[@]}" 2>/dev/null || true
    for _stop_wait in {1..20}; do
        local any_alive=0
        for candidate_pid in "${targets[@]}"; do
            if kill -0 "${candidate_pid}" 2>/dev/null; then
                any_alive=1
                break
            fi
        done
        ((any_alive == 0)) && return 0
        sleep 0.1
    done

    kill -TERM "${targets[@]}" 2>/dev/null || true
    for _term_wait in {1..20}; do
        local any_alive=0
        for candidate_pid in "${targets[@]}"; do
            if kill -0 "${candidate_pid}" 2>/dev/null; then
                any_alive=1
                break
            fi
        done
        ((any_alive == 0)) && return 0
        sleep 0.1
    done

    for candidate_pid in "${targets[@]}"; do
        if kill -0 "${candidate_pid}" 2>/dev/null; then
            kill -KILL "${candidate_pid}" 2>/dev/null || true
        fi
    done
}

topic_field() {
    local topic="$1"
    local field="$2"
    timeout 3 ros2 topic echo "${topic}" --once --field "${field}" 2>/dev/null |
        sed -n '1p' |
        tr -d '\r'
}

publish_emergency_stop() {
    timeout 4 ros2 topic pub --once \
        --qos-reliability reliable \
        --qos-durability transient_local \
        /mission/emergency_stop std_msgs/msg/Bool \
        "{data: true}" >/dev/null 2>&1 || true
}

disarm_vehicle() {
    if timeout 2 ros2 service type /mavros/cmd/arming >/dev/null 2>&1; then
        timeout 6 ros2 service call \
            /mavros/cmd/arming mavros_msgs/srv/CommandBool \
            "{value: false}" >/dev/null 2>&1 || true
    fi
}

publish_rc_release() {
    timeout 4 ros2 topic pub --once \
        /mavros/rc/override mavros_msgs/msg/OverrideRCIn \
        "{channels: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}" \
        >/dev/null 2>&1 || true
}

stop_stale_stack() {
    echo "[hydrophone] safely stopping the previous UUV stack"
    publish_emergency_stop
    disarm_vehicle

    stop_pattern "hydrophone ROS launch" \
        "ros2 launch hydrophone_ctrl"
    stop_pattern "hydrophone component container" \
        "component_container.*(region_local_gradient_homing_pipeline|competition_snr_homing_pipeline)"
    stop_pattern "hydrophone nodes" \
        "/(hydrophone_ctrl|audio_capture)/(region_local_gradient|waypoint_homing|sim_odometry_rebaser|audio_frequency_detector|start_frame_publisher)"
    stop_pattern "pinger audio simulator" \
        "pinger_buoy_audio_sim.py"
    stop_pattern "timestamped audio relay" \
        "sim/current/tools/audio_stamped_relay.py"
    stop_pattern "hydrophone RViz" \
        "rviz2.*region_local_gradient.rviz"
    stop_pattern "stale MAVROS service call" \
        "ros2 service call /mavros/"

    publish_rc_release

    # Stop the GUI before reset so it cannot retain or respawn a stack it owns.
    stop_pattern "Web control GUI" \
        "sim/current/gui/web_control_gui.py"

    if ! UUV_RESET_IGNORE_CONTROLLER_PARITY_LOCK=1 \
        "${SIM_RESET}" >/tmp/uuv_hydrophone_reset.log 2>&1; then
        echo "[hydrophone] simulator reset failed:" >&2
        tail -n 40 /tmp/uuv_hydrophone_reset.log >&2 || true
        return 1
    fi
    rm -f "${PID_FILE}"
    echo "[hydrophone] previous UUV stack stopped"
}

terminate_group() {
    local group_pid="$1"
    local label="$2"
    local signal_name
    local actual_pgid
    [[ -z "${group_pid}" ]] && return 0

    if ! kill -0 -- "-${group_pid}" 2>/dev/null; then
        return 0
    fi
    if kill -0 "${group_pid}" 2>/dev/null; then
        actual_pgid="$(
            ps -o pgid= -p "${group_pid}" 2>/dev/null |
                tr -d '[:space:]'
        )"
        if [[ "${actual_pgid}" != "${group_pid}" ]]; then
            echo "[hydrophone] refusing broad kill for non-leader ${label} PID ${group_pid}" >&2
            kill -TERM "${group_pid}" 2>/dev/null || true
            return 0
        fi
    fi

    echo "[hydrophone] stopping ${label} process group ${group_pid}"
    for signal_name in INT TERM KILL; do
        kill "-${signal_name}" -- "-${group_pid}" 2>/dev/null || true
        for _group_wait in {1..25}; do
            if ! kill -0 -- "-${group_pid}" 2>/dev/null; then
                return 0
            fi
            sleep 0.1
        done
    done
}

cleanup() {
    local exit_code=$?
    if ((CLEANING == 1)); then
        return
    fi
    CLEANING=1
    trap - EXIT INT TERM HUP

    if ((RUN_ACTIVE == 1)); then
        publish_emergency_stop
        disarm_vehicle
        terminate_group "${START_FRAME_PGID}" "start-frame"
        terminate_group "${HOMING_PGID}" "hydrophone/RViz"
        publish_rc_release
        terminate_group "${RELAY_PGID}" "audio relay"
        terminate_group "${ROS_PGID}" "MAVROS/ROS"
        terminate_group "${SIM_PGID}" "MuJoCo/SITL"
    fi
    rm -f "${PID_FILE}"
    if ((SUCCESS_REACHED == 0 && exit_code != 0)); then
        echo "[hydrophone] failed; logs: ${LOG_DIR}" >&2
    fi
    exit "${exit_code}"
}

start_group() {
    local label="$1"
    local log_file="$2"
    shift 2
    echo "[hydrophone] starting ${label}"
    setsid stdbuf -oL -eL "$@" >"${log_file}" 2>&1 &
    LAST_GROUP_PID=$!
    sleep 0.2
    if ! kill -0 "${LAST_GROUP_PID}" 2>/dev/null; then
        echo "[hydrophone] ${label} exited during startup; see ${log_file}" >&2
        return 1
    fi
}

wait_for_topic() {
    local topic="$1"
    local timeout_s="$2"
    local label="$3"
    local deadline_epoch
    deadline_epoch=$(( $(date +%s) + timeout_s ))
    echo "[hydrophone] waiting for ${label}: ${topic}"
    while (( $(date +%s) < deadline_epoch )); do
        if timeout 3 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
            echo "[hydrophone] ready: ${label}"
            return 0
        fi
        sleep 0.5
    done
    echo "[hydrophone] timeout waiting for ${label}: ${topic}" >&2
    return 1
}

wait_for_log_pattern() {
    local process_pid="$1"
    local log_file="$2"
    local ready_pattern="$3"
    local failure_pattern="$4"
    local timeout_s="$5"
    local label="$6"
    local deadline_epoch
    deadline_epoch=$(( $(date +%s) + timeout_s ))
    echo "[hydrophone] waiting for ${label}"
    while (( $(date +%s) < deadline_epoch )); do
        if rg -q "${ready_pattern}" "${log_file}" 2>/dev/null; then
            echo "[hydrophone] ready: ${label}"
            return 0
        fi
        if rg -q "${failure_pattern}" "${log_file}" 2>/dev/null; then
            echo "[hydrophone] ${label} reported failure" >&2
            tail -n 50 "${log_file}" >&2 || true
            return 1
        fi
        if ! kill -0 "${process_pid}" 2>/dev/null; then
            echo "[hydrophone] process exited while waiting for ${label}" >&2
            tail -n 50 "${log_file}" >&2 || true
            return 1
        fi
        sleep 0.5
    done
    echo "[hydrophone] timeout waiting for ${label}" >&2
    tail -n 50 "${log_file}" >&2 || true
    return 1
}

validate_default_spawn() {
    local odometry_sample
    local spawn_x
    local spawn_y
    odometry_sample="$(
        timeout 3 ros2 topic echo /sim/odom --once \
            --field pose.pose.position 2>/dev/null
    )"
    spawn_x="$(awk '/^x:/ {print $2}' <<<"${odometry_sample}")"
    spawn_y="$(awk '/^y:/ {print $2}' <<<"${odometry_sample}")"
    if ! [[ "${spawn_x}" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ &&
            "${spawn_y}" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]]; then
        echo "[hydrophone] could not read the default competition spawn" >&2
        return 1
    fi
    if awk -v x="${spawn_x}" -v y="${spawn_y}" \
        'BEGIN {
            dx=x-(-15.881); if (dx<0) dx=-dx;
            dy=y-1.305; if (dy<0) dy=-dy;
            exit !(dx>0.15 || dy>0.15)
        }'; then
        echo "[hydrophone] scene spawn changed: (${spawn_x}, ${spawn_y}); expected (-15.881, 1.305)" >&2
        return 1
    fi
    echo "[hydrophone] verified competition spawn: (${spawn_x}, ${spawn_y})"
}

wait_for_field_value() {
    local topic="$1"
    local field="$2"
    local expected="$3"
    local timeout_s="$4"
    local label="$5"
    local current_value
    local deadline_epoch
    deadline_epoch=$(( $(date +%s) + timeout_s ))
    while (( $(date +%s) < deadline_epoch )); do
        current_value="$(topic_field "${topic}" "${field}" || true)"
        if [[ "${current_value,,}" == "${expected,,}" ]]; then
            echo "[hydrophone] ready: ${label}=${current_value}"
            return 0
        fi
        sleep 0.5
    done
    echo "[hydrophone] timeout waiting for ${label}=${expected}" >&2
    return 1
}

wait_for_finite_snr() {
    local timeout_s="$1"
    local snr_value
    local deadline_epoch
    deadline_epoch=$(( $(date +%s) + timeout_s ))
    echo "[hydrophone] waiting for finite SNR"
    while (( $(date +%s) < deadline_epoch )); do
        snr_value="$(
            topic_field /audio_frequency_detector/snr_db_stamped data || true
        )"
        if [[ "${snr_value}" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]]; then
            echo "[hydrophone] ready: finite SNR=${snr_value} dB"
            return 0
        fi
        sleep 0.5
    done
    echo "[hydrophone] timeout waiting for finite SNR" >&2
    return 1
}

validate_rc_ownership() {
    local topic_info
    local publisher_count
    local subscriber_count
    topic_info="$(ros2 topic info /mavros/rc/override --verbose 2>/dev/null)"
    publisher_count="$(
        awk '/^Publisher count:/ {print $3}' <<<"${topic_info}"
    )"
    subscriber_count="$(
        awk '/^Subscription count:/ {print $3}' <<<"${topic_info}"
    )"
    if [[ "${publisher_count}" != "1" || "${subscriber_count}" != "1" ]] ||
       ! rg -q "Node name: waypoint_homing_controller" <<<"${topic_info}" ||
       ! rg -q "Node name: rc" <<<"${topic_info}"; then
        echo "[hydrophone] invalid RC ownership: publishers=${publisher_count:-?}, subscribers=${subscriber_count:-?}" >&2
        return 1
    fi
}

set_mode_and_arm() {
    local attempt
    local current_mode
    local armed_value

    echo "[hydrophone] selecting STABILIZE"
    for attempt in {1..5}; do
        timeout 7 ros2 service call \
            /mavros/set_mode mavros_msgs/srv/SetMode \
            "{base_mode: 0, custom_mode: 'STABILIZE'}" \
            >>"${LOG_DIR}/set_mode.log" 2>&1 || true
        current_mode="$(topic_field /mavros/state mode || true)"
        [[ "${current_mode}" == "STABILIZE" ]] && break
        sleep 1
    done
    if [[ "${current_mode:-}" != "STABILIZE" ]]; then
        echo "[hydrophone] failed to select STABILIZE" >&2
        return 1
    fi

    echo "[hydrophone] arming"
    for attempt in {1..5}; do
        timeout 7 ros2 service call \
            /mavros/cmd/arming mavros_msgs/srv/CommandBool \
            "{value: true}" \
            >>"${LOG_DIR}/arm.log" 2>&1 || true
        armed_value="$(topic_field /mavros/state armed || true)"
        [[ "${armed_value,,}" == "true" ]] && break
        sleep 1
    done
    if [[ "${armed_value,,}" != "true" ]]; then
        echo "[hydrophone] failed to arm" >&2
        return 1
    fi
    echo "[hydrophone] STABILIZE + armed"
}

monitor_mission() {
    local state_value=""
    local previous_state=""
    local success_value=""
    local snr_value=""
    local position_value=""
    local armed_value=""
    local connected_value=""
    local mode_value=""
    local mavros_state_sample=""
    local monitor_iteration=0
    local stale_snr_count=0
    local mavros_state_miss_count=0
    local armed_false_count=0
    local disconnected_count=0
    local wrong_mode_count=0
    local position_sample=""
    local position_x=""
    local position_y=""
    local mission_deadline
    mission_deadline=$(( $(date +%s) + MISSION_TIMEOUT_S ))

    echo "[hydrophone] mission gate opened"
    while (( $(date +%s) < mission_deadline )); do
        if ! kill -0 -- "-${HOMING_PGID}" 2>/dev/null; then
            echo "[hydrophone] hydrophone launch exited unexpectedly" >&2
            return 1
        fi
        if ! kill -0 -- "-${SIM_PGID}" 2>/dev/null ||
           ! kill -0 -- "-${ROS_PGID}" 2>/dev/null ||
           ! kill -0 -- "-${RELAY_PGID}" 2>/dev/null; then
            echo "[hydrophone] simulator, MAVROS, or audio relay exited" >&2
            return 1
        fi

        state_value="$(topic_field /homing/control_state data || true)"
        if [[ -n "${state_value}" && "${state_value}" != "${previous_state}" ]]; then
            echo "[hydrophone] state: ${previous_state:-START} -> ${state_value}"
            previous_state="${state_value}"
        fi

        success_value="$(topic_field /homing/success data || true)"
        if [[ "${success_value,,}" == "true" || "${state_value}" == "SUCCESS" ]]; then
            snr_value="$(
                topic_field /audio_frequency_detector/snr_db_stamped data || true
            )"
            position_value="$(
                timeout 3 ros2 topic echo /homing/sim_odom --once \
                    --field pose.pose.position 2>/dev/null |
                    awk '/^[xyz]:/ {printf "%s%s", (count++ ? ", " : ""), $0}'
            )"
            echo "[hydrophone] SUCCESS snr=${snr_value:-unknown} dB position=(${position_value})"
            return 0
        fi

        snr_value="$(
            topic_field /audio_frequency_detector/snr_db_stamped data || true
        )"
        if [[ "${snr_value}" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]]; then
            stale_snr_count=0
        else
            stale_snr_count=$((stale_snr_count + 1))
            if ((stale_snr_count >= 3)); then
                echo "[hydrophone] SNR stream became stale or non-finite" >&2
                return 1
            fi
        fi

        # Read one coherent State sample. Reading each field with a separate
        # short-lived subscription can mix a reconnect sample into an otherwise
        # healthy state and falsely tear down a moving vehicle.
        mavros_state_sample="$(
            timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null || true
        )"
        armed_value="$(awk '/^armed:/ {print $2; exit}' <<<"${mavros_state_sample}")"
        connected_value="$(
            awk '/^connected:/ {print $2; exit}' <<<"${mavros_state_sample}"
        )"
        mode_value="$(awk '/^mode:/ {print $2; exit}' <<<"${mavros_state_sample}")"
        if [[ -z "${armed_value}" || -z "${connected_value}" ||
              -z "${mode_value}" ]]; then
            mavros_state_miss_count=$((mavros_state_miss_count + 1))
            if ((mavros_state_miss_count >= 3)); then
                echo "[hydrophone] MAVROS state stream became stale" >&2
                return 1
            fi
        else
            mavros_state_miss_count=0
        fi
        if [[ -n "${armed_value}" && "${armed_value,,}" != "true" ]]; then
            armed_false_count=$((armed_false_count + 1))
            echo "[hydrophone] warning: unarmed sample ${armed_false_count}/3"
            if ((armed_false_count >= 3)); then
                echo "[hydrophone] vehicle remained disarmed before SUCCESS" >&2
                return 1
            fi
        elif [[ "${armed_value,,}" == "true" ]]; then
            armed_false_count=0
        fi
        if [[ -n "${connected_value}" && "${connected_value,,}" != "true" ]]; then
            disconnected_count=$((disconnected_count + 1))
            if ((disconnected_count >= 3)); then
                echo "[hydrophone] MAVROS remained disconnected during mission" >&2
                return 1
            fi
        elif [[ "${connected_value,,}" == "true" ]]; then
            disconnected_count=0
        fi
        if [[ -n "${mode_value}" && "${mode_value}" != "STABILIZE" ]]; then
            wrong_mode_count=$((wrong_mode_count + 1))
            if ((wrong_mode_count >= 3)); then
                echo "[hydrophone] mode remained ${mode_value} during mission" >&2
                return 1
            fi
        elif [[ "${mode_value}" == "STABILIZE" ]]; then
            wrong_mode_count=0
        fi

        position_sample="$(
            timeout 3 ros2 topic echo /homing/sim_odom --once \
                --field pose.pose.position 2>/dev/null || true
        )"
        position_x="$(awk '/^x:/ {print $2}' <<<"${position_sample}")"
        position_y="$(awk '/^y:/ {print $2}' <<<"${position_sample}")"
        if [[ "${position_x}" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ &&
              "${position_y}" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]] &&
           awk -v x="${position_x}" -v y="${position_y}" \
               'BEGIN {exit !(x < -1.169 || x > 32.931 || y < -15.855 || y > 13.245)}'; then
            echo "[hydrophone] safety-boundary violation: x=${position_x}, y=${position_y}" >&2
            return 1
        fi
        monitor_iteration=$((monitor_iteration + 1))
        if ((monitor_iteration % 5 == 0)); then
            validate_rc_ownership || return 1
        fi
        sleep 2
    done

    echo "[hydrophone] mission timeout after ${MISSION_TIMEOUT_S}s" >&2
    return 1
}

stop_stale_stack
if ((STOP_ONLY == 1)); then
    exit 0
fi

mkdir -p "${LOG_DIR}"
RUN_ACTIVE=1
trap cleanup EXIT INT TERM HUP

start_group "competition MuJoCo/SITL viewer" \
    "${LOG_DIR}/sim.log" \
    "${SIM_START}" --lightweight
SIM_PGID="${LAST_GROUP_PID}"
printf 'sim=%s\n' "${SIM_PGID}" >"${PID_FILE}"

wait_for_log_pattern "${SIM_PGID}" "${LOG_DIR}/sim.log" \
    "strict compatibility transport READY" \
    "strict compatibility readiness failed" \
    105 "strict SITL transport"
wait_for_topic /clock 90 "simulation clock"
wait_for_topic /sim/odom 90 "ground-truth odometry"
wait_for_topic /audio 90 "MuJoCo hydrophone PCM"
validate_default_spawn

start_group "MAVROS/real ROS package stack" \
    "${LOG_DIR}/ros.log" \
    ros2 launch auv rov_start.launch.py \
        fcu_url:=udp://0.0.0.0:14551@ \
        use_sim_time:=true \
        use_dvl:=false \
        use_joy2mavros:=false \
        use_battery_bridge:=false \
        use_odom2mavros:=false \
        use_guided_navigation:=false \
        publish_static_tf:=false \
        use_localization:=true \
        use_ekf:=true \
        surface_pressure_pa:=101640.0 \
        depth_zero_at_start:=false \
        depth_offset_m:=-0.0536 \
        use_web_gui:=false \
        use_rviz:=false \
        use_mission_rviz_visualizer:=false
ROS_PGID="${LAST_GROUP_PID}"
printf 'ros=%s\n' "${ROS_PGID}" >>"${PID_FILE}"

wait_for_topic /mavros/state 90 "MAVROS state"
wait_for_field_value /mavros/state connected true 90 "MAVROS connected"

start_group "PCM acquisition timestamp relay" \
    "${LOG_DIR}/audio_relay.log" \
    python3 "${AUDIO_RELAY}" --ros-args -p use_sim_time:=true
RELAY_PGID="${LAST_GROUP_PID}"
printf 'relay=%s\n' "${RELAY_PGID}" >>"${PID_FILE}"
wait_for_topic /audio_stamped 30 "timestamped hydrophone PCM"
AUDIO_STAMPED_PUBLISHER_COUNT="$(
    ros2 topic info /audio_stamped 2>/dev/null |
        awk '/Publisher count:/ {print $3}'
)"
if [[ "${AUDIO_STAMPED_PUBLISHER_COUNT}" != "1" ]]; then
    echo "[hydrophone] expected one /audio_stamped publisher, found ${AUDIO_STAMPED_PUBLISHER_COUNT:-unknown}" >&2
    exit 1
fi

start_group "competition SNR homing and follow-RViz" \
    "${LOG_DIR}/homing.log" \
    ros2 launch hydrophone_ctrl competition_snr_homing.launch.py \
        use_sim_time:=true \
        use_rviz:=true \
        start_immediately:=false
HOMING_PGID="${LAST_GROUP_PID}"
printf 'homing=%s\n' "${HOMING_PGID}" >>"${PID_FILE}"

wait_for_topic /homing/sim_odom 30 "rebased mission odometry"
wait_for_finite_snr 30
wait_for_field_value /homing/control_state data MOVE_TO_SCAN_CENTER 30 \
    "controller state"

validate_rc_ownership

set_mode_and_arm
sleep 1
wait_for_field_value /mavros/state connected true 10 "MAVROS connected"
wait_for_field_value /mavros/state mode STABILIZE 10 "flight mode"
wait_for_field_value /mavros/state armed true 10 "armed"
validate_rc_ownership

start_group "mission start-frame gate" \
    "${LOG_DIR}/start_frame.log" \
    ros2 run hydrophone_ctrl start_frame_publisher --ros-args \
        -p use_sim_time:=true \
        -p odometry_topic:=/homing/sim_odom \
        -p start_frame_topic:=/start_frame \
        -p frame_id:=odom \
        -p publish_rate_hz:=1.0
START_FRAME_PGID="${LAST_GROUP_PID}"
printf 'start_frame=%s\n' "${START_FRAME_PGID}" >>"${PID_FILE}"
wait_for_topic /start_frame 15 "mission start frame"
wait_for_topic /homing/rviz/markers 15 "RViz marker stream"
if ! pgrep -g "${HOMING_PGID}" -f "/rviz2/rviz2" >/dev/null 2>&1; then
    echo "[hydrophone] RViz process is not alive" >&2
    exit 1
fi

monitor_mission
SUCCESS_REACHED=1
disarm_vehicle
wait_for_field_value /mavros/state armed false 15 "armed"

echo "[hydrophone] completed successfully; vehicle is disarmed"
echo "[hydrophone] MuJoCo viewer and follow-RViz remain open"
echo "[hydrophone] logs: ${LOG_DIR}"
echo "[hydrophone] press Ctrl+C to close this stack"

while kill -0 -- "-${SIM_PGID}" 2>/dev/null &&
      kill -0 -- "-${ROS_PGID}" 2>/dev/null &&
      kill -0 -- "-${HOMING_PGID}" 2>/dev/null; do
    sleep 5
done

echo "[hydrophone] a required process exited" >&2
exit 1
