"""State initialization for the GUI ROS node."""

from __future__ import annotations

from .config import SIM_STACK_DIR
from .models import TelemetrySnapshot
from .node_stereo_camera import initialize_stereo_camera_state
from .runtime import os, threading


def initialize_node_state(self) -> None:
    self._lock = threading.Lock()
    self._snapshot = TelemetrySnapshot()
    self._last_wall = {}
    self._mode_request_in_flight = False
    self._vehicle_info_in_flight = False
    self._vehicle_info_supported = False
    self._last_graph_probe_wall = -1.0
    self._last_mode_seen = ""
    self._last_armed_seen = None
    self._latest_arm_target = None
    self._latest_mode_target = ""
    self._arm_request_in_flight = False
    self._rc_override_subscribers = 0
    self._manual_control_subscribers = 0
    self._one_shot_timers = []
    self._vehicle_connected_since_wall = -1.0
    initialize_stereo_camera_state(self)


def initialize_command_contract_state(self) -> None:
    self._control_request_timeout_s = float(os.environ.get("UUV_GUI_CONTROL_REQUEST_TIMEOUT_S", "5.0"))
    self._control_request_retry_s = min(
        max(float(os.environ.get("UUV_GUI_CONTROL_REQUEST_RETRY_S", "0.05")), 0.01),
        0.05,
    )
    self._arm_mode_command_path = os.environ.get(
        "UUV_GUI_ARM_MODE_COMMAND_PATH",
        "auto",
    ).strip().lower()
    if self._arm_mode_command_path not in {"auto", "topic", "service"}:
        self._arm_mode_command_path = "auto"
    self._require_arm_mode_settle = os.environ.get(
        "UUV_GUI_REQUIRE_ARM_MODE_EKF_SETTLE",
        "0",
    ).strip().lower() not in {"0", "false", "no", "off"}
    self._arm_mode_settle_s = float(os.environ.get("UUV_GUI_ARM_MODE_EKF_SETTLE_S", "3.0"))


def initialize_real_start_state(self) -> None:
    # Keep this path construction near the real-start contract so future changes
    # do not reintroduce implicit opt-in based on CSV existence.
    _default_real_start_csv = (
        SIM_STACK_DIR
        / "debug"
        / "controller_parity_412"
        / "real_20260401_feedback"
        / "real_controller_feedback_20hz.csv"
    )
    del _default_real_start_csv
    real_start_default = "0"
    real_start_enabled = os.environ.get(
        "UUV_REAL_START_STATE",
        real_start_default,
    ).strip().lower() not in {"0", "false", "no", "off", "disable", "disabled"}
    self._snapshot.real_start_required = bool(real_start_enabled)
    self._snapshot.real_start_ok = not bool(real_start_enabled)
    self._snapshot.real_start_status = "pending" if real_start_enabled else "not required"
    real_start_hold_default = "0"
    self._initial_depth_hold_opt_in = os.environ.get(
        "UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE",
        os.environ.get("UUV_REAL_START_STATE_HOLD_UNTIL_RELEASE", real_start_hold_default),
    ).strip().lower() not in {"0", "false", "no", "off"}
    self._initial_depth_release_pending = False
    self._initial_depth_release_in_flight = False
    self._initial_depth_release_reason = ""


__all__ = [
    "initialize_node_state",
    "initialize_command_contract_state",
    "initialize_real_start_state",
]
