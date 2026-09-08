"""Publisher and service-client initialization for the GUI ROS node."""

from __future__ import annotations

import math
import threading

from .runtime import (
    CommandBool,
    HAVE_MAVROS_MSGS,
    HAVE_STD_SRVS,
    ManualControl,
    OverrideRCIn,
    SetMode,
    String,
    Trigger,
    VehicleInfoGet,
    os,
)


def initialize_command_publishers_and_clients(self) -> None:
    self._command_override_pub = self.create_publisher(
        String,
        "/uuv_mujoco/sitl/command_override",
        1,
    )
    yolo_detection_topic = os.environ.get("UUV_GUI_YOLO_DETECTION_TOPIC", "/uuv_mujoco/yolo_buoy_detections")
    self._yolo_buoy_detection_pub = self.create_publisher(
        String,
        yolo_detection_topic,
        1,
    )

    self._rc_override_publisher_lock = threading.RLock()
    self._rc_override_topic = self._topic("rc/override")
    self._rc_override_publisher_suspended = False
    if HAVE_MAVROS_MSGS:
        self._rc_override_pub = self.create_publisher(OverrideRCIn, self._rc_override_topic, 3)
        self._manual_control_pub = self.create_publisher(ManualControl, self._topic("manual_control/send"), 1)
        self._arm_client = self.create_client(CommandBool, self._topic("cmd/arming"))
        self._mode_client = self.create_client(SetMode, self._topic("set_mode"))
        self._vehicle_info_client = self.create_client(VehicleInfoGet, self._topic("vehicle_info_get"))
    else:
        self._rc_override_pub = None
        self._manual_control_pub = None
        self._arm_client = None
        self._mode_client = None
        self._vehicle_info_client = None
    try:
        rc_burst_count = int(float(os.environ.get("UUV_GUI_RC_OVERRIDE_BURST_COUNT", "3")))
    except (TypeError, ValueError):
        rc_burst_count = 3
    self._rc_override_burst_count = max(1, min(6, rc_burst_count))
    try:
        rc_release_delay_s = float(os.environ.get("UUV_GUI_RC_RELEASE_DELAY_S", "0.15"))
    except (TypeError, ValueError):
        rc_release_delay_s = 0.15
    if not math.isfinite(rc_release_delay_s):
        rc_release_delay_s = 0.15
    # Let the bridge forward the zero-stick frame before the MAVLink release
    # markers can replace it in the ROS latest-frame cache.
    self._rc_override_release_delay_s = max(0.05, min(0.50, rc_release_delay_s))
    self._rc_override_publish_generation = 0

    if HAVE_STD_SRVS and self._initial_depth_hold_opt_in:
        self._initial_depth_release_client = self.create_client(
            Trigger,
            "/mujoco/release_initial_depth_hold",
        )
    else:
        self._initial_depth_release_client = None

    # Terminal-launched autonomous controllers cannot call the web app's
    # in-process hand-off method.  Expose the same ownership transition over
    # ROS so the simulator GUI can stay visible without retaining a second
    # /mavros/rc/override publisher and tripping an autonomous RC mux.
    if HAVE_STD_SRVS:
        self._rc_handoff_suspend_service = self.create_service(
            Trigger,
            "/uuv_web_control_gui/suspend_rc_override",
            lambda request, response: _suspend_rc_override_service(self, request, response),
        )
        self._rc_handoff_restore_service = self.create_service(
            Trigger,
            "/uuv_web_control_gui/restore_rc_override",
            lambda request, response: _restore_rc_override_service(self, request, response),
        )
    else:
        self._rc_handoff_suspend_service = None
        self._rc_handoff_restore_service = None


def _suspend_rc_override_service(self, request, response):
    del request
    response.success = suspend_rc_override_publisher(self)
    response.message = (
        "web GUI RC publisher suspended; autonomous mux owns /mavros/rc/override"
        if response.success
        else "failed to suspend web GUI RC publisher"
    )
    return response


def _restore_rc_override_service(self, request, response):
    del request
    response.success = restore_rc_override_publisher(self)
    response.message = (
        "web GUI RC publisher restored"
        if response.success
        else "failed to restore web GUI RC publisher"
    )
    return response


def suspend_rc_override_publisher(self) -> bool:
    """Release MAVROS RC ownership, then remove the GUI output publisher.

    The standalone pinger package owns the only publisher allowed on
    ``/mavros/rc/override`` while its exclusive RC mux is running.  Publishing
    the release frame before destroying the publisher makes the hand-off
    explicit and prevents the last GUI command from being held by MAVROS.
    """
    with self._rc_override_publisher_lock:
        if self._rc_override_publisher_suspended:
            return True
        publisher = self._rc_override_pub
        if publisher is None:
            self._rc_override_publisher_suspended = True
            return True

        # This call is deliberately inside the lifecycle lock and before
        # destroy_publisher: no queued GUI RC frame may overtake the release.
        self.publish_rc_release()
        try:
            destroyed = self.destroy_publisher(publisher)
        except Exception:
            return False
        if destroyed is False:
            return False
        self._rc_override_pub = None
        self._rc_override_publisher_suspended = True
        return True


def restore_rc_override_publisher(self) -> bool:
    """Recreate the GUI MAVROS RC publisher after the pinger process exits."""
    with self._rc_override_publisher_lock:
        if self._rc_override_pub is not None:
            self._rc_override_publisher_suspended = False
            return True
        if not HAVE_MAVROS_MSGS:
            self._rc_override_publisher_suspended = False
            return True
        try:
            self._rc_override_pub = self.create_publisher(
                OverrideRCIn,
                self._rc_override_topic,
                3,
            )
        except Exception:
            return False
        self._rc_override_publisher_suspended = False
        return True


__all__ = [
    "initialize_command_publishers_and_clients",
    "restore_rc_override_publisher",
    "suspend_rc_override_publisher",
]
