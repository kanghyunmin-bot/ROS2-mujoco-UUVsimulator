"""Publisher and service-client initialization for the GUI ROS node."""

from __future__ import annotations

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

    if HAVE_MAVROS_MSGS:
        self._rc_override_pub = self.create_publisher(OverrideRCIn, self._topic("rc/override"), 3)
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

    if HAVE_STD_SRVS and self._initial_depth_hold_opt_in:
        self._initial_depth_release_client = self.create_client(
            Trigger,
            "/mujoco/release_initial_depth_hold",
        )
    else:
        self._initial_depth_release_client = None


__all__ = ["initialize_command_publishers_and_clients"]
