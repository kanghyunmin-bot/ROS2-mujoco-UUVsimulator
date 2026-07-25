"""MAVROS GUI imports with local fallback stubs."""

from __future__ import annotations

try:
    from mavros_msgs.msg import ManualControl, OverrideRCIn, RCIn, RCOut, State, StatusText
    from mavros_msgs.srv import CommandBool, SetMode, VehicleInfoGet

    HAVE_MAVROS_MSGS = True
except ModuleNotFoundError:
    HAVE_MAVROS_MSGS = False

    class OverrideRCIn:
        CHAN_RELEASE = 0
        CHAN_NOCHANGE = 65535

        def __init__(self) -> None:
            self.channels = []

    class ManualControl:
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.r = 0.0
            self.buttons = 0

    class RCOut:
        def __init__(self) -> None:
            self.channels = []

    class RCIn:
        def __init__(self) -> None:
            self.channels = []

    class State:
        connected = False
        armed = False
        guided = False
        manual_input = False
        mode = ""
        system_status = 0

    class StatusText:
        EMERGENCY = 0
        ALERT = 1
        CRITICAL = 2
        ERROR = 3
        WARNING = 4
        NOTICE = 5
        INFO = 6
        DEBUG = 7

        def __init__(self) -> None:
            self.severity = self.INFO
            self.text = ""

    class CommandBool:
        class Request:
            def __init__(self) -> None:
                self.value = False

    class SetMode:
        class Request:
            def __init__(self) -> None:
                self.base_mode = 0
                self.custom_mode = ""

    class VehicleInfoGet:
        class Request:
            def __init__(self) -> None:
                self.sysid = 1
                self.compid = 1
                self.get_all = False


__all__ = [
    "CommandBool",
    "HAVE_MAVROS_MSGS",
    "ManualControl",
    "OverrideRCIn",
    "RCIn",
    "RCOut",
    "SetMode",
    "State",
    "StatusText",
    "VehicleInfoGet",
]
