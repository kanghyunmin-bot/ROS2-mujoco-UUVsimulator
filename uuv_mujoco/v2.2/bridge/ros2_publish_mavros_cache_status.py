"""MAVROS-compatible status, pressure, and battery builders."""

from __future__ import annotations

from .ros2_standard_messages import build_battery_msg, build_pressure_msg, build_vfr_hud_msg


def build_mavros_state_msg(bridge, stamp):
    return bridge._build_mavros_state(stamp)


def build_vfr_hud_status_msg(bridge, stamp, state):
    return build_vfr_hud_msg(bridge.VfrHud, stamp, state.ros_depth_m)


def build_static_pressure_msg(bridge, stamp, state):
    return build_pressure_msg(bridge.FluidPressure, stamp, state.static_pressure_pa)


def build_atm_pressure_msg(bridge, stamp):
    return build_pressure_msg(
        bridge.FluidPressure,
        stamp,
        bridge._mavros_atm_pressure_value,
    )


def build_battery_status_msg(bridge, stamp):
    return build_battery_msg(
        bridge.BatteryState,
        stamp,
        voltage=bridge._mavros_battery_voltage,
        current=bridge._mavros_battery_current,
        state_of_charge_percent=bridge._mavros_battery_soc,
    )


__all__ = [
    "build_atm_pressure_msg",
    "build_battery_status_msg",
    "build_mavros_state_msg",
    "build_static_pressure_msg",
    "build_vfr_hud_status_msg",
]
