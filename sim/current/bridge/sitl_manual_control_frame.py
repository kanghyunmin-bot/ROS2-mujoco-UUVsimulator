"""MANUAL_CONTROL frame construction and priming helpers."""

from __future__ import annotations

from dataclasses import dataclass

from sim.transport import manual_axes_are_near_neutral, manual_axis_to_int, manual_thrust_to_int


@dataclass(frozen=True)
class ManualControlFrame:
    x: int
    y: int
    z: int
    r: int
    buttons: int


def build_manual_control_frame(*, x: float, y: float, z: float, r: float, buttons: int) -> ManualControlFrame:
    return ManualControlFrame(
        x=manual_axis_to_int(x),
        y=manual_axis_to_int(y),
        z=manual_thrust_to_int(z),
        r=manual_axis_to_int(r),
        buttons=int(buttons) & 0xFFFF,
    )


def send_manual_control_frame(mav, *, target_sys: int, frame: ManualControlFrame) -> None:
    mav.mav.manual_control_send(
        int(target_sys),
        int(frame.x),
        int(frame.y),
        int(frame.z),
        int(frame.r),
        int(frame.buttons),
    )


def prime_manual_control_if_needed(owner, mav, *, target_sys: int, frame: ManualControlFrame) -> None:
    if owner._sitl_manual_control_primed:
        return
    if manual_axes_are_near_neutral(x=frame.x, y=frame.y, z=frame.z, r=frame.r):
        owner._sitl_manual_control_primed = True
        return
    # ArduSub's joystick path ignores held non-neutral input until it has first
    # seen all axes near neutral.
    mav.mav.manual_control_send(
        int(target_sys),
        0,
        0,
        500,
        0,
        int(frame.buttons),
    )
    owner._sitl_manual_control_primed = True


__all__ = [
    "ManualControlFrame",
    "build_manual_control_frame",
    "prime_manual_control_if_needed",
    "send_manual_control_frame",
]
