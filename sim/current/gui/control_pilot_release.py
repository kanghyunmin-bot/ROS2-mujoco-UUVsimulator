"""Pilot command release helpers."""

from __future__ import annotations

from .config import GUI_PILOT_CONTROL_MODE, PILOT_CONTROL_RC_OVERRIDE


def center_rc_sticks(owner) -> None:
    owner.rc_forward_var.set(0.0)
    owner.rc_lateral_var.set(0.0)
    owner.rc_heave_var.set(0.0)
    owner.rc_yaw_var.set(0.0)
    owner._on_rc_stick_changed()


def release_rc_override(owner) -> None:
    owner.rc_override_enabled.set(False)
    owner._center_rc_sticks()
    owner._publish_rc_release_and_neutral()


def publish_rc_release_and_neutral(owner) -> None:
    owner._pilot_input_release_requested = False
    owner._rc_override_prev = False
    if GUI_PILOT_CONTROL_MODE == PILOT_CONTROL_RC_OVERRIDE:
        owner.node.publish_rc_override(yaw=0.0, heave=0.0, forward=0.0, lateral=0.0)
        owner.node.publish_rc_release()
        return
    owner.node.publish_manual_control(yaw=0.0, heave=0.0, forward=0.0, lateral=0.0)
    owner.node.publish_rc_release()


__all__ = ["center_rc_sticks", "publish_rc_release_and_neutral", "release_rc_override"]
