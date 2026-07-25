"""Compatibility exports for SITL PWM plant-input handling."""

from __future__ import annotations

from .sitl_pwm_frame_handler import _handle_pwm_values


__all__ = ["_handle_pwm_values"]
