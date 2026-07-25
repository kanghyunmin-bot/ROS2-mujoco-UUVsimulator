"""Rule helper facade for axis RC health checks."""

from __future__ import annotations

from axis_rc_health_flags import apply_failure_flag, apply_warning_flag
from axis_rc_health_phase_rules import check_armed_fraction, check_rcout_movement, check_sample_count
from axis_rc_health_response_rules import check_neutral_residuals, check_primary_axis_response


__all__ = [
    "check_armed_fraction",
    "check_neutral_residuals",
    "check_primary_axis_response",
    "check_rcout_movement",
    "check_sample_count",
]
