"""ROS2 probe execution helpers for roll stability sweeps."""

from __future__ import annotations

import math
from typing import Any


def validate_probe_metrics(metrics: dict[str, Any], *, stimulus: str) -> None:
    if int(metrics.get("rc_valid_samples", 0)) <= 0:
        raise RuntimeError("invalid_no_servo_output: /mavros/rc/out produced no valid 8-channel PWM samples")
    if stimulus != "neutral":
        servo_pwm = float(metrics.get("servo_delta_rms_pwm", float("nan")))
        if not math.isfinite(servo_pwm) or servo_pwm < 2.0:
            raise RuntimeError(
                "invalid_no_active_servo_output: "
                f"servo_delta_rms_pwm={servo_pwm}"
            )


def collect_probe_metrics(
    *,
    settle_s: float,
    measure_s: float,
    stimulus: str,
    hold_mode: str,
    axis_command: float,
    pulse_s_override: float | None,
) -> dict[str, Any]:
    import rclpy

    from roll_stability_probe import StabilityProbe

    rclpy.init(args=None)
    node = StabilityProbe()
    try:
        metrics = node.run_probe(
            settle_s=settle_s,
            measure_s=measure_s,
            stimulus=stimulus,
            hold_mode=hold_mode,
            axis_command=axis_command,
            pulse_s_override=pulse_s_override,
        )
        validate_probe_metrics(metrics, stimulus=stimulus)
        return metrics
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["collect_probe_metrics", "validate_probe_metrics"]
