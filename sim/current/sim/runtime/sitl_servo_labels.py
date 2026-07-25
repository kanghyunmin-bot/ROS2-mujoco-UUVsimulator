"""Display labels for SITL/plant-replay PWM servo runtime."""

from __future__ import annotations


def sitl_servo_mapping_label(raw_map: list[str], servo_signs: list[float]) -> str:
    return ", ".join(
        f"ch{idx + 1}->{thr_name}*{servo_signs[idx]:+0.0f}"
        for idx, thr_name in enumerate(raw_map)
    )


__all__ = ["sitl_servo_mapping_label"]
