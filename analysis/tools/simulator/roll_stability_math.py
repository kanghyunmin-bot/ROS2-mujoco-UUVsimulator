"""Math primitives shared by roll-stability probes and metrics."""

from __future__ import annotations

import math
import statistics


def quat_to_rpy_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def rms(values: list[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def stddev(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0 if values else float("nan")
    return statistics.pstdev(values)


def unwrap_rad(values: list[float]) -> list[float]:
    if not values:
        return []
    out = [values[0]]
    offset = 0.0
    prev = values[0]
    for value in values[1:]:
        delta = value - prev
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        out.append(value + offset)
        prev = value
    return out


__all__ = ["quat_to_rpy_deg", "rms", "stddev", "unwrap_rad"]
