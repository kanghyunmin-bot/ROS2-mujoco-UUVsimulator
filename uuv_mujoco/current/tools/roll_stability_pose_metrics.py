"""Pose/depth/IMU metric extraction for roll-stability sweeps."""

from __future__ import annotations

import math
import statistics
from typing import Any

from roll_stability_math import rms, stddev, unwrap_rad


def finite_sample_values(samples: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for sample in samples:
        value = float(sample[key])
        if math.isfinite(value):
            values.append(value)
    return values


def compute_pose_metrics(
    samples: list[dict[str, Any]],
    depth_samples: list[tuple[float, float]],
    pulse_s: float,
) -> dict[str, Any]:
    roll = [float(sample["roll_deg"]) for sample in samples]
    pitch = [float(sample["pitch_deg"]) for sample in samples]
    yaw = [float(sample["yaw_deg"]) for sample in samples]
    z = [float(sample["z_m"]) for sample in samples]
    depths = [float(value) for _, value in depth_samples if math.isfinite(float(value))]
    gyro_x = finite_sample_values(samples, "gyro_x")
    gyro_y = finite_sample_values(samples, "gyro_y")
    gyro_z = finite_sample_values(samples, "gyro_z")

    depth_drift = float(depths[-1] - depths[0]) if len(depths) >= 2 else float("nan")
    z_drift = float(z[-1] - z[0]) if len(z) >= 2 else float("nan")
    yaw_delta = 0.0
    if len(yaw) >= 2:
        yaw_unwrapped = [math.degrees(value) for value in unwrap_rad([math.radians(value) for value in yaw])]
        yaw_delta = float(yaw_unwrapped[-1] - yaw_unwrapped[0])

    sample_t0 = float(samples[0]["t"])
    post_samples = [sample for sample in samples if float(sample["t"]) - sample_t0 >= pulse_s + 0.25]
    post_gyro_z = finite_sample_values(post_samples, "gyro_z")
    post_roll = [float(sample["roll_deg"]) for sample in post_samples]
    post_pitch = [float(sample["pitch_deg"]) for sample in post_samples]

    return {
        "pose_samples": len(samples),
        "depth_samples": len(depths),
        "roll_rms_deg": rms(roll),
        "roll_peak_deg": max(abs(value) for value in roll),
        "roll_mean_deg": statistics.fmean(roll),
        "pitch_rms_deg": rms(pitch),
        "pitch_peak_deg": max(abs(value) for value in pitch),
        "gyro_x_rms_rad_s": rms(gyro_x),
        "gyro_y_rms_rad_s": rms(gyro_y),
        "gyro_z_rms_rad_s": rms(gyro_z),
        "yaw_delta_deg": yaw_delta,
        "yaw_peak_rate_rad_s": max((abs(value) for value in gyro_z), default=float("nan")),
        "post_gyro_z_rms_rad_s": rms(post_gyro_z),
        "post_roll_rms_deg": rms(post_roll),
        "post_pitch_rms_deg": rms(post_pitch),
        "depth_mean_m": statistics.fmean(depths) if depths else float("nan"),
        "depth_std_m": stddev(depths),
        "depth_drift_m": depth_drift,
        "z_drift_m": z_drift,
    }


__all__ = ["compute_pose_metrics", "finite_sample_values"]
