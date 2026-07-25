"""RC-output component metrics for roll-stability sweeps."""

from __future__ import annotations

import math

from roll_stability_math import rms


def _valid_channels(channels: list[int], start: int, stop: int) -> bool:
    return len(channels) >= stop and all(900 <= channels[idx] <= 2100 for idx in range(start, stop))


def compute_rc_metrics(rc_samples: list[tuple[float, list[int]]]) -> dict[str, object]:
    vertical_rc: list[float] = []
    horizontal_rc: list[float] = []
    all_rc: list[float] = []
    roll_mix: list[float] = []
    pitch_mix: list[float] = []
    yaw_mix: list[float] = []
    rc_valid_samples = 0
    rc_min = [math.nan] * 8
    rc_max = [math.nan] * 8

    for _, channels in rc_samples:
        if _valid_channels(channels, 0, 8):
            rc_valid_samples += 1
            for idx in range(8):
                value = int(channels[idx])
                rc_min[idx] = value if math.isnan(rc_min[idx]) else min(rc_min[idx], value)
                rc_max[idx] = value if math.isnan(rc_max[idx]) else max(rc_max[idx], value)
                all_rc.append(float(value - 1500))
            horizontal_devs = [channels[idx] - 1500 for idx in range(4)]
            horizontal_rc.extend(float(value) for value in horizontal_devs)
            yaw_mix.append(float((channels[0] + channels[3]) - (channels[1] + channels[2])))

        if _valid_channels(channels, 4, 8):
            devs = [channels[idx] - 1500 for idx in range(4, 8)]
            vertical_rc.extend(float(value) for value in devs)
            roll_mix.append(float((channels[5] + channels[6]) - (channels[4] + channels[7])))
            pitch_mix.append(float((channels[6] + channels[7]) - (channels[4] + channels[5])))

    return {
        "rc_samples": len(rc_samples),
        "servo_delta_rms_pwm": rms(all_rc),
        "horizontal_rc_rms_pwm": rms(horizontal_rc),
        "vertical_rc_rms_pwm": rms(vertical_rc),
        "roll_mix_rms_pwm": rms(roll_mix),
        "pitch_mix_rms_pwm": rms(pitch_mix),
        "yaw_mix_rms_pwm": rms(yaw_mix),
        "rc_valid_samples": rc_valid_samples,
        "rc_min_ch1_8": rc_min,
        "rc_max_ch1_8": rc_max,
    }


__all__ = ["compute_rc_metrics"]
