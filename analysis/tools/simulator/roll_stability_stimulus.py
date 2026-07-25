"""Stimulus execution helpers for roll stability probes."""

from __future__ import annotations


def default_pulse_duration(*, stimulus: str, measure_s: float, pulse_s_override: float | None) -> float:
    if pulse_s_override is not None:
        return float(pulse_s_override)
    if stimulus == "heave-pulse":
        return min(0.35, max(0.05, measure_s * 0.08))
    return min(0.60, max(0.10, measure_s * 0.10))


def run_probe_stimulus(
    owner,
    *,
    stimulus: str,
    measure_s: float,
    axis_command: float,
    pulse_s_override: float | None,
) -> float:
    if stimulus == "neutral":
        owner.spin_neutral(measure_s)
        return 0.0

    if stimulus not in {"heave-pulse", "forward-pulse", "roll-pulse", "pitch-pulse", "yaw-pulse", "sway-pulse"}:
        raise RuntimeError(f"unknown stimulus: {stimulus}")

    pulse_s = default_pulse_duration(
        stimulus=stimulus,
        measure_s=measure_s,
        pulse_s_override=pulse_s_override,
    )
    spin_probe_pulse(owner, stimulus=stimulus, pulse_s=pulse_s, axis_command=axis_command)
    owner.spin_neutral(max(0.0, measure_s - pulse_s))
    return pulse_s


def spin_probe_pulse(owner, *, stimulus: str, pulse_s: float, axis_command: float) -> None:
    if stimulus == "heave-pulse":
        owner.spin_rc(pulse_s, heave=-0.25)
    elif stimulus == "forward-pulse":
        owner.spin_rc(pulse_s, forward=0.20)
    else:
        owner.spin_rc(pulse_s, **{stimulus.removesuffix("-pulse"): axis_command})


__all__ = ["default_pulse_duration", "run_probe_stimulus", "spin_probe_pulse"]
