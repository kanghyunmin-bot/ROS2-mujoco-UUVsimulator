"""Probe execution sequence for roll stability diagnostics."""

from __future__ import annotations

from typing import Any

from roll_stability_metrics import compute_metrics
from roll_stability_stimulus import run_probe_stimulus


class RollStabilitySequenceMixin:
    def run_probe(
        self,
        settle_s: float,
        measure_s: float,
        stimulus: str,
        hold_mode: str,
        axis_command: float,
        pulse_s_override: float | None,
    ) -> dict[str, Any]:
        self.wait_for_stack()
        self.spin_neutral(1.5)
        self.set_mode("MANUAL")
        self.spin_neutral(1.0)
        self.arm(True)
        self.spin_neutral(3.0)
        if hold_mode != "MANUAL":
            self.set_mode(hold_mode)
        self.spin_neutral(settle_s)

        self.samples.clear()
        self.depth_samples.clear()
        self.rc_samples.clear()
        self.sample_enabled = True
        pulse_s_used = run_probe_stimulus(
            self,
            stimulus=stimulus,
            measure_s=measure_s,
            axis_command=axis_command,
            pulse_s_override=pulse_s_override,
        )
        self.sample_enabled = False
        self.arm(False)
        self.spin_neutral(0.5)
        return compute_metrics(self.samples, self.depth_samples, self.rc_samples, measure_s, pulse_s_used)


__all__ = ["RollStabilitySequenceMixin"]
