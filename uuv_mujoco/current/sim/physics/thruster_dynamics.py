"""First-order actuator prior with continuous deceleration through reversal.

The state is effective normalized drive, not measured RPM. Time constants need
vehicle step-response identification; static T200 curves do not identify them.
"""

import math

from physics.hydrodynamics_math import first_order_response


def thruster_response(current: float, target: float, dt: float, tau_up: float, tau_down: float) -> float:
    if not all(math.isfinite(x) for x in (current, target, dt, tau_up, tau_down)):
        raise ValueError("thruster dynamics require finite values")
    if dt <= 0:
        return current
    if current * target >= 0:
        return first_order_response(current, target, dt, tau_up, tau_down)
    # Opposite command first brakes the existing rotation. Use the exact zero
    # crossing time so splitting a step does not change the reversal response.
    tau_down = max(tau_down, 1e-6)
    crossing_s = tau_down * math.log1p(abs(current / target))
    if dt <= crossing_s:
        return target + (current - target) * math.exp(-dt / tau_down)
    return first_order_response(0.0, target, dt - crossing_s, tau_up, tau_down)
