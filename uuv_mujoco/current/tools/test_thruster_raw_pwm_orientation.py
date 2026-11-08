"""Check that scene-axis conversion does not mirror a measured ESC pulse."""

from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from physics.thruster_mapping import (  # noqa: E402
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
)
from sim.physics.thruster_force_performance import force_from_performance_curve  # noqa: E402
from sim.runtime.sitl_servo_pwm import packet_commands_from_pwm  # noqa: E402


@pytest.mark.parametrize("channel", range(8))
@pytest.mark.parametrize("pwm", [1300, 1400, 1500, 1600, 1700])
def test_raw_esc_curve_precedes_scene_axis_conversion(channel: int, pwm: int) -> None:
    # An intentionally asymmetric measured curve exposes sign/lookup order.
    # The old symmetric-curve axis checks cannot distinguish the two orders.
    curve = {"direct": True, "pwm": np.array([1100., 1500., 1900.]),
             "force": np.array([-20., 0., 40.])}
    name = ARDUSUB_VECTORED_6DOF_SERVO_MAP[channel]
    direction = ARDUSUB_VECTORED_6DOF_SERVO_SIGNS[channel]
    pulses = [1500] * 8
    pulses[channel] = pwm
    commands = packet_commands_from_pwm(
        all_thruster_names=ARDUSUB_VECTORED_6DOF_SERVO_MAP,
        raw_map=ARDUSUB_VECTORED_6DOF_SERVO_MAP,
        servo_signs=ARDUSUB_VECTORED_6DOF_SERVO_SIGNS, pwm_values=pulses,
    )
    actual = force_from_performance_curve(
        name=name, command_shaped=commands[name], gain=3., perf_cfg=curve,
        thruster_direct_scale={name: .7},
    )
    native_force = float(np.interp(pwm, curve["pwm"], curve["force"]))
    assert actual == pytest.approx(direction * native_force * .7)


def test_non_direct_curve_retains_existing_command_convention() -> None:
    actual = force_from_performance_curve(
        name="ver_rf", command_shaped=.5, gain=3.,
        perf_cfg={"direct": False, "pwm": [1100., 1500., 1900.], "force": [-20., 0., 40.]},
        thruster_direct_scale={"ver_rf": .7},
    )
    assert actual == pytest.approx(60.)
