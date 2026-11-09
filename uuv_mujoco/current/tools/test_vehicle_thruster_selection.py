"""Regression checks for vehicle-specific thruster selection."""

from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from physics.thruster_mapping import BLUEROV2_T200_FORWARD_SIGNS, vehicle_thruster_mapping
from sim.physics.thruster_force_performance import force_from_performance_curve
from sim.runtime.simulation_step_hold_release import maybe_release_initial_depth_hold
from sim.runtime.sitl_servo_pwm import packet_commands_from_pwm
from sim.runtime.sitl_servo_state import SitlServoRuntime


def test_bluerov_pwm_mapping_selects_asymmetric_forward_and_reverse_branches():
    horizontal, vertical, channels, signs = vehicle_thruster_mapping("bluerov2_standard")
    names = list(horizontal + vertical)
    targets = packet_commands_from_pwm(
        all_thruster_names=names,
        raw_map=channels,
        servo_signs=signs,
        pwm_values=[1600] * 6 + [1900, 1100],
    )
    assert len(targets) == 6
    assert set(targets) == set(channels)
    perf = {
        "direct": True,
        "pwm": np.array([1100, 1500, 1900]),
        "force": np.array([-20, 0, 40]),
        "native_pwm_directions": dict(zip(channels, BLUEROV2_T200_FORWARD_SIGNS, strict=True)),
    }
    for name, sign in zip(channels, signs):
        force = force_from_performance_curve(
            name=name, command_shaped=targets[name], gain=1, perf_cfg=perf,
            thruster_direct_scale={},
        )
        assert force == (10 if sign > 0 else -5)


def test_accessory_pwm_does_not_release_six_motor_depth_hold():
    events = []
    bridge = SimpleNamespace(sitl_vehicle_armed=lambda: True, sitl_vehicle_mode=lambda: "MANUAL")
    options = dict(initial_depth_hold={"active": True}, auto_release=True, ros_bridge=bridge,
                   release_initial_depth_hold=events.append, active_channel_count=6)
    maybe_release_initial_depth_hold(sitl_servo_pwm_values=[1500] * 6 + [1900, 1100], **options)
    assert events == []
    maybe_release_initial_depth_hold(sitl_servo_pwm_values=[1500] * 5 + [1600, 1900, 1100], **options)
    assert events == ["auto:armed_nonneutral_servo"]


def test_dfki_wiring_selects_the_mounted_t200_curve_branch():
    _, _, channels, stock = vehicle_thruster_mapping("bluerov2_standard")
    _, _, dfki_channels, signs = vehicle_thruster_mapping("bluerov2_standard", pwm_layout="dfki")
    assert channels == dfki_channels
    assert [i for i, (a, b) in enumerate(zip(stock, signs, strict=True)) if a != b] == [4]
    targets = packet_commands_from_pwm(
        all_thruster_names=channels, raw_map=channels, servo_signs=signs,
        pwm_values=[1500] * 4 + [1900, 1900],
    )
    perf = {
        "direct": True, "pwm": np.array([1100, 1500, 1900]), "force": np.array([-40, 0, 51]),
        "native_pwm_directions": dict(zip(channels, BLUEROV2_T200_FORWARD_SIGNS, strict=True)),
    }
    assert force_from_performance_curve(
        name="ver_rf", command_shaped=targets["ver_rf"], gain=1, perf_cfg=perf, thruster_direct_scale={},
    ) == -40
    assert force_from_performance_curve(
        name="ver_lf", command_shaped=targets["ver_lf"], gain=1, perf_cfg=perf, thruster_direct_scale={},
    ) == 51


def test_ut27_mapping_keeps_eight_distinct_motors():
    horizontal, vertical, channels, signs = vehicle_thruster_mapping("ut27")
    assert len(horizontal) == len(vertical) == 4
    assert len(channels) == len(signs) == len(set(channels)) == 8


def test_sitl_packet_ignores_accessories_and_clears_disabled_motors():
    horizontal, vertical, channels, signs = vehicle_thruster_mapping("bluerov2_standard")
    runtime = SitlServoRuntime.create(
        all_thruster_names=list(horizontal + vertical), raw_map=list(channels),
        servo_signs=list(signs), sitl_servo_scale=1.0,
    )
    runtime.on_packet([1600, 1600, 1600, 1600, 1600, 1600, 1900, 1100])
    assert list(runtime.cmd_norm.values()) == [-0.25, -0.25, 0.25, 0.25, 0.25, 0.25]
    assert runtime.mapping_label().count("ch") == 6
    runtime.on_packet([0, 65535, 1500, 1500, 1500, 1500, 1900, 1100])
    assert all(value == 0 for value in runtime.cmd_norm.values())
    assert runtime.pwm_values[6:] == [1900, 1100]
