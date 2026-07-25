"""Constants for the ArduSub VECTORED_6DOF to MuJoCo thruster contract."""

from __future__ import annotations


AXES = ("roll", "pitch", "yaw", "throttle", "forward", "lateral")

PRIMARY_WRENCH_INDEX = {
    "roll": 3,
    "pitch": 4,
    "yaw": 5,
    "throttle": 2,
    "forward": 0,
    "lateral": 1,
}

EXPECTED_PRIMARY_SIGN = {
    # This verifier audits the runtime plant-replay contract: final ArduSub
    # SERVO_OUTPUT_RAW deltas, after MOT_x_DIRECTION has already been applied,
    # are converted into this MuJoCo scene's actuator-positive convention.
    # The sign target is therefore the mounted-wrench convention used by
    # raw final PWM replay, not a second application of ArduPilot semantics.
    "roll": 1.0,
    "pitch": 1.0,
    "yaw": 1.0,
    "throttle": -1.0,
    "forward": 1.0,
    "lateral": 1.0,
}
