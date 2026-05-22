#!/usr/bin/env python3
"""Single source of truth for UUV thruster ordering and sensor mount offsets."""

from __future__ import annotations

import sys

PHYSICAL_VERTICAL_THRUSTERS = ("ver_lf", "ver_lr", "ver_rf", "ver_rr")
PHYSICAL_YAW_THRUSTERS = ("yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr")

# ArduSub AP_Motors6DOF.cpp, SUB_FRAME_VECTORED_6DOF.
# Columns are roll, pitch, yaw, throttle, forward, lateral in ArduPilot's FRD
# body frame. Keep this next to the servo map so version drift is visible.
ARDUSUB_VECTORED_6DOF_MOTOR_FACTORS_FRD = (
    (0.0, 0.0, 1.0, 0.0, -1.0, 1.0),
    (0.0, 0.0, -1.0, 0.0, -1.0, -1.0),
    (0.0, 0.0, -1.0, 0.0, 1.0, 1.0),
    (0.0, 0.0, 1.0, 0.0, 1.0, -1.0),
    (1.0, -1.0, 0.0, -1.0, 0.0, 0.0),
    (-1.0, -1.0, 0.0, -1.0, 0.0, 0.0),
    (1.0, 1.0, 0.0, -1.0, 0.0, 0.0),
    (-1.0, 1.0, 0.0, -1.0, 0.0, 0.0),
)

# real_robot.param MOT_1_DIRECTION .. MOT_8_DIRECTION. ArduSub applies these
# before generating final JSON/SERVO PWM, so the simulator must not multiply
# them again when converting final PWM delta into MuJoCo actuator-positive force.
# Keep this tuple for contract verification and documentation only.
REAL_ROBOT_MOT_DIRECTIONS = (1, 1, -1, -1, -1, 1, 1, -1)

# ArduSub VECTORED_6DOF motor order is defined in AP_Motors6DOF.cpp:
#   1..4: horizontal yaw/forward/lateral motors
#   5..8: vertical roll/pitch/throttle motors
# The signs below convert the final SERVO_OUTPUT_RAW PWM delta, after the real
# MOT_x_DIRECTION parameters have already been applied inside ArduSub, into this
# MuJoCo model's actuator-positive force direction. ArduSub's body convention is
# FRD, while the MuJoCo body is FLU, so yaw/pitch/lateral signs must be compared
# after converting the model wrench into FRD axes.
ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER = ("yaw_rf", "yaw_lf", "yaw_rr", "yaw_lr")
# BlueROV/ArduSub VECTORED_6DOF SERVO5..8 physical vertical motor order.
ARDUSUB_VECTORED_6DOF_VERTICAL_CHANNEL_ORDER = ("ver_rf", "ver_lf", "ver_rr", "ver_lr")
ARDUSUB_VECTORED_6DOF_SERVO_MAP = (
    ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER
    + ARDUSUB_VECTORED_6DOF_VERTICAL_CHANNEL_ORDER
)
# These signs make a positive ArduSub roll, pitch, throttle, forward, lateral,
# and yaw command produce the same primary FRD wrench direction in MuJoCo after
# ArduSub's real MOT_x_DIRECTION parameters have already shaped SERVO_OUTPUT_RAW.
ARDUSUB_VECTORED_6DOF_SERVO_SIGNS = (-1, -1, 1, 1, -1, 1, 1, -1)

SENSOR_SITES_FLU = {
    "imu_site": (0.0, 0.0, 0.0),
    "bar30_site": (0.0, 0.0, -0.0600),
    "dvl_site": (0.0, 0.0, -0.1000),
}


def flu_pos_to_frd(pos_flu: tuple[float, float, float]) -> tuple[float, float, float]:
    x_fwd, y_left, z_up = pos_flu
    return (x_fwd, -y_left, -z_up)


RNGFINDER_POS_FRD = flu_pos_to_frd(SENSOR_SITES_FLU["dvl_site"])


def mapping_value(name: str) -> str:
    if name == "rngfnd1-pos-x":
        return f"{RNGFINDER_POS_FRD[0]:.4f}"
    if name == "rngfnd1-pos-y":
        return f"{RNGFINDER_POS_FRD[1]:.4f}"
    if name == "rngfnd1-pos-z":
        return f"{RNGFINDER_POS_FRD[2]:.4f}"
    raise KeyError(f"unknown mapping key: {name}")


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) != 1:
        print(
            "usage: thruster_mapping.py {rngfnd1-pos-x|rngfnd1-pos-y|rngfnd1-pos-z}",
            file=sys.stderr,
        )
        return 2
    try:
        print(mapping_value(args[0]))
    except KeyError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
