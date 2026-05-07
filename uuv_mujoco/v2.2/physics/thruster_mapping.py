#!/usr/bin/env python3
"""Single source of truth for UUV thruster ordering and sensor mount offsets."""

from __future__ import annotations

import sys

PHYSICAL_VERTICAL_THRUSTERS = ("ver_lf", "ver_lr", "ver_rf", "ver_rr")
PHYSICAL_YAW_THRUSTERS = ("yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr")

ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER = ("yaw_rf", "yaw_lf", "yaw_rr", "yaw_lr")
ARDUSUB_VECTORED_6DOF_VERTICAL_CHANNEL_ORDER = ("ver_rf", "ver_lf", "ver_rr", "ver_lr")
ARDUSUB_VECTORED_6DOF_SERVO_MAP = (
    ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER
    + ARDUSUB_VECTORED_6DOF_VERTICAL_CHANNEL_ORDER
)
# Physical ESC/prop convention that converts ArduSub SERVO_OUTPUT_RAW to MuJoCo
# thruster force. The real vehicle's QGC Motor Config reverse flags for motors
# 3/4/5/8 are already injected into ArduSub SITL as MOT_*_DIRECTION parameters
# by start_ardusub_sitl_mj311.sh, so they must not be applied a second time here.
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
