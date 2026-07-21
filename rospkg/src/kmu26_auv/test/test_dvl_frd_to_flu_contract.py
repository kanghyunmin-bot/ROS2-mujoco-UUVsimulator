#!/usr/bin/env python3
"""Guard the physical A50 FRD -> ROS FLU localization boundary."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "dvl_to_twist_bridge.cpp").read_text()
LAUNCH = (ROOT / "launch" / "rov_start.launch.py").read_text()


def require(fragment: str, text: str, label: str) -> None:
    if fragment not in text:
        raise AssertionError(f"missing {label}: {fragment}")


def main() -> int:
    require(
        'declare_parameter<bool>("input_velocity_is_frd", true)',
        SOURCE,
        "real-safe default",
    )
    require(
        "input_velocity_is_frd_ ? -msg->velocity.y : msg->velocity.y",
        SOURCE,
        "FRD right to FLU left conversion",
    )
    require(
        "input_velocity_is_frd_ ? -msg->velocity.z : msg->velocity.z",
        SOURCE,
        "FRD down to FLU up conversion",
    )
    require("convert_covariance_frd_to_flu(cov)", SOURCE, "covariance conversion")
    require('"dvl_input_velocity_is_frd"', LAUNCH, "launch argument")
    require(
        '"input_velocity_is_frd": ParameterValue(',
        LAUNCH,
        "launch forwarding",
    )
    print("PASS: physical A50 FRD velocity is converted once at the ROS localization boundary")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
