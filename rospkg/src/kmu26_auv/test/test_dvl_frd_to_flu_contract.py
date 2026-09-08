#!/usr/bin/env python3
"""Guard the physical A50 FRD frame against a double sign conversion."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "dvl_to_twist_bridge.cpp").read_text()
LAUNCH = (ROOT / "launch" / "rov_start.launch.py").read_text()


def require(fragment: str, text: str, label: str) -> None:
    if fragment not in text:
        raise AssertionError(f"missing {label}: {fragment}")


def main() -> int:
    require(
        'declare_parameter<std::string>("output_frame_id", "dvl_link")',
        SOURCE,
        "native A50 output frame",
    )
    require("out.twist.twist.linear.y = msg->velocity.y", SOURCE, "native FRD y")
    require("out.twist.twist.linear.z = msg->velocity.z", SOURCE, "native FRD z")
    require('DeclareLaunchArgument("dvl_roll", default_value="3.141592653589793")',
            LAUNCH, "single FLU/FRD static rotation")
    if "input_velocity_is_frd" in SOURCE or "input_velocity_is_frd" in LAUNCH:
        raise AssertionError("manual FRD/FLU sign conversion must not coexist with the X-pi TF")
    if "convert_covariance_frd_to_flu" in SOURCE:
        raise AssertionError("covariance must remain expressed in dvl_link")
    print("PASS: A50 FRD remains in dvl_link and TF performs exactly one conversion")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
