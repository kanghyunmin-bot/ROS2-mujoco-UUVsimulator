#!/usr/bin/env python3
"""Guard the opt-in measured physical IMX219 static-TF boundary."""

from __future__ import annotations

import json
from pathlib import Path
import re


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = Path(__file__).resolve().parents[4]
LAUNCH_PATH = PACKAGE / "launch" / "rov_start.launch.py"
CONTRACT_PATH = (
    REPOSITORY / "uuv_mujoco" / "current" / "config" / "real_sim_sensor_contract.json"
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def require_static_tf_call(source: str, camera_index: int) -> None:
    prefix = f"imx219_camera{camera_index}"
    call = re.search(
        rf'_static_tf_node\(\s*"{prefix}_static_tf",(?P<body>.*?)\n\s*\),',
        source,
        flags=re.DOTALL,
    )
    require(call is not None, f"{prefix} static TF node is missing")
    body = call.group("body")
    expected = (
        "base_frame",
        f"{prefix}_frame",
        f"{prefix}_x",
        f"{prefix}_y",
        f"{prefix}_z",
        f"{prefix}_roll",
        f"{prefix}_pitch",
        f"{prefix}_yaw",
        "condition=imx219_static_tf_enabled",
    )
    for fragment in expected:
        require(fragment in body, f"{prefix} static TF does not use {fragment}")


def main() -> int:
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))

    declaration = re.search(
        r'DeclareLaunchArgument\(\s*"publish_imx219_static_tf",(?P<body>.*?)\n\s*\),',
        source,
        flags=re.DOTALL,
    )
    require(declaration is not None, "publish_imx219_static_tf argument is missing")
    require(
        'default_value="false"' in declaration.group("body"),
        "physical IMX219 static TF must remain disabled by default",
    )

    condition = source.split("imx219_static_tf_enabled =", maxsplit=1)[1].split(
        "imx219_static_tf_warning =", maxsplit=1
    )[0]
    for fragment in (
        "use_imx219",
        "imx219_launch_file",
        "use_sim_time",
        "publish_static_tf",
        "publish_imx219_static_tf",
        "not in ('true', '1', 'yes')",
    ):
        require(fragment in condition, f"camera TF guard is missing {fragment}")

    require(
        "Localization/SLAM must not consume these images" in source,
        "camera-without-TF launch warning is missing",
    )
    for camera_index in (0, 1):
        calibration_argument = f"camera{camera_index}_calibration_file"
        require(
            f'"{calibration_argument}",' in source,
            f"{calibration_argument} launch argument is missing",
        )
        require(
            f'"{calibration_argument}": {calibration_argument}' in source,
            f"{calibration_argument} is not forwarded to dual_imx219.launch.py",
        )
    for camera_index in (0, 1):
        prefix = f"imx219_camera{camera_index}"
        require(
            f'"camera{camera_index}_frame_id": {prefix}_frame' in source,
            f"physical camera{camera_index} image frame is not tied to its TF frame",
        )
        for suffix in ("x", "y", "z", "roll", "pitch", "yaw"):
            require(
                f'DeclareLaunchArgument("{prefix}_{suffix}", default_value="0.0")'
                in source,
                f"{prefix}_{suffix} launch argument is missing",
            )
        require_static_tf_call(source, camera_index)

    transforms = {
        (item["parent"], item["child"]): item
        for item in contract["frames"]["static_transforms"]
    }
    for camera_index in (0, 1):
        prefix = f"imx219_camera{camera_index}"
        key = ("base_link", f"{prefix}_optical_frame")
        require(key in transforms, f"sensor contract is missing {key}")
        transform = transforms[key]
        require(
            transform["translation_m"] is None and transform["rotation_rpy_rad"] is None,
            f"{prefix} contract must not invent an unmeasured extrinsic",
        )
        require(
            transform["status"] == "requires_measured_physical_extrinsic",
            f"{prefix} calibration status is not explicit",
        )
        require(
            transform["enabled_by_default"] is False,
            f"{prefix} physical TF must be opt-in",
        )
        require(
            transform["launch_arguments"]
            == [f"{prefix}_{suffix}" for suffix in ("x", "y", "z", "roll", "pitch", "yaw")],
            f"{prefix} contract launch arguments do not match the launch file",
        )

    imx219 = contract["device_boundaries"]["imx219"]
    require(
        imx219["physical_calibration_launch_arguments"]
        == ["camera0_calibration_file", "camera1_calibration_file"],
        "physical IMX219 calibration launch arguments are not contractual",
    )
    require(
        imx219["physical_calibration_policy"]
        == "supply_measured_in_water_camera_info_yaml_before_localization_or_slam_use",
        "physical IMX219 calibration policy is missing",
    )

    print("imx219_static_tf_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
