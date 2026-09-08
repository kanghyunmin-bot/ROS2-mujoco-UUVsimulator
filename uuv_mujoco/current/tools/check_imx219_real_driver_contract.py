#!/usr/bin/env python3
"""Validate the pinned physical IMX219 driver and capture-time overlay."""

from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[3]
DRIVER = ROOT / "rospkg" / "src" / "auv_imx219_camera"
EXPECTED_COMMIT = "a6b5b9455f082c201496326bfdaee2791f4a4d90"


def main() -> int:
    source = (DRIVER / "src" / "imx219_camera_node.cpp").read_text(encoding="utf-8")
    launch = (DRIVER / "launch" / "dual_imx219.launch.py").read_text(encoding="utf-8")
    package = (DRIVER / "package.xml").read_text(encoding="utf-8")
    commit = subprocess.run(
        ["git", "-C", str(DRIVER), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    assert commit == EXPECTED_COMMIT

    required_source = (
        'declare_parameter<std::string>("timestamp_source", "gstreamer_pts")',
        'declare_parameter<int>("max_capture_age_ms", 2000)',
        "GST_BUFFER_PTS(buffer)",
        "gst_sample_get_segment(sample)",
        "gst_segment_to_running_time_full(",
        "gst_element_get_current_running_time(pipeline_)",
        "image.header.stamp = capture_stamp(sample, buffer);",
        "sensor_msgs::image_encodings::BGR8",
        "camera_info_.header = image.header;",
    )
    for token in required_source:
        assert token in source, f"physical IMX219 driver missing: {token}"
    assert "image.header.stamp = now();" not in source

    required_launch = (
        'DeclareLaunchArgument("width", default_value="1280")',
        'DeclareLaunchArgument("height", default_value="720")',
        'DeclareLaunchArgument("framerate", default_value="30")',
        'DeclareLaunchArgument("timestamp_source", default_value="gstreamer_pts")',
        'default_value="imx219/camera0"',
        'default_value="imx219/camera1"',
    )
    for token in required_launch:
        assert token in launch, f"physical IMX219 launch missing: {token}"

    assert "<depend>image_transport</depend>" in package
    assert "<exec_depend>compressed_image_transport</exec_depend>" in package
    print(f"imx219_real_driver_contract=PASS base_commit={commit}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
