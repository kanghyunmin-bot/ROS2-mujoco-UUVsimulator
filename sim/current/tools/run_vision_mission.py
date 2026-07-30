#!/usr/bin/env python3
"""Run the real vision package with simulator-only tuning and an oracle gate."""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String


class MissionMonitor(Node):
    def __init__(self, status_path: Path) -> None:
        super().__init__("sim_vision_mission_monitor")
        self.status_path = status_path
        self.started_wall = time.monotonic()
        self.state = "WAITING"
        self.state_history: list[str] = []
        self.bbox_messages = 0
        self.detection_count = 0
        self.class_detection_count: dict[str, int] = {}
        self.class_max_area_ratio: dict[str, float] = {}
        self.last_detections: dict[str, dict[str, float]] = {}
        self.last_detection_wall = 0.0
        self.prescan_elapsed_s = 0.0
        self.prescan_selected_area_ratio = 0.0
        self.initial_physical_success: set[str] | None = None
        self.physical_success: set[str] = set()
        self.enable_pub = self.create_publisher(Bool, "/mission/control_enable", 10)
        self.rc_pub = self.create_publisher(
            OverrideRCIn, "/mavros/rc/override", 10
        )
        self.create_subscription(String, "/mission/state", self._on_state, 10)
        self.create_subscription(
            Float32MultiArray, "/vision/buoy_bbox", self._on_bbox, 10
        )
        self.create_subscription(
            String, "/mujoco/course_buoys/status", self._on_buoys, 10
        )
        self.create_timer(0.2, self._tick)

    def _on_state(self, msg: String) -> None:
        state = str(msg.data).strip() or "UNKNOWN"
        self.state = state
        if not self.state_history or self.state_history[-1] != state:
            self.state_history.append(state)
            self._write_status("running")

    def _on_bbox(self, msg: Float32MultiArray) -> None:
        self.bbox_messages += 1
        values = list(msg.data)
        for base in range(0, len(values) - 9, 10):
            if float(values[base + 1]) < 0.5:
                continue
            self.detection_count += 1
            self.last_detection_wall = time.monotonic()
            class_id = str(int(round(float(values[base + 2]))))
            width = float(values[base + 6])
            height = float(values[base + 7])
            image_width = max(1.0, float(values[base + 8]))
            image_height = max(1.0, float(values[base + 9]))
            area_ratio = max(0.0, width * height / (image_width * image_height))
            self.class_detection_count[class_id] = (
                self.class_detection_count.get(class_id, 0) + 1
            )
            self.class_max_area_ratio[class_id] = max(
                self.class_max_area_ratio.get(class_id, 0.0), area_ratio
            )
            self.last_detections[class_id] = {
                "confidence": round(float(values[base + 3]), 4),
                "center_x_ratio": round(float(values[base + 4]) / image_width, 4),
                "center_y_ratio": round(float(values[base + 5]) / image_height, 4),
                "area_ratio": round(area_ratio, 6),
            }

    def _on_buoys(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        successful: set[str] = set()
        for buoy in payload.get("buoys", []):
            if not isinstance(buoy, dict):
                continue
            buoy_id = str(buoy.get("id", ""))
            if bool(buoy.get("netted", False)):
                successful.add(f"netted:{buoy_id}")
            if bool(buoy.get("has_magnet", False)) and bool(
                buoy.get("detached", False)
            ):
                successful.add(f"detached:{buoy_id}")
        if self.initial_physical_success is None:
            self.initial_physical_success = set(successful)
        self.physical_success = successful - self.initial_physical_success

    def _tick(self) -> None:
        msg = Bool()
        # Do not arm the FSM while PyTorch is still loading the model.  The
        # first bbox message (detected or empty) proves the whole camera/YOLO
        # path is live.
        msg.data = self.bbox_messages > 0
        self.enable_pub.publish(msg)

    def elapsed_s(self) -> float:
        return time.monotonic() - self.started_wall

    def succeeded(self) -> bool:
        return bool(self.physical_success)

    def publish_prescan(self, yaw_pwm: int | None) -> None:
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        if yaw_pwm is None:
            for index in (2, 3, 4):
                msg.channels[index] = OverrideRCIn.CHAN_RELEASE
        else:
            msg.channels[2] = 1500
            msg.channels[3] = int(yaw_pwm)
            msg.channels[4] = 1500
        self.rc_pub.publish(msg)

    def _payload(self, result: str) -> dict[str, object]:
        return {
            "available": True,
            "mission": "vision",
            "result": result,
            "state": self.state,
            "state_history": list(self.state_history),
            "elapsed_wall_s": round(self.elapsed_s(), 3),
            "detection_count": self.detection_count,
            "class_detection_count": dict(self.class_detection_count),
            "class_max_area_ratio": {
                key: round(value, 6)
                for key, value in self.class_max_area_ratio.items()
            },
            "last_detections": dict(self.last_detections),
            "bbox_messages": self.bbox_messages,
            "last_detection_age_s": (
                None
                if self.last_detection_wall <= 0.0
                else round(time.monotonic() - self.last_detection_wall, 3)
            ),
            "physical_success": sorted(self.physical_success),
            "prescan_elapsed_s": round(self.prescan_elapsed_s, 3),
            "prescan_selected_area_ratio": round(
                self.prescan_selected_area_ratio, 6
            ),
        }

    def _write_status(self, result: str) -> None:
        self.status_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.status_path.with_suffix(self.status_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(self._payload(result), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.status_path)

    def finish(self, result: str) -> None:
        msg = Bool()
        msg.data = False
        self.enable_pub.publish(msg)
        self._write_status(result)


def _terminate(process: subprocess.Popen[bytes] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def _commands(model_path: Path) -> tuple[list[str], list[str]]:
    detector = [
        "ros2",
        "launch",
        "auv_buoy_vision_control",
        "laptop_yolo_detection.launch.py",
        "image_topic:=/camera/camera/color/image_raw/compressed",
        "annotated_image_topic:=/vision/buoy/image_annotated/compressed",
        "publish_annotated_image:=true",
        "bbox_topic:=/vision/buoy_bbox",
        # The model was trained for the standard 640-pixel YOLO surface.
        # Keeping inference there gives the CPU controller enough fresh
        # observations to close the loop instead of steering on stale boxes.
        "imgsz:=640",
        "cpu_threads:=1",
        "confidence_threshold:=0.10",
        "target_class_id:=-1",
        "publish_per_class:=true",
        "show_preview:=false",
        f"model_path:={model_path}",
    ]
    controller = [
        "ros2",
        "run",
        "auv_buoy_vision_control",
        "mission_state_machine_node",
        "--ros-args",
        "-p",
        "bbox_topic:=/vision/buoy_bbox",
        "-p",
        "depth_pose_topic:=/depth/pose",
        "-p",
        "depth_pose_scale:=-1.0",
        "-p",
        "work_depth_m:=0.50",
        "-p",
        "surface_depth_m:=0.10",
        "-p",
        "max_depth_m:=1.30",
        "-p",
        "vertical_positive_is_up:=true",
        "-p",
        "buoyancy_hold_delta_pwm:=30",
        "-p",
        "depth_kp_pwm_per_m:=200.0",
        "-p",
        "depth_stable_sec:=0.8",
        "-p",
        "depth_timeout_sec:=3.0",
        "-p",
        "detection_timeout_sec:=4.0",
        "-p",
        "buoy_same_target_center_ratio:=0.50",
        "-p",
        "min_detection_hits:=1",
        "-p",
        # Do not stop several metres from a small surface float.  The
        # collector mouth must be within contact range before switching from
        # surge approach to fine stick alignment.
        "approach_area_ratio:=0.03",
        "-p",
        # The surface buoy's stick enters near the top of the camera image.
        # Retain a small depth-hold component, but let visual tracking raise
        # the collector until the stick reaches the fork target.
        "approach_vision_throttle_weight:=0.85",
        "-p",
        "fork_target_x:=0.50",
        "-p",
        # The MuJoCo camera is mounted lower than the package's pool-camera
        # default; the collector mouth projects at roughly 40% image height.
        "fork_target_y:=0.40",
        "-p",
        "stick_deadband_x:=0.12",
        "-p",
        "stick_deadband_y:=0.15",
        "-p",
        "align_stable_sec:=0.30",
        "-p",
        "insert_duration_sec:=0.45",
        "-p",
        "detach_duration_sec:=0.20",
        "-p",
        "backoff_duration_sec:=0.35",
        "-p",
        "verify_clear_sec:=0.60",
        "-p",
        "verify_timeout_sec:=2.0",
        "-p",
        "max_target_retries:=1",
        "-p",
        "search_timeout_sec:=15.0",
        "-p",
        "area_verify_sec:=3.0",
        "-p",
        "approach_forward_pwm:=1700",
        "-p",
        "approach_forward_min_pwm:=1580",
        "-p",
        "search_yaw_pwm:=1570",
    ]
    return detector, controller


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--status-json", type=Path, required=True)
    parser.add_argument("--deadline-s", type=float, default=75.0)
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error(f"model does not exist: {args.model}")

    detector_cmd, controller_cmd = _commands(args.model)
    detector: subprocess.Popen[bytes] | None = None
    controller: subprocess.Popen[bytes] | None = None
    rclpy.init()
    monitor = MissionMonitor(args.status_json)
    try:
        detector = subprocess.Popen(detector_cmd)
        monitor._write_status("starting")
        # The normal competition spawn initially sees a much farther float
        # straight ahead than the red buoy a few metres to port.  Do a bounded
        # sensor-only yaw scan before launching the untouched package
        # controller, then hand RC ownership over at the largest nearby box.
        scan_started: float | None = None
        while rclpy.ok() and monitor.elapsed_s() < min(args.deadline_s, 20.0):
            rclpy.spin_once(monitor, timeout_sec=0.1)
            if monitor.bbox_messages <= 0:
                continue
            if scan_started is None:
                scan_started = time.monotonic()
            scan_elapsed = time.monotonic() - scan_started
            monitor.prescan_elapsed_s = scan_elapsed
            monitor.publish_prescan(1650)
            latest_buoy = monitor.last_detections.get("0", {})
            latest_area = float(latest_buoy.get("area_ratio", 0.0))
            if (
                scan_elapsed >= 0.5
                and latest_area >= 0.0016
            ) or scan_elapsed >= 12.0:
                monitor.prescan_selected_area_ratio = latest_area
                break
        monitor.publish_prescan(None)
        for _ in range(2):
            rclpy.spin_once(monitor, timeout_sec=0.1)
        controller = subprocess.Popen(controller_cmd)
        while rclpy.ok() and monitor.elapsed_s() < max(5.0, args.deadline_s):
            rclpy.spin_once(monitor, timeout_sec=0.1)
            if monitor.succeeded():
                monitor.finish("success")
                print(
                    "vision_mission=PASS "
                    f"elapsed={monitor.elapsed_s():.2f}s "
                    f"physical={sorted(monitor.physical_success)}",
                    flush=True,
                )
                return 0
            if monitor.state == "FAILSAFE":
                monitor.finish("failsafe")
                print("vision_mission=FAIL reason=FAILSAFE", file=sys.stderr)
                return 1
            if controller.poll() is not None:
                monitor.finish("controller_exited")
                print("vision_mission=FAIL reason=controller_exited", file=sys.stderr)
                return 1
        monitor.finish("timeout")
        print(
            f"vision_mission=FAIL reason=timeout state={monitor.state}",
            file=sys.stderr,
        )
        return 1
    finally:
        _terminate(controller)
        _terminate(detector)
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
