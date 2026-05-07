#!/usr/bin/env python3
"""Apply/revert MuJoCo AUV stability candidates and measure ALT_HOLD response.

This tool intentionally modifies only files under uuv_mujoco/v2.2, restores
them after each candidate, starts the SITL/MuJoCo stack headless, commands
MANUAL -> arm -> ALT_HOLD with neutral RC override, and records short stability
metrics from ROS2 topics.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import signal
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn, RCOut, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / "config" / "sim_profiles.json"
SCENE_PATH = ROOT / "scenes" / "tank_current_scene.xml"
MAPPING_PATH = ROOT / "physics" / "thruster_mapping.py"
START_SCRIPT = ROOT / "start_sitl_mujoco_mj311.sh"
RESET_SCRIPT = ROOT / "reset_uuv_sim.sh"
LOG_ROOT = ROOT / "logs"

FLUID_GEOMS = (
    "fluid_center_enclosure",
    "fluid_port_lower_body",
    "fluid_starboard_lower_body",
)


@dataclass(frozen=True)
class Candidate:
    name: str
    profile_updates: dict[str, Any] = field(default_factory=dict)
    fluid_angular_scale: float | None = None
    servo_signs: tuple[int, ...] | None = None
    note: str = ""


def default_candidates(include_sign_checks: bool) -> list[Candidate]:
    candidates = [
        Candidate("baseline_current", note="current profile as-is"),
        Candidate(
            "restore_too_strong_check",
            {
                "buoyancy_scale": 1.015,
                "cob_torque_scale": 1.05,
                "cob_z_offset": 0.026,
                "heave_damping_scale": 4.4,
            },
            note="previous stronger restoring candidate",
        ),
        Candidate(
            "restore_soft",
            {
                "buoyancy_scale": 1.003,
                "cob_torque_scale": 0.50,
                "cob_z_offset": 0.012,
                "heave_damping_scale": 4.4,
            },
            note="lower GM/restoring stiffness",
        ),
        Candidate(
            "near_neutral_soft",
            {
                "buoyancy_scale": 1.0005,
                "cob_torque_scale": 0.50,
                "cob_z_offset": 0.012,
                "heave_damping_scale": 4.4,
            },
            note="almost neutral buoyancy with soft restoring",
        ),
        Candidate(
            "heave_damping_low",
            {"heave_damping_scale": 2.2},
            note="test over-damped z-loop hypothesis",
        ),
        Candidate(
            "heave_damping_high",
            {"heave_damping_scale": 6.0},
            note="test under-damped z-loop hypothesis",
        ),
        Candidate(
            "ellipsoid_roll_damping_p50",
            fluid_angular_scale=1.50,
            note="increase MuJoCo ellipsoid angular damping only",
        ),
        Candidate(
            "ellipsoid_roll_damping_p100",
            fluid_angular_scale=2.00,
            note="stronger MuJoCo ellipsoid angular damping only",
        ),
        Candidate(
            "near_neutral_ellipsoid_p50",
            {
                "buoyancy_scale": 1.0005,
                "cob_torque_scale": 0.50,
                "cob_z_offset": 0.012,
                "heave_damping_scale": 4.4,
            },
            fluid_angular_scale=1.50,
            note="near-neutral hydrostatics plus moderate ellipsoid angular damping",
        ),
        Candidate(
            "near_neutral_ellipsoid_p100",
            {
                "buoyancy_scale": 1.0005,
                "cob_torque_scale": 0.50,
                "cob_z_offset": 0.012,
                "heave_damping_scale": 4.4,
            },
            fluid_angular_scale=2.00,
            note="near-neutral hydrostatics plus strong ellipsoid angular damping",
        ),
        Candidate(
            "legacy_prefix_physics",
            {
                "buoyancy_scale": 1.005,
                "cob_torque_scale": 0.75,
                "cob_z_offset": 0.018,
                "heave_damping_scale": 4.4,
            },
            fluid_angular_scale=2.0 / 3.0,
            note="diagnostic: earlier stronger hydrostatics and earlier ellipsoid angular damping",
        ),
        Candidate(
            "legacy_hydro_current_ellipsoid",
            {
                "buoyancy_scale": 1.005,
                "cob_torque_scale": 0.75,
                "cob_z_offset": 0.018,
                "heave_damping_scale": 4.4,
            },
            note="diagnostic: earlier stronger hydrostatics with current ellipsoid angular damping",
        ),
        Candidate(
            "current_hydro_old_ellipsoid",
            fluid_angular_scale=2.0 / 3.0,
            note="diagnostic: current hydrostatics with weaker earlier ellipsoid angular damping",
        ),
        Candidate(
            "current_hydro_stronger_ellipsoid",
            fluid_angular_scale=4.0 / 3.0,
            note="diagnostic: current hydrostatics with stronger ellipsoid angular damping",
        ),
        Candidate(
            "too_soft_hydro",
            {
                "buoyancy_scale": 1.0000,
                "cob_torque_scale": 0.25,
                "cob_z_offset": 0.006,
                "heave_damping_scale": 4.4,
            },
            note="diagnostic: under-restored hydrostatics",
        ),
    ]
    if include_sign_checks:
        candidates.extend(
            [
                Candidate(
                    "vertical_signs_inverted_check",
                    servo_signs=(-1, -1, 1, 1, 1, -1, -1, 1),
                    note="diagnostic only: invert vertical PWM-to-force signs",
                ),
                Candidate(
                    "qgc_reverse_removed_check",
                    servo_signs=(-1, -1, -1, -1, 1, 1, 1, 1),
                    note="diagnostic only: remove simulated QGC reverse compensation for motors 3/4/5/8",
                ),
                Candidate(
                    "horizontal_yaw_signs_inverted_check",
                    servo_signs=(1, 1, -1, -1, -1, 1, 1, -1),
                    note="diagnostic only: invert horizontal/yaw PWM-to-force signs",
                ),
            ]
        )
    return candidates


def write_profile(original_profile_text: str, candidate: Candidate) -> None:
    profiles = json.loads(original_profile_text)
    current = dict(profiles["current"])
    current.update(candidate.profile_updates)
    profiles["current"] = current
    PROFILE_PATH.write_text(json.dumps(profiles, indent=2, ensure_ascii=False) + "\n")


def scale_fluid_angular(scene_text: str, scale: float | None) -> str:
    if scale is None:
        return scene_text
    updated = scene_text
    for geom_name in FLUID_GEOMS:
        pattern = re.compile(
            rf'(<geom\b(?=[^>]*\bname="{re.escape(geom_name)}")[^>]*\bfluidcoef=")([^"]+)(")',
            re.DOTALL,
        )

        def repl(match: re.Match[str]) -> str:
            values = [float(part) for part in match.group(2).split()]
            if len(values) < 3:
                raise RuntimeError(f"{geom_name} fluidcoef has fewer than 3 values")
            values[2] *= float(scale)
            return match.group(1) + " ".join(f"{value:.6g}" for value in values) + match.group(3)

        updated, count = pattern.subn(repl, updated, count=1)
        if count != 1:
            raise RuntimeError(f"Could not find fluidcoef for {geom_name}")
    return updated


def write_scene(original_scene_text: str, candidate: Candidate) -> None:
    SCENE_PATH.write_text(scale_fluid_angular(original_scene_text, candidate.fluid_angular_scale))


def write_mapping(original_mapping_text: str, candidate: Candidate) -> None:
    if candidate.servo_signs is None:
        MAPPING_PATH.write_text(original_mapping_text)
        return
    replacement = "ARDUSUB_VECTORED_6DOF_SERVO_SIGNS = (" + ", ".join(str(v) for v in candidate.servo_signs) + ")"
    updated, count = re.subn(
        r"ARDUSUB_VECTORED_6DOF_SERVO_SIGNS\s*=\s*\([^)]+\)",
        replacement,
        original_mapping_text,
        count=1,
    )
    if count != 1:
        raise RuntimeError("Could not replace ARDUSUB_VECTORED_6DOF_SERVO_SIGNS")
    MAPPING_PATH.write_text(updated)


def apply_candidate(original_profile_text: str, original_scene_text: str, original_mapping_text: str, candidate: Candidate) -> None:
    write_profile(original_profile_text, candidate)
    write_scene(original_scene_text, candidate)
    write_mapping(original_mapping_text, candidate)


def restore_files(original_profile_text: str, original_scene_text: str, original_mapping_text: str) -> None:
    PROFILE_PATH.write_text(original_profile_text)
    SCENE_PATH.write_text(original_scene_text)
    MAPPING_PATH.write_text(original_mapping_text)


def quat_to_rpy_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def rms(values: list[float]) -> float:
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


def stddev(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0 if values else float("nan")
    return statistics.pstdev(values)


class StabilityProbe(Node):
    def __init__(self) -> None:
        super().__init__("uuv_roll_stability_probe")
        self.state: State | None = None
        self.samples: list[dict[str, Any]] = []
        self.depth_samples: list[tuple[float, float]] = []
        self.rc_samples: list[tuple[float, list[int]]] = []
        self.latest_gyro = (float("nan"), float("nan"), float("nan"))
        self.latest_depth = float("nan")
        self.sample_enabled = False
        self.t0 = time.monotonic()

        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.state_sub = self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/mujoco/ground_truth/pose", self._on_pose, 50)
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self._on_imu, 50)
        self.depth_sub = self.create_subscription(Float32, "/depth", self._on_depth, 10)
        self.rc_out_sub = self.create_subscription(RCOut, "/mavros/rc/out", self._on_rc_out, 30)
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

    def elapsed(self) -> float:
        return time.monotonic() - self.t0

    def _on_state(self, msg: State) -> None:
        self.state = msg

    def _on_imu(self, msg: Imu) -> None:
        self.latest_gyro = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )

    def _on_depth(self, msg: Float32) -> None:
        self.latest_depth = float(msg.data)
        if self.sample_enabled:
            self.depth_samples.append((self.elapsed(), self.latest_depth))

    def _on_rc_out(self, msg: RCOut) -> None:
        if self.sample_enabled:
            self.rc_samples.append((self.elapsed(), [int(v) for v in msg.channels]))

    def _on_pose(self, msg: PoseStamped) -> None:
        if not self.sample_enabled:
            return
        q = msg.pose.orientation
        roll, pitch, yaw = quat_to_rpy_deg(float(q.w), float(q.x), float(q.y), float(q.z))
        gx, gy, gz = self.latest_gyro
        self.samples.append(
            {
                "t": self.elapsed(),
                "roll_deg": roll,
                "pitch_deg": pitch,
                "yaw_deg": yaw,
                "z_m": float(msg.pose.position.z),
                "depth_m": self.latest_depth,
                "gyro_x": gx,
                "gyro_y": gy,
                "gyro_z": gz,
            }
        )

    def publish_rc(self, *, forward: float = 0.0, sway: float = 0.0, yaw: float = 0.0, heave: float = 0.0) -> None:
        msg = OverrideRCIn()
        for idx in range(len(msg.channels)):
            msg.channels[idx] = 0
        for idx in range(min(8, len(msg.channels))):
            msg.channels[idx] = 1500
        if len(msg.channels) >= 6:
            msg.channels[2] = int(round(1500 + 300.0 * max(-1.0, min(1.0, heave))))
            msg.channels[3] = int(round(1500 + 300.0 * max(-1.0, min(1.0, yaw))))
            msg.channels[4] = int(round(1500 + 300.0 * max(-1.0, min(1.0, forward))))
            msg.channels[5] = int(round(1500 + 300.0 * max(-1.0, min(1.0, sway))))
        self.rc_pub.publish(msg)

    def neutral_rc(self) -> None:
        self.publish_rc()

    def spin_neutral(self, duration: float, hz: float = 20.0) -> None:
        self.spin_rc(duration, hz=hz)

    def spin_rc(
        self,
        duration: float,
        *,
        forward: float = 0.0,
        sway: float = 0.0,
        yaw: float = 0.0,
        heave: float = 0.0,
        hz: float = 20.0,
    ) -> None:
        end_t = time.monotonic() + duration
        dt = 1.0 / hz
        while time.monotonic() < end_t:
            self.publish_rc(forward=forward, sway=sway, yaw=yaw, heave=heave)
            rclpy.spin_once(self, timeout_sec=min(0.05, dt))
            remaining = end_t - time.monotonic()
            if remaining > 0:
                time.sleep(min(dt, remaining))

    def wait_for_stack(self, timeout: float = 75.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            services_ok = self.arm_client.wait_for_service(timeout_sec=0.15) and self.mode_client.wait_for_service(timeout_sec=0.15)
            rclpy.spin_once(self, timeout_sec=0.05)
            state_ok = self.state is not None and bool(self.state.connected)
            if services_ok and state_ok:
                return
        raise RuntimeError("ROS2/MAVROS services or connected state did not become ready")

    def set_mode(self, mode: str, timeout: float = 8.0) -> None:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not future.done():
            self.neutral_rc()
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(f"set_mode({mode}) timeout")
        response = future.result()
        response_ok = response is not None and bool(response.mode_sent)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.neutral_rc()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state is not None and str(self.state.mode) == mode:
                return
        if not response_ok:
            raise RuntimeError(f"set_mode({mode}) failed")
        raise RuntimeError(f"state did not report mode {mode}")

    def arm(self, value: bool, timeout: float = 8.0) -> None:
        req = CommandBool.Request()
        req.value = bool(value)
        future = self.arm_client.call_async(req)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not future.done():
            self.neutral_rc()
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(f"arming({value}) timeout")
        response = future.result()
        response_ok = response is not None and bool(response.success)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.neutral_rc()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state is not None and bool(self.state.armed) == bool(value):
                return
        if not response_ok:
            raise RuntimeError(f"arming({value}) failed")
        raise RuntimeError(f"state did not report armed={value}")

    def run_probe(self, settle_s: float, measure_s: float, stimulus: str, hold_mode: str) -> dict[str, Any]:
        self.wait_for_stack()
        self.spin_neutral(1.5)
        self.set_mode("MANUAL")
        self.spin_neutral(1.0)
        self.arm(True)
        self.spin_neutral(3.0)
        if hold_mode != "MANUAL":
            self.set_mode(hold_mode)
        self.spin_neutral(settle_s)

        self.samples.clear()
        self.depth_samples.clear()
        self.rc_samples.clear()
        self.sample_enabled = True
        if stimulus == "neutral":
            self.spin_neutral(measure_s)
        elif stimulus == "heave-pulse":
            pulse_s = min(0.35, max(0.05, measure_s * 0.08))
            self.spin_rc(pulse_s, heave=-0.25)
            self.spin_neutral(max(0.0, measure_s - pulse_s))
        elif stimulus == "forward-pulse":
            pulse_s = min(0.60, max(0.10, measure_s * 0.10))
            self.spin_rc(pulse_s, forward=0.20)
            self.spin_neutral(max(0.0, measure_s - pulse_s))
        else:
            raise RuntimeError(f"unknown stimulus: {stimulus}")
        self.sample_enabled = False
        self.arm(False)
        self.spin_neutral(0.5)
        return compute_metrics(self.samples, self.depth_samples, self.rc_samples, measure_s)


def compute_metrics(
    samples: list[dict[str, Any]],
    depth_samples: list[tuple[float, float]],
    rc_samples: list[tuple[float, list[int]]],
    measure_s: float,
) -> dict[str, Any]:
    if len(samples) < max(8, measure_s * 2):
        raise RuntimeError(f"too few pose samples collected: {len(samples)}")

    roll = [float(s["roll_deg"]) for s in samples]
    pitch = [float(s["pitch_deg"]) for s in samples]
    gyro_x = [float(s["gyro_x"]) for s in samples if math.isfinite(float(s["gyro_x"]))]
    gyro_y = [float(s["gyro_y"]) for s in samples if math.isfinite(float(s["gyro_y"]))]
    z = [float(s["z_m"]) for s in samples]
    depths = [float(value) for _, value in depth_samples if math.isfinite(float(value))]

    vertical_rc = []
    roll_mix = []
    rc_valid_samples = 0
    rc_min = [math.nan] * 8
    rc_max = [math.nan] * 8
    for _, channels in rc_samples:
        if len(channels) >= 8 and all(900 <= channels[idx] <= 2100 for idx in range(8)):
            rc_valid_samples += 1
            for idx in range(8):
                value = int(channels[idx])
                rc_min[idx] = value if math.isnan(rc_min[idx]) else min(rc_min[idx], value)
                rc_max[idx] = value if math.isnan(rc_max[idx]) else max(rc_max[idx], value)
        if len(channels) >= 8 and all(900 <= channels[idx] <= 2100 for idx in range(4, 8)):
            devs = [channels[idx] - 1500 for idx in range(4, 8)]
            vertical_rc.extend(float(v) for v in devs)
            roll_mix.append(float((channels[5] + channels[6]) - (channels[4] + channels[7])))

    depth_drift = float(depths[-1] - depths[0]) if len(depths) >= 2 else float("nan")
    z_drift = float(z[-1] - z[0]) if len(z) >= 2 else float("nan")

    metrics: dict[str, Any] = {
        "pose_samples": len(samples),
        "depth_samples": len(depths),
        "rc_samples": len(rc_samples),
        "roll_rms_deg": rms(roll),
        "roll_peak_deg": max(abs(v) for v in roll),
        "roll_mean_deg": statistics.fmean(roll),
        "pitch_rms_deg": rms(pitch),
        "pitch_peak_deg": max(abs(v) for v in pitch),
        "gyro_x_rms_rad_s": rms(gyro_x),
        "gyro_y_rms_rad_s": rms(gyro_y),
        "depth_mean_m": statistics.fmean(depths) if depths else float("nan"),
        "depth_std_m": stddev(depths),
        "depth_drift_m": depth_drift,
        "z_drift_m": z_drift,
        "vertical_rc_rms_pwm": rms(vertical_rc),
        "roll_mix_rms_pwm": rms(roll_mix),
        "rc_valid_samples": rc_valid_samples,
        "rc_min_ch1_8": rc_min,
        "rc_max_ch1_8": rc_max,
    }

    score = (
        metrics["roll_rms_deg"]
        + 0.35 * metrics["pitch_rms_deg"]
        + 8.0 * metrics["gyro_x_rms_rad_s"]
        + 15.0 * metrics["depth_std_m"]
        + 10.0 * abs(metrics["depth_drift_m"] if math.isfinite(metrics["depth_drift_m"]) else 0.0)
    )
    metrics["score"] = float(score)
    return metrics


def stop_stack() -> None:
    subprocess.run([str(RESET_SCRIPT), "--with-qgc-stop"], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def wait_for_launcher_ready(proc: subprocess.Popen, launch_log: Path, timeout_s: float = 90.0) -> None:
    deadline = time.monotonic() + float(timeout_s)
    last_text = ""
    while time.monotonic() < deadline:
        if launch_log.exists():
            last_text = launch_log.read_text(errors="replace")
            if "readiness OK:" in last_text:
                return
            failure_markers = (
                "SITL exited while waiting for readiness",
                "MuJoCo exited while waiting for readiness",
                "readiness timeout",
            )
            for marker in failure_markers:
                if marker in last_text:
                    raise RuntimeError(f"launcher readiness failed: {marker}")
        if proc.poll() is not None:
            raise RuntimeError(f"launcher exited before readiness, code={proc.returncode}")
        time.sleep(0.25)
    tail = "\n".join(last_text.splitlines()[-40:])
    raise RuntimeError(f"launcher readiness timeout after {timeout_s:.1f}s\n{tail}")


def run_candidate(
    candidate: Candidate,
    out_dir: Path,
    settle_s: float,
    measure_s: float,
    stimulus: str,
    hold_mode: str,
    wait_ready: bool,
) -> dict[str, Any]:
    cand_dir = out_dir / candidate.name
    cand_dir.mkdir(parents=True, exist_ok=True)
    launch_log = cand_dir / "launcher.log"
    stop_stack()
    with launch_log.open("w") as log:
        proc = subprocess.Popen(
            [
                str(START_SCRIPT),
                "--sitl-no-rebuild",
                "--ros2",
                "--",
                "--headless",
                "--no-qgc-video",
                "--tank-549x274x132",
            ],
            cwd=ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    try:
        if wait_ready:
            wait_for_launcher_ready(proc, launch_log)
        rclpy.init(args=None)
        node = StabilityProbe()
        try:
            metrics = node.run_probe(settle_s=settle_s, measure_s=measure_s, stimulus=stimulus, hold_mode=hold_mode)
            if int(metrics.get("rc_valid_samples", 0)) <= 0:
                raise RuntimeError("invalid_no_servo_output: /mavros/rc/out produced no valid 8-channel PWM samples")
            if stimulus != "neutral":
                vertical_pwm = float(metrics.get("vertical_rc_rms_pwm", float("nan")))
                if not math.isfinite(vertical_pwm) or vertical_pwm < 2.0:
                    raise RuntimeError(
                        "invalid_no_active_vertical_servo_output: "
                        f"vertical_rc_rms_pwm={vertical_pwm}"
                    )
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as exc:
        metrics = {"status": "fail", "error": str(exc)}
    finally:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except Exception:
            pass
        stop_stack()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass
        stop_stack()

    result = {
        "candidate": candidate.name,
        "note": candidate.note,
        "profile_updates": candidate.profile_updates,
        "fluid_angular_scale": candidate.fluid_angular_scale,
        "servo_signs": list(candidate.servo_signs) if candidate.servo_signs is not None else None,
        "launcher_log": str(launch_log),
        "stimulus": stimulus,
        "hold_mode": hold_mode,
        "wait_ready": wait_ready,
        **metrics,
    }
    (cand_dir / "metrics.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    return result


def write_summary(out_dir: Path, results: list[dict[str, Any]]) -> None:
    keys = [
        "candidate",
        "status",
        "score",
        "roll_rms_deg",
        "roll_peak_deg",
        "pitch_rms_deg",
        "gyro_x_rms_rad_s",
        "depth_std_m",
        "depth_drift_m",
        "vertical_rc_rms_pwm",
        "roll_mix_rms_pwm",
        "rc_valid_samples",
        "stimulus",
        "hold_mode",
        "wait_ready",
        "note",
        "error",
        "launcher_log",
    ]
    with (out_dir / "summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for result in results:
            row = {key: result.get(key, "") for key in keys}
            row["status"] = result.get("status", "ok" if "score" in result else "fail")
            writer.writerow(row)
    (out_dir / "summary.json").write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--measure-s", type=float, default=8.0)
    parser.add_argument("--settle-s", type=float, default=4.0)
    parser.add_argument("--max-candidates", type=int, default=0, help="0 means all")
    parser.add_argument("--candidate", action="append", default=[])
    parser.add_argument("--include-sign-checks", action="store_true")
    parser.add_argument("--stimulus", choices=("neutral", "heave-pulse", "forward-pulse"), default="neutral")
    parser.add_argument("--hold-mode", choices=("ALT_HOLD", "MANUAL"), default="ALT_HOLD")
    parser.add_argument("--skip-launcher-ready", action="store_true")
    parser.add_argument("--out-dir", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir or (LOG_ROOT / f"roll_stability_sweep_{time.strftime('%Y%m%d_%H%M%S')}")
    out_dir.mkdir(parents=True, exist_ok=True)

    original_profile_text = PROFILE_PATH.read_text()
    original_scene_text = SCENE_PATH.read_text()
    original_mapping_text = MAPPING_PATH.read_text()

    candidates = default_candidates(include_sign_checks=args.include_sign_checks)
    if args.candidate:
        wanted = set(args.candidate)
        candidates = [candidate for candidate in candidates if candidate.name in wanted]
    if args.max_candidates > 0:
        candidates = candidates[: args.max_candidates]
    if not candidates:
        raise RuntimeError("No candidates selected")

    print(f"[sweep] out={out_dir}", flush=True)
    print(f"[sweep] candidates={', '.join(c.name for c in candidates)}", flush=True)

    results: list[dict[str, Any]] = []
    try:
        for idx, candidate in enumerate(candidates, start=1):
            print(f"[sweep] {idx}/{len(candidates)} apply {candidate.name}: {candidate.note}", flush=True)
            restore_files(original_profile_text, original_scene_text, original_mapping_text)
            apply_candidate(original_profile_text, original_scene_text, original_mapping_text, candidate)
            result = run_candidate(
                candidate,
                out_dir,
                settle_s=args.settle_s,
                measure_s=args.measure_s,
                stimulus=args.stimulus,
                hold_mode=args.hold_mode,
                wait_ready=not args.skip_launcher_ready,
            )
            results.append(result)
            write_summary(out_dir, results)
            if "score" in result:
                print(
                    "[sweep] result "
                    f"{candidate.name}: score={result['score']:.3f}, "
                    f"roll_rms={result['roll_rms_deg']:.2f}deg, "
                    f"gyro_x_rms={result['gyro_x_rms_rad_s']:.3f}rad/s, "
                    f"depth_std={result['depth_std_m']:.3f}m",
                    flush=True,
                )
            else:
                print(f"[sweep] result {candidate.name}: FAIL {result.get('error')}", flush=True)
    finally:
        restore_files(original_profile_text, original_scene_text, original_mapping_text)
        stop_stack()

    ok_results = [result for result in results if "score" in result]
    if ok_results:
        best = min(ok_results, key=lambda item: float(item["score"]))
        print(f"[sweep] best={best['candidate']} score={best['score']:.3f}", flush=True)
    else:
        print("[sweep] no successful candidates", flush=True)
        return 1
    print("[sweep] original files restored; inspect summary.csv before applying anything.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
