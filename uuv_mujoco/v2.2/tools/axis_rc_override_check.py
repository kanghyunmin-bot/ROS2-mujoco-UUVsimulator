#!/usr/bin/env python3
"""Axis-by-axis RC override validation for the MuJoCo + ArduSub SITL stack."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import ManualControl, OverrideRCIn, RCIn, RCOut, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


WORKSPACE = Path(__file__).resolve().parents[1]
LOG_ROOT = WORKSPACE / "logs"
RC_NEUTRAL = 1500
RC_SPAN = 400
AXIS_TO_CHANNEL = {
    "pitch": 0,    # ch1
    "roll": 1,     # ch2
    "heave": 2,    # ch3
    "yaw": 3,      # ch4
    "forward": 4,  # ch5
    "lateral": 5,  # ch6
}
AXIS_ORDER = ("roll", "pitch", "yaw", "heave", "forward", "lateral")
EXPECTED_AXIS_METRIC = {
    "roll": "gyro_x",
    "pitch": "gyro_y",
    "yaw": "gyro_z",
    "heave": "dvl_vz",
    "forward": "dvl_vx",
    "lateral": "dvl_vy",
}
MIN_EXPECTED_PEAK = {
    "roll": 0.003,
    "pitch": 0.003,
    "yaw": 0.03,
    "heave": 0.01,
    "forward": 0.02,
    "lateral": 0.02,
}


@dataclass
class Phase:
    name: str
    axis: str
    command: float
    start: float
    end: float


def quat_to_rpy_rad(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rms(values: list[float]) -> float:
    finite = [float(v) for v in values if math.isfinite(float(v))]
    if not finite:
        return float("nan")
    return math.sqrt(sum(v * v for v in finite) / len(finite))


def mean(values: list[float]) -> float:
    finite = [float(v) for v in values if math.isfinite(float(v))]
    if not finite:
        return float("nan")
    return sum(finite) / len(finite)


def finite_values(values: list[float]) -> list[float]:
    return [float(v) for v in values if math.isfinite(float(v))]


class AxisRcOverrideCheck(Node):
    def __init__(self, sample_hz: float, input_mode: str = "rc-override") -> None:
        super().__init__("axis_rc_override_check")
        self.sample_dt = 1.0 / max(1.0, float(sample_hz))
        self.input_mode = str(input_mode)
        self._rc_released = False
        self.start_wall = time.monotonic()
        self._last_sample_wall = -1.0
        self.current_phase = "init"
        self.recording = False

        self.state: State | None = None
        self.imu: Imu | None = None
        self.dvl_twist: TwistWithCovarianceStamped | None = None
        self.depth: Float32 | None = None
        self.local_odom: Odometry | None = None
        self.rc_in: RCIn | None = None
        self.rc_out: RCOut | None = None

        self.samples: list[dict[str, Any]] = []
        self.phases: list[Phase] = []

        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.manual_pub = self.create_publisher(ManualControl, "/mavros/manual_control/send", 10)
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.switch_initial_depth_hold_client = self.create_client(
            Trigger,
            "/mujoco/switch_initial_depth_hold_to_target",
        )
        self.release_initial_depth_hold_client = self.create_client(Trigger, "/mujoco/release_initial_depth_hold")

        self.create_subscription(State, "/mavros/state", self._on_state, 20)
        self.create_subscription(RCIn, "/mavros/rc/in", self._on_rc_in, 20)
        self.create_subscription(RCOut, "/mavros/rc/out", self._on_rc_out, 20)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 50)
        self.create_subscription(TwistWithCovarianceStamped, "/dvl/twist", self._on_dvl_twist, 50)
        self.create_subscription(Float32, "/depth", self._on_depth, 20)
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._on_local_odom, 20)
        self.create_timer(self.sample_dt, self._sample)

    def elapsed(self) -> float:
        return time.monotonic() - self.start_wall

    def _on_state(self, msg: State) -> None:
        self.state = msg

    def _on_rc_in(self, msg: RCIn) -> None:
        self.rc_in = msg

    def _on_rc_out(self, msg: RCOut) -> None:
        self.rc_out = msg

    def _on_imu(self, msg: Imu) -> None:
        self.imu = msg

    def _on_dvl_twist(self, msg: TwistWithCovarianceStamped) -> None:
        self.dvl_twist = msg

    def _on_depth(self, msg: Float32) -> None:
        self.depth = msg

    def _on_local_odom(self, msg: Odometry) -> None:
        self.local_odom = msg

    def _sample(self) -> None:
        now = time.monotonic()
        if self._last_sample_wall > 0.0 and now - self._last_sample_wall < self.sample_dt * 0.9:
            return
        self._last_sample_wall = now
        if not self.recording:
            return
        sample: dict[str, Any] = {
            "t": self.elapsed(),
            "wall_mono_s": now,
            "phase": self.current_phase,
            "mode": str(self.state.mode) if self.state is not None else "",
            "armed": bool(self.state.armed) if self.state is not None else False,
        }
        if self.imu is not None:
            q = self.imu.orientation
            roll, pitch, yaw = quat_to_rpy_rad(float(q.w), float(q.x), float(q.y), float(q.z))
            sample.update(
                {
                    "roll_rad": roll,
                    "pitch_rad": pitch,
                    "yaw_rad": yaw,
                    "gyro_x": float(self.imu.angular_velocity.x),
                    "gyro_y": float(self.imu.angular_velocity.y),
                    "gyro_z": float(self.imu.angular_velocity.z),
                    "acc_x": float(self.imu.linear_acceleration.x),
                    "acc_y": float(self.imu.linear_acceleration.y),
                    "acc_z": float(self.imu.linear_acceleration.z),
                }
            )
        if self.dvl_twist is not None:
            v = self.dvl_twist.twist.twist.linear
            sample.update({"dvl_vx": float(v.x), "dvl_vy": float(v.y), "dvl_vz": float(v.z)})
        if self.local_odom is not None:
            p = self.local_odom.pose.pose.position
            v = self.local_odom.twist.twist.linear
            sample.update(
                {
                    "odom_x": float(p.x),
                    "odom_y": float(p.y),
                    "odom_z": float(p.z),
                    "odom_vx": float(v.x),
                    "odom_vy": float(v.y),
                    "odom_vz": float(v.z),
                }
            )
        if self.depth is not None:
            sample["depth_m"] = float(self.depth.data)
        if self.rc_in is not None:
            channels = [int(v) for v in getattr(self.rc_in, "channels", [])]
            for idx, value in enumerate(channels[:8], start=1):
                sample[f"rcin{idx}"] = value
        if self.rc_out is not None:
            channels = [int(v) for v in getattr(self.rc_out, "channels", [])]
            for idx, value in enumerate(channels[:8], start=1):
                sample[f"rcout{idx}"] = value
        self.samples.append(sample)

    def publish_rc(self, axis: str | None = None, command: float = 0.0) -> None:
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        for idx in range(8):
            msg.channels[idx] = RC_NEUTRAL
        if axis:
            channel = AXIS_TO_CHANNEL[axis]
            value = RC_NEUTRAL + RC_SPAN * max(-1.0, min(1.0, float(command)))
            msg.channels[channel] = int(round(value))
        self._rc_released = False
        self.rc_pub.publish(msg)

    def publish_manual(self, axis: str | None = None, command: float = 0.0) -> None:
        msg = ManualControl()
        cmd = max(-1.0, min(1.0, float(command)))
        if axis == "forward":
            msg.x = cmd
        elif axis == "lateral":
            msg.y = cmd
        elif axis == "heave":
            msg.z = cmd
        elif axis == "yaw":
            msg.r = cmd
        # MAVLink v1 MANUAL_CONTROL has no roll/pitch extension fields in the
        # pymavlink path used by this bridge; those axes remain RC-override only.
        self.manual_pub.publish(msg)

    def release_rc(self) -> None:
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        for idx in range(8):
            msg.channels[idx] = OverrideRCIn.CHAN_RELEASE
        self.rc_pub.publish(msg)
        self._rc_released = True

    def publish_neutral_control(self) -> None:
        if self.input_mode == "manual-control":
            if not self._rc_released:
                self.release_rc()
            self.publish_manual()
        elif self.input_mode == "both":
            self.publish_rc()
            self.publish_manual()
        else:
            self.publish_rc()

    def spin_with_rc(
        self,
        duration: float,
        axis: str | None = None,
        command: float = 0.0,
        hz: float = 20.0,
        input_mode: str = "rc-override",
    ) -> None:
        end_t = time.monotonic() + float(duration)
        dt = 1.0 / max(1.0, float(hz))
        while time.monotonic() < end_t:
            if input_mode in ("rc-override", "both") or axis in ("roll", "pitch"):
                self.publish_rc(axis, command)
            elif input_mode == "manual-control" and not self._rc_released:
                self.release_rc()
            if input_mode in ("manual-control", "both"):
                self.publish_manual(axis, command)
            rclpy.spin_once(self, timeout_sec=min(0.03, dt))
            self._sample()
            remaining = end_t - time.monotonic()
            if remaining > 0:
                time.sleep(min(dt, remaining))

    def wait_for_stack(self, timeout: float) -> None:
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            services_ok = self.arm_client.wait_for_service(timeout_sec=0.1) and self.mode_client.wait_for_service(timeout_sec=0.1)
            state_ok = (
                self.state is not None
                and bool(self.state.connected)
                and bool(str(self.state.mode).strip())
            )
            if services_ok and state_ok:
                return
            self.publish_neutral_control()
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError("MAVROS-like surface did not become ready")

    def call_set_mode(self, mode: str, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        mode_sent = False
        while time.monotonic() < deadline:
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = str(mode)
            future = self.mode_client.call_async(req)
            request_deadline = min(deadline, time.monotonic() + 2.0)
            while time.monotonic() < request_deadline and not future.done():
                self.publish_neutral_control()
                rclpy.spin_once(self, timeout_sec=0.05)
            mode_sent = future.done() and future.result() is not None and bool(future.result().mode_sent)
            if not mode_sent:
                self.publish_neutral_control()
                rclpy.spin_once(self, timeout_sec=0.05)
                continue
            self.publish_neutral_control()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state is not None and str(self.state.mode) == mode:
                return
        reason = "service rejected request" if not mode_sent else "state did not report requested mode"
        raise RuntimeError(f"set_mode({mode}) failed: {reason}")

    def call_arm(self, armed: bool, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        accepted_any = False
        while time.monotonic() < deadline:
            if self.state is not None and bool(self.state.armed) == bool(armed):
                return
            if accepted_any:
                self.publish_neutral_control()
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.05)
                continue
            req = CommandBool.Request()
            req.value = bool(armed)
            future = self.arm_client.call_async(req)
            request_deadline = min(deadline, time.monotonic() + 2.0)
            while time.monotonic() < request_deadline and not future.done():
                self.publish_neutral_control()
                rclpy.spin_once(self, timeout_sec=0.05)
            accepted = future.done() and future.result() is not None and bool(future.result().success)
            accepted_any = accepted_any or accepted
            if not accepted_any:
                self.publish_neutral_control()
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
                continue
            self.publish_neutral_control()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state is not None and bool(self.state.armed) == bool(armed):
                return
        reason = "service rejected request" if not accepted_any else "state did not report requested arm state"
        raise RuntimeError(f"arming({armed}) failed: {reason}")

    def call_trigger_service(self, client: Any, service_name: str, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.1):
                break
            self.publish_neutral_control()
            rclpy.spin_once(self, timeout_sec=0.05)
        else:
            raise RuntimeError(f"{service_name} service did not become ready")

        future = client.call_async(Trigger.Request())
        while time.monotonic() < deadline and not future.done():
            self.publish_neutral_control()
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"{service_name} call timed out")
        result = future.result()
        if not bool(result.success):
            raise RuntimeError(f"{service_name} failed: {result.message}")

    def switch_initial_depth_hold_to_target(self, timeout: float = 10.0) -> None:
        self.call_trigger_service(
            self.switch_initial_depth_hold_client,
            "/mujoco/switch_initial_depth_hold_to_target",
            timeout=timeout,
        )

    def release_initial_depth_hold(self, timeout: float = 10.0) -> None:
        self.call_trigger_service(
            self.release_initial_depth_hold_client,
            "/mujoco/release_initial_depth_hold",
            timeout=timeout,
        )


def summarize(samples: list[dict[str, Any]], phases: list[Phase]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for phase in phases:
        part = [s for s in samples if s.get("phase") == phase.name]
        row: dict[str, Any] = {
            "phase": phase.name,
            "axis": phase.axis,
            "command": phase.command,
            "samples": len(part),
        }
        for key in ("gyro_x", "gyro_y", "gyro_z", "dvl_vx", "dvl_vy", "dvl_vz", "odom_vx", "odom_vy", "odom_vz"):
            values = [float(s.get(key, float("nan"))) for s in part]
            row[f"{key}_mean"] = mean(values)
            row[f"{key}_rms"] = rms(values)
            row[f"{key}_peak_abs"] = max((abs(v) for v in values if math.isfinite(v)), default=float("nan"))
        for key in ("roll_rad", "pitch_rad", "yaw_rad", "depth_m"):
            values = [float(s.get(key, float("nan"))) for s in part]
            finite = finite_values(values)
            row[f"{key}_mean"] = mean(values)
            row[f"{key}_span"] = (max(finite) - min(finite)) if finite else float("nan")
        for prefix in ("rcin", "rcout"):
            deltas = []
            for idx in range(1, 9):
                values = [float(s.get(f"{prefix}{idx}", float("nan"))) for s in part]
                deltas.extend(abs(v - RC_NEUTRAL) for v in finite_values(values))
            row[f"{prefix}_max_delta"] = max(deltas, default=float("nan"))
            row[f"{prefix}_mean_abs_delta"] = mean(deltas)
        expected_metric = EXPECTED_AXIS_METRIC.get(phase.axis)
        if expected_metric:
            row["expected_metric"] = expected_metric
            row["expected_metric_mean"] = row.get(f"{expected_metric}_mean", float("nan"))
            row["expected_metric_rms"] = row.get(f"{expected_metric}_rms", float("nan"))
            row["expected_metric_peak_abs"] = row.get(f"{expected_metric}_peak_abs", float("nan"))
        row["armed_fraction"] = mean([1.0 if s.get("armed") else 0.0 for s in part])
        rows.append(row)
    return rows


def build_health(
    summary: list[dict[str, Any]],
    *,
    input_mode: str,
    sample_hz: float,
    axis_s: float,
    neutral_s: float,
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    phase_by_name = {str(row["phase"]): row for row in summary}
    expected_axis_samples = max(1, int(0.55 * float(sample_hz) * float(axis_s)))
    expected_neutral_samples = max(1, int(0.45 * float(sample_hz) * float(neutral_s)))

    for row in summary:
        phase = str(row["phase"])
        axis = str(row["axis"])
        flags: list[str] = []
        severity = "ok"
        min_samples = expected_neutral_samples if axis == "neutral" else expected_axis_samples
        if int(row.get("samples", 0)) < min_samples:
            flags.append("too_few_samples")
            severity = "fail"
        if float(row.get("armed_fraction", 0.0)) < 0.99:
            flags.append("not_fully_armed")
            severity = "fail"
        if axis != "neutral" and input_mode in ("rc-override", "both"):
            if float(row.get("rcout_max_delta", 0.0)) < 5.0:
                flags.append("rcout_not_moving")
                severity = "fail"
        if axis != "neutral":
            threshold = MIN_EXPECTED_PEAK.get(axis, 0.0)
            peak = float(row.get("expected_metric_peak_abs", float("nan")))
            if math.isfinite(peak) and peak < threshold:
                flags.append("weak_primary_axis_response")
                if severity == "ok":
                    severity = "warn"
        else:
            # Yaw braking in ALT_HOLD can leave short residual motion, so this is
            # a warning for later tuning instead of a hard failure.
            if float(row.get("gyro_z_peak_abs", 0.0)) > 0.25:
                flags.append("high_neutral_yaw_residual")
                if severity == "ok":
                    severity = "warn"
            if abs(float(row.get("dvl_vz_mean", 0.0))) > 0.12:
                flags.append("high_neutral_heave_residual")
                if severity == "ok":
                    severity = "warn"
        checks.append(
            {
                "phase": phase,
                "axis": axis,
                "severity": severity,
                "flags": flags,
            }
        )

    for axis in AXIS_ORDER:
        pos = phase_by_name.get(f"{axis}_pos")
        neg = phase_by_name.get(f"{axis}_neg")
        metric = EXPECTED_AXIS_METRIC.get(axis)
        if not pos or not neg or not metric:
            continue
        pos_mean = float(pos.get("expected_metric_mean", float("nan")))
        neg_mean = float(neg.get("expected_metric_mean", float("nan")))
        if math.isfinite(pos_mean) and math.isfinite(neg_mean):
            if abs(pos_mean) > 1e-6 and abs(neg_mean) > 1e-6 and pos_mean * neg_mean > 0.0:
                checks.append(
                    {
                        "phase": f"{axis}_sign_pair",
                        "axis": axis,
                        "severity": "warn",
                        "flags": ["positive_negative_response_same_sign"],
                    }
                )

    if any(check["severity"] == "fail" for check in checks):
        overall = "fail"
    elif any(check["severity"] == "warn" for check in checks):
        overall = "warn"
    else:
        overall = "pass"
    return {"overall": overall, "checks": checks}


def write_outputs(
    out_dir: Path,
    samples: list[dict[str, Any]],
    phases: list[Phase],
    summary: list[dict[str, Any]],
    metadata: dict[str, Any],
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    all_keys: list[str] = []
    for sample in samples:
        for key in sample:
            if key not in all_keys:
                all_keys.append(key)
    with (out_dir / "axis_timeseries.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=all_keys)
        writer.writeheader()
        writer.writerows(samples)

    summary_keys: list[str] = []
    for row in summary:
        for key in row:
            if key not in summary_keys:
                summary_keys.append(key)
    with (out_dir / "axis_summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=summary_keys)
        writer.writeheader()
        writer.writerows(summary)

    payload = {
        "metadata": metadata,
        "phases": [phase.__dict__ for phase in phases],
        "summary": summary,
        "sample_count": len(samples),
        "health": build_health(
            summary,
            input_mode=str(metadata.get("input_mode", "")),
            sample_hz=float(metadata.get("sample_hz", 1.0)),
            axis_s=float(metadata.get("axis_s", 1.0)),
            neutral_s=float(metadata.get("neutral_s", 1.0)),
        ),
    }
    (out_dir / "axis_summary.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    plot_timeseries(out_dir / "axis_response.png", samples, phases)


def values(samples: list[dict[str, Any]], key: str) -> list[float]:
    return [float(sample.get(key, float("nan"))) for sample in samples]


def plot_timeseries(path: Path, samples: list[dict[str, Any]], phases: list[Phase]) -> None:
    if not samples:
        return
    t0 = float(samples[0]["t"])
    t = [float(s["t"]) - t0 for s in samples]
    dvl_vx = values(samples, "dvl_vx")
    dvl_vy = values(samples, "dvl_vy")
    dvl_vz = values(samples, "dvl_vz")
    dvl_speed = [
        math.sqrt(vx * vx + vy * vy + vz * vz)
        if math.isfinite(vx) and math.isfinite(vy) and math.isfinite(vz)
        else float("nan")
        for vx, vy, vz in zip(dvl_vx, dvl_vy, dvl_vz)
    ]

    fig, axes = plt.subplots(4, 1, figsize=(12, 8.5), sharex=True)

    axes[0].plot(t, values(samples, "depth_m"), label="Bar30 depth", lw=1.1)
    axes[0].plot(t, values(samples, "odom_z"), label="local z", lw=0.9, alpha=0.65)
    axes[0].set_ylabel("depth m")

    axes[1].plot(t, dvl_vx, label="vx", lw=0.9)
    axes[1].plot(t, dvl_vy, label="vy", lw=0.9)
    axes[1].plot(t, dvl_vz, label="vz", lw=0.9)
    axes[1].plot(t, dvl_speed, label="speed", lw=1.1, alpha=0.8)
    axes[1].set_ylabel("DVL m/s")

    axes[2].plot(t, values(samples, "gyro_x"), label="gyro x", lw=0.9)
    axes[2].plot(t, values(samples, "gyro_y"), label="gyro y", lw=0.9)
    axes[2].plot(t, values(samples, "gyro_z"), label="gyro z", lw=0.9)
    axes[2].set_ylabel("gyro rad/s")

    for idx in range(1, 9):
        rcin = values(samples, f"rcin{idx}")
        if any(math.isfinite(v) for v in rcin):
            axes[3].plot(t, rcin, label=f"ch{idx}", lw=0.8)
    axes[3].set_ylabel("RC pwm")

    for axis in axes:
        for phase in phases:
            if phase.axis != "neutral":
                axis.axvspan(phase.start - t0, phase.end - t0, color="0.2", alpha=0.035, lw=0)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right", ncol=4, fontsize=8)
    axes[-1].set_xlabel("time s")
    fig.suptitle(path.parent.name)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--command", type=float, default=0.25)
    parser.add_argument("--axis-s", type=float, default=5.0)
    parser.add_argument("--neutral-s", type=float, default=3.0)
    parser.add_argument("--baseline-s", type=float, default=5.0)
    parser.add_argument("--sample-hz", type=float, default=25.0)
    parser.add_argument("--mode", default="ALT_HOLD")
    parser.add_argument("--input-mode", choices=("rc-override", "manual-control", "both"), default="rc-override")
    parser.add_argument("--wait-timeout", type=float, default=90.0)
    parser.add_argument("--axes", nargs="+", choices=AXIS_ORDER, default=list(AXIS_ORDER))
    parser.add_argument("--pre-dive-s", type=float, default=0.0)
    parser.add_argument("--pre-dive-command", type=float, default=-0.35)
    parser.add_argument("--pre-settle-s", type=float, default=0.5)
    parser.add_argument("--switch-initial-depth-before-arm", action="store_true")
    parser.add_argument("--release-initial-depth-hold", action="store_true")
    parser.add_argument("--post-release-neutral-s", type=float, default=0.5)
    parser.add_argument(
        "--neutral-only",
        action="store_true",
        help="Record only the baseline neutral hold phase; useful for AltHold drift checks.",
    )
    parser.add_argument("--disarm-at-end", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir or LOG_ROOT / f"axis_rc_override_check_{time.strftime('%Y%m%d_%H%M%S')}"
    rclpy.init()
    node = AxisRcOverrideCheck(sample_hz=args.sample_hz, input_mode=args.input_mode)
    try:
        node.wait_for_stack(timeout=args.wait_timeout)
        node.spin_with_rc(1.0, input_mode=args.input_mode)
        node.call_set_mode("MANUAL", timeout=args.wait_timeout)
        node.spin_with_rc(1.0, input_mode=args.input_mode)
        if args.switch_initial_depth_before_arm:
            node.switch_initial_depth_hold_to_target(timeout=args.wait_timeout)
            node.spin_with_rc(1.0, input_mode=args.input_mode)
        node.call_arm(True, timeout=args.wait_timeout)
        node.spin_with_rc(2.0, input_mode=args.input_mode)
        if args.mode:
            node.call_set_mode(args.mode, timeout=args.wait_timeout)
        node.spin_with_rc(2.0, input_mode=args.input_mode)
        if args.release_initial_depth_hold:
            node.release_initial_depth_hold(timeout=args.wait_timeout)
            if args.post_release_neutral_s > 0.0:
                node.spin_with_rc(args.post_release_neutral_s, input_mode=args.input_mode)
        if args.pre_dive_s > 0.0:
            node.spin_with_rc(
                args.pre_dive_s,
                axis="heave",
                command=args.pre_dive_command,
                input_mode=args.input_mode,
            )
            if args.pre_settle_s > 0.0:
                node.spin_with_rc(args.pre_settle_s, input_mode=args.input_mode)

        if args.input_mode == "manual-control":
            node.release_rc()
            node.spin_with_rc(0.5, input_mode=args.input_mode)

        node.recording = True
        start = node.elapsed()
        node.current_phase = "baseline_neutral"
        node.phases.append(Phase("baseline_neutral", "neutral", 0.0, start, start + args.baseline_s))
        node.spin_with_rc(args.baseline_s, input_mode=args.input_mode)

        if not args.neutral_only and abs(float(args.command)) > 1e-9:
            for axis in args.axes:
                for command in (abs(args.command), -abs(args.command)):
                    phase_start = node.elapsed()
                    phase_name = f"{axis}_{'pos' if command > 0 else 'neg'}"
                    node.current_phase = phase_name
                    node.phases.append(Phase(phase_name, axis, command, phase_start, phase_start + args.axis_s))
                    node.spin_with_rc(args.axis_s, axis=axis, command=command, input_mode=args.input_mode)

                    neutral_start = node.elapsed()
                    neutral_name = f"neutral_after_{phase_name}"
                    node.current_phase = neutral_name
                    node.phases.append(Phase(neutral_name, "neutral", 0.0, neutral_start, neutral_start + args.neutral_s))
                    node.spin_with_rc(args.neutral_s, input_mode=args.input_mode)
        elif not args.neutral_only:
            # A zero command is a neutral-hold check, not a positive/negative
            # axis response. Keep the report unambiguous.
            args.neutral_only = True

        if args.neutral_only and args.neutral_s > 0.0:
            neutral_start = node.elapsed()
            node.current_phase = "neutral_hold"
            node.phases.append(Phase("neutral_hold", "neutral", 0.0, neutral_start, neutral_start + args.neutral_s))
            node.spin_with_rc(args.neutral_s, input_mode=args.input_mode)

        node.recording = False
        node.spin_with_rc(1.0, input_mode=args.input_mode)
        if args.disarm_at_end:
            node.call_arm(False, timeout=args.wait_timeout)
        node.release_rc()

        summary = summarize(node.samples, node.phases)
        metadata = {
            "mode": args.mode,
            "input_mode": args.input_mode,
            "command": args.command,
            "axis_s": args.axis_s,
            "neutral_s": args.neutral_s,
            "baseline_s": args.baseline_s,
            "pre_dive_s": args.pre_dive_s,
            "pre_dive_command": args.pre_dive_command,
            "pre_settle_s": args.pre_settle_s,
            "release_initial_depth_hold": bool(args.release_initial_depth_hold),
            "post_release_neutral_s": args.post_release_neutral_s,
            "neutral_only": bool(args.neutral_only),
            "sample_hz": args.sample_hz,
            "axes": args.axes,
            "rc_neutral": RC_NEUTRAL,
            "rc_span": RC_SPAN,
            "node_start_wall_mono_s": node.start_wall,
        }
        write_outputs(out_dir, node.samples, node.phases, summary, metadata)
        health = build_health(
            summary,
            input_mode=args.input_mode,
            sample_hz=args.sample_hz,
            axis_s=args.axis_s,
            neutral_s=args.neutral_s,
        )
        print(f"[axis-check] out={out_dir}")
        print(f"[axis-check] samples={len(node.samples)} phases={len(node.phases)}")
        print(f"[axis-check] health={health['overall']}")
        for row in summary:
            if row.get("axis") == "neutral":
                continue
            print(
                "[axis-check] "
                f"{row['phase']}: gyro_peak=({row['gyro_x_peak_abs']:.3f},"
                f"{row['gyro_y_peak_abs']:.3f},{row['gyro_z_peak_abs']:.3f}) "
                f"dvl_mean=({row['dvl_vx_mean']:.3f},{row['dvl_vy_mean']:.3f},{row['dvl_vz_mean']:.3f}) "
                f"depth_span={row['depth_m_span']:.3f}"
            )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
