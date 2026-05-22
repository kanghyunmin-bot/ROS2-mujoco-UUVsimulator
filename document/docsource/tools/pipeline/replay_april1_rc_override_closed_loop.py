from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import rosbag2_py
from mavros_msgs.msg import OverrideRCIn, RCOut, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, String


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_april1_real_bags import read_bag, read_bag_metadata  # noqa: E402


DEFAULT_BAG = Path(
    "real_robot_ros_bag/extracted_2026_04_01/"
    "bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_0.db3"
)
DEFAULT_OUT = Path("document/docsource/runs/closed_loop/closed_loop_april1_rc_replay")
PWM_CENTER = 1500.0
RC_OVERRIDE_SPAN = 300.0


@dataclass
class RcSequence:
    bag_name: str
    topic: str
    origin_s: float
    t: np.ndarray
    channels: np.ndarray


@dataclass
class ModeSequence:
    topic: str
    events: list[tuple[float, str]]


def axis_column(axes: np.ndarray, index: int) -> np.ndarray:
    if axes.size == 0:
        return np.empty(0, dtype=float)
    return axes[:, index] if index < axes.shape[1] else np.zeros(axes.shape[0], dtype=float)


def button_column(buttons: np.ndarray, index: int) -> np.ndarray:
    if buttons.size == 0:
        return np.empty(0, dtype=float)
    return buttons[:, index] if index < buttons.shape[1] else np.zeros(buttons.shape[0], dtype=float)


def joy_node_rc_override(joy_t: np.ndarray, joy_axes: np.ndarray, joy_buttons: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rebuild the kmu26_auv joy2mavros RC override stream from recorded /joy."""
    if joy_t.size == 0 or joy_axes.size == 0:
        return np.empty(0, dtype=float), np.empty((0, 18), dtype=float)
    axes = np.asarray(joy_axes, dtype=float)
    if axes.ndim == 1:
        axes = axes.reshape((-1, 1))
    count = min(joy_t.size, axes.shape[0])
    if count <= 1:
        return np.empty(0, dtype=float), np.empty((0, 18), dtype=float)
    axes = axes[:count]
    buttons = np.asarray(joy_buttons, dtype=float)
    if buttons.size:
        if buttons.ndim == 1:
            buttons = buttons.reshape((-1, 1))
        buttons = buttons[: min(count, buttons.shape[0])]

    channels = np.full((count, 18), PWM_CENTER, dtype=float)
    # joy2mavros.cpp: ch3=heave, ch4=yaw, ch5=forward, ch6=lateral.
    channels[:, 3] = PWM_CENTER + (-axis_column(axes, 2)) * RC_OVERRIDE_SPAN
    channels[:, 2] = PWM_CENTER + axis_column(axes, 3) * RC_OVERRIDE_SPAN
    channels[:, 5] = PWM_CENTER + (-axis_column(axes, 0)) * RC_OVERRIDE_SPAN
    channels[:, 4] = PWM_CENTER + axis_column(axes, 1) * RC_OVERRIDE_SPAN

    led_pwm = PWM_CENTER
    channels[:, 8] = led_pwm
    if buttons.size and buttons.shape[0] >= count:
        b5 = button_column(buttons, 5)
        b6 = button_column(buttons, 6)
        for i in range(1, count):
            if b6[i] == 1.0 and b6[i - 1] != 1.0:
                led_pwm = max(1100.0, led_pwm - 100.0) if b5[i] == 1.0 else min(1800.0, led_pwm + 100.0)
            channels[i, 8] = led_pwm

    # The real C++ node stores the first /joy sample and starts publishing on the
    # next callback.
    return np.asarray(joy_t[:count], dtype=float)[1:], channels[1:]


def load_rc_sequence(
    db_path: Path,
    *,
    topic: str,
    start_offset_s: float,
    duration_s: float | None,
    max_publish_hz: float,
    fill_neutral_first8: bool,
) -> RcSequence:
    data = read_bag(db_path)
    t, channels = data.array(f"{topic}:channels")
    if channels.size == 0:
        raise RuntimeError(f"No RC override samples found at {topic} in {db_path}")
    channels = np.asarray(channels, dtype=float)
    if channels.ndim == 1:
        channels = channels.reshape((-1, 1))
    if channels.shape[1] < 18:
        padded = np.zeros((channels.shape[0], 18), dtype=float)
        padded[:, : channels.shape[1]] = channels
        channels = padded
    else:
        channels = channels[:, :18]

    t = np.asarray(t, dtype=float).reshape(-1)
    if t.size != channels.shape[0]:
        n = min(t.size, channels.shape[0])
        t = t[:n]
        channels = channels[:n]
    mask = t >= float(start_offset_s)
    if duration_s is not None:
        mask &= t <= float(start_offset_s) + float(duration_s)
    t = t[mask]
    channels = channels[mask]
    if t.size == 0:
        raise RuntimeError("No RC samples remain after start/duration crop")

    origin_s = float(t[0])
    t = t - origin_s
    if fill_neutral_first8:
        first8 = channels[:, :8]
        first8[(first8 <= 0.0) | (first8 == 65535.0)] = 1500.0
        channels[:, :8] = first8

    if max_publish_hz > 0.0 and t.size > 2:
        min_dt = 1.0 / float(max_publish_hz)
        keep = np.zeros(t.shape, dtype=bool)
        keep[0] = True
        last_t = float(t[0])
        for idx in range(1, t.size - 1):
            if float(t[idx]) - last_t >= min_dt:
                keep[idx] = True
                last_t = float(t[idx])
        keep[-1] = True
        t = t[keep]
        channels = channels[keep]

    channels = np.nan_to_num(channels, nan=0.0, posinf=0.0, neginf=0.0)
    channels = np.clip(np.rint(channels), 0, 65535).astype(np.uint16)
    return RcSequence(data.name, topic, origin_s, t, channels)


def load_joy_node_sequence(
    db_path: Path,
    *,
    start_offset_s: float,
    duration_s: float | None,
    max_publish_hz: float,
) -> RcSequence:
    data = read_bag(db_path)
    joy_t, joy_axes = data.array("/joy:axes")
    _, joy_buttons = data.array("/joy:buttons")
    t, channels = joy_node_rc_override(joy_t, joy_axes, joy_buttons)
    if channels.size == 0:
        raise RuntimeError(f"No /joy samples could be converted to joy-node RC override in {db_path}")
    channels = np.asarray(channels, dtype=float)
    if channels.shape[1] < 18:
        padded = np.full((channels.shape[0], 18), PWM_CENTER, dtype=float)
        padded[:, : channels.shape[1]] = channels
        channels = padded
    else:
        channels = channels[:, :18]

    t = np.asarray(t, dtype=float).reshape(-1)
    if t.size != channels.shape[0]:
        n = min(t.size, channels.shape[0])
        t = t[:n]
        channels = channels[:n]
    mask = t >= float(start_offset_s)
    if duration_s is not None:
        mask &= t <= float(start_offset_s) + float(duration_s)
    t = t[mask]
    channels = channels[mask]
    if t.size == 0:
        raise RuntimeError("No joy-node RC samples remain after start/duration crop")

    origin_s = float(t[0])
    t = t - origin_s
    if max_publish_hz > 0.0 and t.size > 2:
        min_dt = 1.0 / float(max_publish_hz)
        keep = np.zeros(t.shape, dtype=bool)
        keep[0] = True
        last_t = float(t[0])
        for idx in range(1, t.size - 1):
            if float(t[idx]) - last_t >= min_dt:
                keep[idx] = True
                last_t = float(t[idx])
        keep[-1] = True
        t = t[keep]
        channels = channels[keep]

    channels = np.nan_to_num(channels, nan=PWM_CENTER, posinf=PWM_CENTER, neginf=PWM_CENTER)
    channels = np.clip(np.rint(channels), 0, 65535).astype(np.uint16)
    return RcSequence(data.name, "joy-node:/joy", origin_s, t, channels)


def load_mode_sequence(
    db_path: Path,
    *,
    topic: str,
    rc_origin_s: float,
    duration_s: float | None,
) -> ModeSequence:
    db_path = Path(db_path)
    if db_path.is_dir():
        candidates = sorted(db_path.glob("*.db3")) or sorted(db_path.glob("**/*.db3"))
        if not candidates:
            return ModeSequence(topic, [])
        db_file = candidates[0]
        bag_uri = db_file.parent
    else:
        db_file = db_path
        bag_uri = db_path.parent
    topics, _, t0_ns, _ = read_bag_metadata(db_file)
    if topic not in topics:
        return ModeSequence(topic, [])

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    samples: list[tuple[float, str]] = []
    while reader.has_next():
        msg_topic, raw, stamp_ns = reader.read_next()
        if msg_topic != topic:
            continue
        msg = deserialize_message(raw, State)
        mode = str(getattr(msg, "mode", "")).strip()
        if not mode:
            continue
        t_abs = (int(stamp_ns) - int(t0_ns)) * 1.0e-9
        samples.append((t_abs, mode))
    if not samples:
        return ModeSequence(topic, [])

    start_abs = float(rc_origin_s)
    end_abs = float("inf") if duration_s is None else start_abs + float(duration_s)
    current_mode = samples[0][1]
    for t_abs, mode in samples:
        if t_abs <= start_abs + 1.0e-9:
            current_mode = mode
        else:
            break

    events: list[tuple[float, str]] = [(0.0, current_mode)]
    prev_mode = current_mode
    for t_abs, mode in samples:
        if t_abs < start_abs - 1.0e-9:
            continue
        if t_abs > end_abs + 1.0e-9:
            break
        if mode == prev_mode:
            continue
        events.append((max(0.0, t_abs - start_abs), mode))
        prev_mode = mode
    return ModeSequence(topic, events)


def mode_at_time(mode_sequence: ModeSequence | None, t_s: float, fallback: str) -> str:
    if mode_sequence is None or not mode_sequence.events:
        return fallback
    mode = fallback
    for event_t, event_mode in mode_sequence.events:
        if event_t <= t_s + 1.0e-9:
            mode = event_mode
        else:
            break
    return mode


class ClosedLoopRcReplayer(Node):
    def __init__(self, output_dir: Path) -> None:
        super().__init__("april1_closed_loop_rc_replayer")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.rcout_override_pub = self.create_publisher(RCOut, "/uuv_mujoco/rc/out_override", 10)
        self.phase_pub = self.create_publisher(String, "/measurement/phase", 10)
        self.state_sub = self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.rc_out_sub = self.create_subscription(RCOut, "/mavros/rc/out", self._on_rc_out, 10)
        self.depth_sub = self.create_subscription(Float32, "/depth", self._on_depth, 10)
        self.local_odom_sub = self.create_subscription(Odometry, "/mavros/local_position/odom", self._on_local_odom, 10)
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.release_initial_depth_hold_client = self.create_client(
            Trigger,
            "/mujoco/release_initial_depth_hold",
        )
        self.switch_initial_depth_hold_target_client = self.create_client(
            Trigger,
            "/mujoco/switch_initial_depth_hold_to_target",
        )
        self.latest_state: State | None = None
        self.latest_rc_out: RCOut | None = None
        self.latest_rc_out_wall = 0.0
        self.rc_out_sample_count = 0
        self.latest_depth: Float32 | None = None
        self.latest_depth_wall = 0.0
        self.latest_local_odom: Odometry | None = None
        self.latest_local_odom_wall = 0.0
        self.events: list[dict[str, Any]] = []
        self.t0_wall = time.monotonic()

    def elapsed(self) -> float:
        return time.monotonic() - self.t0_wall

    def _on_state(self, msg: State) -> None:
        prev = self.latest_state
        self.latest_state = msg
        if prev is None or prev.connected != msg.connected or prev.armed != msg.armed or prev.mode != msg.mode:
            self.events.append(
                {
                    "t": self.elapsed(),
                    "type": "state",
                    "connected": bool(msg.connected),
                    "armed": bool(msg.armed),
                    "mode": str(msg.mode),
                }
            )

    def _on_rc_out(self, msg: RCOut) -> None:
        self.latest_rc_out = msg
        self.latest_rc_out_wall = time.monotonic()
        self.rc_out_sample_count += 1
        if self.rc_out_sample_count == 1:
            channels = [int(v) for v in list(getattr(msg, "channels", []))[:8]]
            self.events.append(
                {
                    "t": self.elapsed(),
                    "type": "rc_out_first",
                    "channels": channels,
                }
            )

    def _on_depth(self, msg: Float32) -> None:
        self.latest_depth = msg
        self.latest_depth_wall = time.monotonic()

    def _on_local_odom(self, msg: Odometry) -> None:
        self.latest_local_odom = msg
        self.latest_local_odom_wall = time.monotonic()

    def snapshot(self, label: str, spin_s: float = 0.0) -> None:
        deadline = time.monotonic() + max(float(spin_s), 0.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())))
        now = time.monotonic()

        state = self.latest_state
        rc_out = self.latest_rc_out
        odom = self.latest_local_odom
        event: dict[str, Any] = {
            "t": self.elapsed(),
            "wall_mono_s": float(now),
            "type": "snapshot",
            "label": str(label),
            "state": None
            if state is None
            else {
                "connected": bool(state.connected),
                "armed": bool(state.armed),
                "mode": str(state.mode),
            },
            "rc_out": None
            if rc_out is None
            else {
                "sample_count": int(self.rc_out_sample_count),
                "age_s": float(now - self.latest_rc_out_wall),
                "channels": [int(v) for v in list(getattr(rc_out, "channels", []))[:8]],
            },
            "depth_m": None if self.latest_depth is None else float(self.latest_depth.data),
            "depth_age_s": None if self.latest_depth is None else float(now - self.latest_depth_wall),
            "local_odom": None,
        }
        if odom is not None:
            pose = odom.pose.pose.position
            twist = odom.twist.twist.linear
            event["local_odom"] = {
                "age_s": float(now - self.latest_local_odom_wall),
                "position_m": [float(pose.x), float(pose.y), float(pose.z)],
                "linear_m_s": [float(twist.x), float(twist.y), float(twist.z)],
            }
        self.events.append(event)

    def wait_for_connected_state(self, timeout_s: float) -> None:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_state is not None and bool(self.latest_state.connected):
                return
        raise RuntimeError("No connected /mavros/state observed before timeout")

    def wait_for_state_match(
        self,
        *,
        mode: str | None = None,
        armed: bool | None = None,
        timeout_s: float = 5.0,
        label: str = "state_match",
    ) -> None:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            state = self.latest_state
            if state is None:
                continue
            mode_ok = mode is None or str(state.mode) == str(mode)
            armed_ok = armed is None or bool(state.armed) == bool(armed)
            if mode_ok and armed_ok:
                self.events.append(
                    {
                        "t": self.elapsed(),
                        "type": label,
                        "ok": True,
                        "mode": str(state.mode),
                        "armed": bool(state.armed),
                    }
                )
                return
        state = self.latest_state
        self.events.append(
            {
                "t": self.elapsed(),
                "type": label,
                "ok": False,
                "expected_mode": mode,
                "expected_armed": armed,
                "mode": str(state.mode) if state is not None else "",
                "armed": bool(state.armed) if state is not None else None,
            }
        )
        raise RuntimeError(
            f"State confirmation timeout for mode={mode!r}, armed={armed!r}; "
            f"latest mode={str(state.mode) if state is not None else None!r}, "
            f"armed={bool(state.armed) if state is not None else None!r}"
        )

    def wait_for_rc_out_stream(
        self,
        *,
        timeout_s: float = 8.0,
        min_samples: int = 5,
        max_age_s: float = 0.5,
    ) -> None:
        if timeout_s <= 0.0:
            return
        start_count = self.rc_out_sample_count
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            got_enough = self.rc_out_sample_count - start_count >= int(min_samples)
            fresh = self.latest_rc_out is not None and (now - self.latest_rc_out_wall) <= float(max_age_s)
            if got_enough and fresh:
                channels = [int(v) for v in list(getattr(self.latest_rc_out, "channels", []))[:8]]
                self.events.append(
                    {
                        "t": self.elapsed(),
                        "type": "rc_out_stream_ready",
                        "samples": int(self.rc_out_sample_count - start_count),
                        "channels": channels,
                    }
                )
                return
        self.events.append(
            {
                "t": self.elapsed(),
                "type": "rc_out_stream_ready",
                "ok": False,
                "samples": int(self.rc_out_sample_count - start_count),
            }
        )
        raise RuntimeError("No stable /mavros/rc/out stream observed before replay start")

    def wait_for_services(self, timeout_s: float) -> None:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if self.arm_client.wait_for_service(timeout_sec=0.1) and self.mode_client.wait_for_service(timeout_sec=0.1):
                return
        raise RuntimeError("MAVROS-compatible arming/mode services did not become available")

    def set_mode(self, mode: str, timeout_s: float = 5.0) -> None:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = str(mode)
        future = self.mode_client.call_async(req)
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        resp = future.result() if future.done() else None
        ok = bool(resp is not None and resp.mode_sent)
        self.events.append({"t": self.elapsed(), "type": "set_mode", "mode": mode, "ok": ok})
        if not ok:
            raise RuntimeError(f"set_mode({mode}) failed")

    def arm(self, value: bool, timeout_s: float = 5.0) -> None:
        req = CommandBool.Request()
        req.value = bool(value)
        future = self.arm_client.call_async(req)
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        resp = future.result() if future.done() else None
        ok = bool(resp is not None and resp.success)
        self.events.append({"t": self.elapsed(), "type": "arm", "value": bool(value), "ok": ok})
        if not ok:
            raise RuntimeError(f"arming({value}) failed")

    def release_initial_depth_hold(self, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if self.release_initial_depth_hold_client.wait_for_service(timeout_sec=0.1):
                break
            rclpy.spin_once(self, timeout_sec=0.0)
        else:
            raise RuntimeError("/mujoco/release_initial_depth_hold service not available")

        future = self.release_initial_depth_hold_client.call_async(Trigger.Request())
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        resp = future.result() if future.done() else None
        ok = bool(resp is not None and resp.success)
        self.events.append(
            {
                "t": self.elapsed(),
                "type": "release_initial_depth_hold",
                "ok": ok,
                "message": str(getattr(resp, "message", "")) if resp is not None else "",
            }
        )
        if not ok:
            raise RuntimeError("release_initial_depth_hold failed")

    def switch_initial_depth_hold_to_target(self, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if self.switch_initial_depth_hold_target_client.wait_for_service(timeout_sec=0.1):
                break
            rclpy.spin_once(self, timeout_sec=0.0)
        else:
            raise RuntimeError("/mujoco/switch_initial_depth_hold_to_target service not available")

        future = self.switch_initial_depth_hold_target_client.call_async(Trigger.Request())
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        resp = future.result() if future.done() else None
        ok = bool(resp is not None and resp.success)
        self.events.append(
            {
                "t": self.elapsed(),
                "type": "switch_initial_depth_hold_to_target",
                "ok": ok,
                "message": str(getattr(resp, "message", "")) if resp is not None else "",
            }
        )
        if not ok:
            raise RuntimeError("switch_initial_depth_hold_to_target failed")

    def publish_phase(self, name: str) -> None:
        msg = String()
        msg.data = str(name)
        self.phase_pub.publish(msg)

    def publish_channels(self, channels: np.ndarray) -> None:
        msg = OverrideRCIn()
        values = [int(v) for v in channels[:18]]
        if len(values) < 18:
            values.extend([0] * (18 - len(values)))
        try:
            msg.channels = values
        except Exception:
            for idx, value in enumerate(values):
                msg.channels[idx] = value
        self.rc_pub.publish(msg)

    def publish_rcout_override(self, channels: np.ndarray) -> None:
        msg = RCOut()
        header = getattr(msg, "header", None)
        if header is not None:
            header.stamp = self.get_clock().now().to_msg()
            if hasattr(header, "frame_id"):
                header.frame_id = "fcu"
        values = [int(v) for v in channels[:18]]
        if len(values) < 18:
            values.extend([0] * (18 - len(values)))
        try:
            msg.channels = values
        except Exception:
            for idx, value in enumerate(values):
                msg.channels[idx] = value
        self.rcout_override_pub.publish(msg)

    def hold_neutral(self, duration_s: float, hz: float = 20.0) -> None:
        neutral = np.zeros(18, dtype=np.uint16)
        neutral[:8] = 1500
        dt = 1.0 / max(float(hz), 1.0)
        end_wall = time.monotonic() + float(duration_s)
        while time.monotonic() < end_wall:
            self.publish_phase("neutral")
            self.publish_channels(neutral)
            rclpy.spin_once(self, timeout_sec=min(dt, 0.05))
            remaining = end_wall - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(dt, remaining))

    def hold_axes(
        self,
        duration_s: float,
        *,
        forward: float = 0.0,
        lateral: float = 0.0,
        heave: float = 0.0,
        yaw: float = 0.0,
        hz: float = 50.0,
        phase: str = "axis_hold",
    ) -> None:
        channels = np.zeros(18, dtype=np.uint16)
        channels[:8] = 1500

        def axis_to_pwm(value: float) -> int:
            return int(round(1500.0 + 300.0 * float(np.clip(value, -1.0, 1.0))))

        channels[2] = axis_to_pwm(heave)
        channels[3] = axis_to_pwm(yaw)
        channels[4] = axis_to_pwm(forward)
        channels[5] = axis_to_pwm(lateral)
        dt = 1.0 / max(float(hz), 1.0)
        end_wall = time.monotonic() + max(float(duration_s), 0.0)
        self.events.append(
            {
                "t": self.elapsed(),
                "type": "axis_hold",
                "phase": phase,
                "duration_s": float(duration_s),
                "forward": float(forward),
                "lateral": float(lateral),
                "heave": float(heave),
                "yaw": float(yaw),
                "channels": [int(v) for v in channels[:8]],
            }
        )
        while time.monotonic() < end_wall:
            self.publish_phase(phase)
            self.publish_channels(channels)
            rclpy.spin_once(self, timeout_sec=min(dt, 0.02))
            remaining = end_wall - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(dt, remaining))

    def replay(
        self,
        seq: RcSequence,
        *,
        rate_scale: float,
        mode_sequence: ModeSequence | None = None,
        output_kind: str = "rc-override",
    ) -> None:
        rate_scale = max(float(rate_scale), 1e-6)
        replay_start = time.monotonic()
        next_mode_idx = 0
        self.events.append(
            {
                "t": self.elapsed(),
                "wall_mono_s": float(time.monotonic()),
                "type": "replay_start",
                "bag": seq.bag_name,
                "topic": seq.topic,
                "samples": int(seq.t.size),
                "output_kind": str(output_kind),
                "rate_scale": rate_scale,
                "mode_events": [
                    {"t": float(t), "mode": str(mode)}
                    for t, mode in (mode_sequence.events if mode_sequence is not None else [])
                ],
            }
        )
        for sample_t, channels in zip(seq.t, seq.channels):
            target_wall = replay_start + float(sample_t) / rate_scale
            while True:
                now = time.monotonic()
                if now >= target_wall:
                    break
                rclpy.spin_once(self, timeout_sec=min(0.02, max(0.0, target_wall - now)))
            if mode_sequence is not None:
                while next_mode_idx < len(mode_sequence.events):
                    event_t, event_mode = mode_sequence.events[next_mode_idx]
                    if float(event_t) > float(sample_t) + 1.0e-9:
                        break
                    if self.latest_state is None or str(self.latest_state.mode) != str(event_mode):
                        self.set_mode(str(event_mode), timeout_s=5.0)
                    next_mode_idx += 1
            self.publish_phase("rcout_replay" if output_kind == "rc-out" else "closed_loop_replay")
            if output_kind == "rc-out":
                self.publish_rcout_override(channels)
            else:
                self.publish_channels(channels)
            rclpy.spin_once(self, timeout_sec=0.0)
        self.events.append({"t": self.elapsed(), "wall_mono_s": float(time.monotonic()), "type": "replay_end"})


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Replay real /mavros/rc/override samples into the ROS2/MuJoCo/SITL stack. "
            "For pure ArduSub controller-in-the-loop tests, launch MuJoCo with "
            "ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK=0."
        )
    )
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--topic", default="/mavros/rc/override")
    parser.add_argument(
        "--command-source",
        choices=("rc-override", "joy-node", "rc-out"),
        default="rc-override",
        help=(
            "Replay recorded /mavros/rc/override, rebuild it from /joy like the "
            "real joy node, or inject recorded /mavros/rc/out into MuJoCo for "
            "RCOUT-based plant identification."
        ),
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--start-offset-s", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--rate-scale", type=float, default=1.0)
    parser.add_argument("--max-publish-hz", type=float, default=50.0)
    parser.add_argument("--pre-neutral-s", type=float, default=2.0)
    parser.add_argument("--post-neutral-s", type=float, default=2.0)
    parser.add_argument("--mode", default="MANUAL")
    parser.add_argument("--mode-from-bag", action="store_true")
    parser.add_argument("--mode-topic", default="/mavros/state")
    parser.add_argument("--skip-initial-mode-event", action="store_true")
    parser.add_argument("--force-initial-mode", default="")
    parser.add_argument("--initial-mode-settle-s", type=float, default=2.0)
    parser.add_argument("--no-mode", action="store_true")
    parser.add_argument("--arm", action="store_true", default=True)
    parser.add_argument("--no-arm", action="store_false", dest="arm")
    parser.add_argument("--wait-connected-s", type=float, default=30.0)
    parser.add_argument("--service-timeout-s", type=float, default=20.0)
    parser.add_argument(
        "--wait-rc-out-s",
        type=float,
        default=8.0,
        help="Require a fresh /mavros/rc/out stream before releasing hold/replay. Set 0 to disable.",
    )
    parser.add_argument("--switch-sim-initial-depth-hold-to-target", action="store_true")
    parser.add_argument("--post-depth-switch-neutral-s", type=float, default=1.0)
    parser.add_argument("--release-sim-initial-depth-hold", action="store_true")
    parser.add_argument(
        "--release-before-force-initial-mode",
        action="store_true",
        help="Release the artificial MuJoCo hold before forcing ALT_HOLD so the controller does not wind up against a locked body.",
    )
    parser.add_argument(
        "--post-release-neutral-s",
        type=float,
        default=1.0,
        help="Neutral settle time after releasing the artificial MuJoCo initial-depth hold and before replay starts.",
    )
    parser.add_argument("--fill-neutral-first8", action="store_true")
    parser.add_argument("--pre-kick-duration-s", type=float, default=0.0)
    parser.add_argument("--pre-kick-forward", type=float, default=0.0)
    parser.add_argument("--pre-kick-lateral", type=float, default=0.0)
    parser.add_argument("--pre-kick-heave", type=float, default=0.0)
    parser.add_argument("--pre-kick-yaw", type=float, default=0.0)
    parser.add_argument("--post-pre-kick-neutral-s", type=float, default=0.1)
    args = parser.parse_args()

    output_kind = "rc-out" if args.command_source == "rc-out" else "rc-override"
    if args.command_source == "joy-node":
        seq = load_joy_node_sequence(
            args.bag,
            start_offset_s=args.start_offset_s,
            duration_s=args.duration_s,
            max_publish_hz=args.max_publish_hz,
        )
    else:
        topic = "/mavros/rc/out" if args.command_source == "rc-out" and args.topic == "/mavros/rc/override" else args.topic
        seq = load_rc_sequence(
            args.bag,
            topic=topic,
            start_offset_s=args.start_offset_s,
            duration_s=args.duration_s,
            max_publish_hz=args.max_publish_hz,
            fill_neutral_first8=args.fill_neutral_first8 or args.command_source == "rc-out",
        )
    mode_sequence = (
        load_mode_sequence(
            args.bag,
            topic=args.mode_topic,
            rc_origin_s=seq.origin_s,
            duration_s=args.duration_s,
        )
        if args.mode_from_bag
        else None
    )
    if mode_sequence is not None and args.skip_initial_mode_event:
        mode_sequence.events = [(t, mode) for t, mode in mode_sequence.events if float(t) > 1.0e-6]

    rclpy.init()
    node = ClosedLoopRcReplayer(args.output_dir)
    try:
        node.wait_for_connected_state(args.wait_connected_s)
        if not args.no_mode or args.arm:
            node.wait_for_services(args.service_timeout_s)
        node.hold_neutral(args.pre_neutral_s)
        node.snapshot("after_pre_neutral")
        if args.switch_sim_initial_depth_hold_to_target:
            node.switch_initial_depth_hold_to_target(args.service_timeout_s)
            node.hold_neutral(args.post_depth_switch_neutral_s)
            node.snapshot("after_initial_depth_target_switch")
        if not args.no_mode and args.mode and not args.mode_from_bag:
            node.set_mode(args.mode)
            node.wait_for_state_match(mode=args.mode, timeout_s=args.service_timeout_s, label="fixed_mode_confirmed")
            node.hold_neutral(1.0)
            node.snapshot("after_fixed_mode")
        if args.arm:
            node.arm(True)
            node.wait_for_state_match(armed=True, timeout_s=args.service_timeout_s, label="armed_confirmed")
            node.hold_neutral(2.0)
            node.wait_for_rc_out_stream(timeout_s=args.wait_rc_out_s)
            node.snapshot("after_arm_rc_out_ready")
        released_initial_hold = False
        if (
            args.release_sim_initial_depth_hold
            and args.release_before_force_initial_mode
            and args.force_initial_mode
        ):
            node.release_initial_depth_hold(args.service_timeout_s)
            released_initial_hold = True
            if args.post_release_neutral_s > 0.0:
                node.hold_neutral(float(args.post_release_neutral_s))
            node.snapshot("after_release_before_force_mode")
        if (
            not args.force_initial_mode
            and not args.no_mode
            and args.mode_from_bag
            and mode_sequence is not None
            and mode_sequence.events
        ):
            initial_mode = mode_at_time(mode_sequence, 0.0, args.mode)
            if initial_mode and (node.latest_state is None or str(node.latest_state.mode) != str(initial_mode)):
                node.set_mode(initial_mode)
                node.wait_for_state_match(mode=initial_mode, timeout_s=args.service_timeout_s, label="bag_initial_mode_confirmed")
                node.hold_neutral(1.0)
        if args.force_initial_mode and not args.no_mode:
            node.set_mode(str(args.force_initial_mode))
            node.wait_for_state_match(mode=str(args.force_initial_mode), timeout_s=args.service_timeout_s, label="force_initial_mode_confirmed")
            node.hold_neutral(max(float(args.initial_mode_settle_s), 0.0))
            node.snapshot("after_force_initial_mode_settle")
        if args.release_sim_initial_depth_hold and not released_initial_hold:
            node.wait_for_rc_out_stream(timeout_s=args.wait_rc_out_s)
            node.release_initial_depth_hold(args.service_timeout_s)
            if args.post_release_neutral_s > 0.0:
                node.hold_neutral(float(args.post_release_neutral_s))
            node.snapshot("after_release_after_force_mode")
        if args.pre_kick_duration_s > 0.0:
            node.hold_axes(
                args.pre_kick_duration_s,
                forward=args.pre_kick_forward,
                lateral=args.pre_kick_lateral,
                heave=args.pre_kick_heave,
                yaw=args.pre_kick_yaw,
                phase="pre_replay_kick",
            )
            if args.post_pre_kick_neutral_s > 0.0:
                node.hold_neutral(args.post_pre_kick_neutral_s)
            node.snapshot("after_pre_replay_kick")
        node.snapshot("before_replay")
        node.replay(
            seq,
            rate_scale=args.rate_scale,
            mode_sequence=None if args.no_mode else mode_sequence,
            output_kind=output_kind,
        )
        node.snapshot("after_replay", spin_s=0.0)
        node.hold_neutral(args.post_neutral_s)
        result = {
            "script": Path(__file__).name,
            "bag": str(args.bag),
            "topic": args.topic,
            "command_source": args.command_source,
            "bag_name": seq.bag_name,
            "rc_origin_s": float(seq.origin_s),
            "sample_count": int(seq.t.size),
            "replay_duration_s": float(seq.t[-1] - seq.t[0]) if seq.t.size >= 2 else 0.0,
            "rate_scale": float(args.rate_scale),
            "max_publish_hz": float(args.max_publish_hz),
            "mode_from_bag": bool(args.mode_from_bag),
            "skip_initial_mode_event": bool(args.skip_initial_mode_event),
            "force_initial_mode": str(args.force_initial_mode),
            "initial_mode_settle_s": float(args.initial_mode_settle_s),
            "mode_events": [
                {"t": float(t), "mode": str(mode)}
                for t, mode in (mode_sequence.events if mode_sequence is not None else [])
            ],
            "events": node.events,
        }
        (args.output_dir / "closed_loop_replay_events.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False)
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
