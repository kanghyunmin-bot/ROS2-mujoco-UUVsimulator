from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


FORWARD_CH = 4
SWAY_CH = 5
YAW_CH = 3
HEAVE_CH = 2
MODE_SEQUENCE = ("MANUAL", "STABILIZE", "ALT_HOLD", "POSHOLD")


def axis_to_pwm(value: float) -> int:
    value = max(-1.0, min(1.0, float(value)))
    return int(round(1500 + 300.0 * value))


def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class MavrosModePathSequence(Node):
    def __init__(self, output_dir: Path) -> None:
        super().__init__("mavros_mode_path_measurement_runner")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.phase_pub = self.create_publisher(String, "/measurement/phase", 10)
        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self.state_sub = self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.odom_sub = self.create_subscription(Odometry, "/mavros/local_position/odom", self._on_odom, 30)
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.latest_state: State | None = None
        self.latest_odom: Odometry | None = None
        self.events: list[dict[str, object]] = []
        self._t0 = time.monotonic()

    def _elapsed(self) -> float:
        return time.monotonic() - self._t0

    def _on_state(self, msg: State) -> None:
        prev = self.latest_state
        self.latest_state = msg
        if (
            prev is None
            or prev.armed != msg.armed
            or prev.mode != msg.mode
            or prev.connected != msg.connected
        ):
            self.events.append(
                {
                    "t": self._elapsed(),
                    "type": "state",
                    "connected": bool(msg.connected),
                    "armed": bool(msg.armed),
                    "mode": str(msg.mode),
                }
            )

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def spin_for(self, duration: float, timeout_sec: float = 0.05) -> None:
        end_t = time.monotonic() + duration
        while time.monotonic() < end_t:
            rclpy.spin_once(self, timeout_sec=timeout_sec)

    def wait_for_services(self, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.arm_client.wait_for_service(timeout_sec=0.25) and self.mode_client.wait_for_service(timeout_sec=0.25):
                return
            time.sleep(0.25)
        raise RuntimeError("MAVROS services did not become available in time")

    def wait_for_connected_state(self, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_state is not None and bool(self.latest_state.connected):
                return
        raise RuntimeError("No connected /mavros/state observed in time")

    def wait_for_odom(self, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_odom is not None:
                return
        raise RuntimeError("No /mavros/local_position/odom observed in time")

    def wait_for_mode(self, expected_mode: str, timeout: float = 8.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_state is not None and str(self.latest_state.mode) == expected_mode:
                return
        raise RuntimeError(f"Mode did not converge to {expected_mode}")

    def call_set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(f"set_mode({mode}) timeout")
        resp = future.result()
        ok = bool(resp is not None and resp.mode_sent)
        self.events.append({"t": self._elapsed(), "type": "set_mode", "mode": mode, "ok": ok})
        if not ok:
            raise RuntimeError(f"set_mode({mode}) failed")
        self.wait_for_mode(mode)
        return ok

    def call_arm(self, value: bool, timeout: float = 5.0) -> bool:
        req = CommandBool.Request()
        req.value = bool(value)
        future = self.arm_client.call_async(req)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError(f"arming({value}) timeout")
        resp = future.result()
        ok = bool(resp is not None and resp.success)
        self.events.append({"t": self._elapsed(), "type": "arm", "value": bool(value), "ok": ok})
        if not ok:
            raise RuntimeError(f"arming({value}) failed")
        return ok

    def current_pose_state(self) -> dict[str, float]:
        if self.latest_odom is None:
            raise RuntimeError("No odometry available")
        pose = self.latest_odom.pose.pose
        orientation = pose.orientation
        yaw = quat_to_yaw(
            float(orientation.w),
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
        )
        return {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "yaw_rad": float(yaw),
            "yaw_deg": float(math.degrees(yaw)),
        }

    def publish_phase(self, name: str) -> None:
        msg = String()
        msg.data = name
        self.phase_pub.publish(msg)

    def publish_rc(self, *, forward: float = 0.0, sway: float = 0.0, yaw: float = 0.0, heave: float = 0.0) -> None:
        msg = OverrideRCIn()
        for idx in range(18):
            msg.channels[idx] = 0
        for idx in range(8):
            msg.channels[idx] = 1500
        msg.channels[FORWARD_CH] = axis_to_pwm(forward)
        msg.channels[SWAY_CH] = axis_to_pwm(sway)
        msg.channels[YAW_CH] = axis_to_pwm(yaw)
        msg.channels[HEAVE_CH] = axis_to_pwm(heave)
        self.rc_pub.publish(msg)

    def phase_start_payload(self, name: str, **kwargs: float) -> dict[str, object]:
        payload: dict[str, object] = {
            "t": self._elapsed(),
            "type": "phase_start",
            "name": name,
            "pose_start": self.current_pose_state(),
        }
        payload.update(kwargs)
        return payload

    def hold_phase(
        self,
        name: str,
        duration: float,
        *,
        forward: float = 0.0,
        sway: float = 0.0,
        yaw: float = 0.0,
        heave: float = 0.0,
        hz: float = 20.0,
    ) -> None:
        self.events.append(
            self.phase_start_payload(
                name,
                duration=float(duration),
                forward=float(forward),
                sway=float(sway),
                yaw=float(yaw),
                heave=float(heave),
            )
        )
        dt = 1.0 / max(hz, 1.0)
        end_t = time.monotonic() + duration
        while time.monotonic() < end_t:
            self.publish_phase(name)
            self.publish_rc(forward=forward, sway=sway, yaw=yaw, heave=heave)
            rclpy.spin_once(self, timeout_sec=min(dt, 0.05))
            remaining = end_t - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(dt, remaining))
        self.publish_phase(f"{name}_end")
        self.publish_rc()
        self.events.append(
            {
                "t": self._elapsed(),
                "type": "phase_end",
                "name": name,
                "pose_end": self.current_pose_state(),
            }
        )

    def turn_until_yaw_delta(
        self,
        name: str,
        *,
        yaw_cmd: float,
        target_delta_deg: float = 90.0,
        max_duration: float = 8.0,
        hz: float = 20.0,
    ) -> None:
        start_pose = self.current_pose_state()
        start_yaw = float(start_pose["yaw_rad"])
        target_delta_rad = math.radians(abs(target_delta_deg))
        direction = 1.0 if yaw_cmd >= 0.0 else -1.0
        self.events.append(
            self.phase_start_payload(
                name,
                duration=float(max_duration),
                yaw=float(yaw_cmd),
                yaw_target_deg=float(target_delta_deg) * direction,
            )
        )

        dt = 1.0 / max(hz, 1.0)
        end_t = time.monotonic() + max_duration
        achieved_delta_rad = 0.0
        while time.monotonic() < end_t:
            self.publish_phase(name)
            self.publish_rc(yaw=yaw_cmd)
            rclpy.spin_once(self, timeout_sec=min(dt, 0.05))
            current_yaw = float(self.current_pose_state()["yaw_rad"])
            achieved_delta_rad = wrap_to_pi(current_yaw - start_yaw)
            if direction * achieved_delta_rad >= target_delta_rad:
                break
            remaining = end_t - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(dt, remaining))

        self.publish_phase(f"{name}_end")
        self.publish_rc()
        end_pose = self.current_pose_state()
        self.events.append(
            {
                "t": self._elapsed(),
                "type": "phase_end",
                "name": name,
                "pose_end": end_pose,
                "yaw_delta_deg": float(math.degrees(achieved_delta_rad)),
                "target_reached": abs(math.degrees(achieved_delta_rad)) >= abs(target_delta_deg),
            }
        )

    def run_mode_sequence(self, mode: str) -> None:
        mode_key = mode.lower()
        self.call_set_mode(mode)
        self.hold_phase(f"{mode_key}_mode_settle", 2.5)
        self.hold_phase(f"{mode_key}_neutral_pre", 1.5)
        self.hold_phase(f"{mode_key}_forward_leg", 3.5, forward=0.55)
        self.hold_phase(f"{mode_key}_pause_after_forward", 1.5)
        self.turn_until_yaw_delta(f"{mode_key}_turn_90", yaw_cmd=0.48, target_delta_deg=90.0, max_duration=6.0)
        self.hold_phase(f"{mode_key}_pause_after_turn", 1.5)
        self.hold_phase(f"{mode_key}_right_leg", 3.5, sway=0.55)
        self.hold_phase(f"{mode_key}_neutral_post", 2.0)

    def run(self) -> None:
        self.wait_for_services()
        self.wait_for_connected_state()
        self.wait_for_odom()

        self.hold_phase("preflight_neutral", 2.0)
        self.call_set_mode("MANUAL")
        self.hold_phase("manual_mode_settle", 1.5)
        self.call_arm(True)
        self.hold_phase("post_arm_settle", 5.0)

        for mode in MODE_SEQUENCE:
            self.run_mode_sequence(mode)

        self.call_arm(False)
        self.hold_phase("post_disarm", 1.0)

        result = {
            "runner": "measure_mavros_mode_path_sequence.py",
            "elapsed_s": self._elapsed(),
            "rc_mapping": {
                "forward_channel_index": FORWARD_CH,
                "sway_channel_index": SWAY_CH,
                "yaw_channel_index": YAW_CH,
                "heave_channel_index": HEAVE_CH,
            },
            "modes": list(MODE_SEQUENCE),
            "events": self.events,
        }
        (self.output_dir / "sequence_events.json").write_text(json.dumps(result, indent=2, ensure_ascii=False))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = MavrosModePathSequence(Path(args.output_dir))
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
