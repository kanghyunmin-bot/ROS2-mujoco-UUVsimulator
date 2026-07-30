#!/usr/bin/env python3
"""Drive a forced hydrophone handoff and record four-lane vision acceptance."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import OverrideRCIn, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, Float32MultiArray, String, UInt32


PINGER_ID = "course_buoy_pinger_white_1_float"
EXPECTED_A_MAGNET_IDS = {
    PINGER_ID,
    "course_buoy_a_yellow_1_float",
    "course_buoy_a_yellow_2_float",
    "course_buoy_a_yellow_3_float",
    "course_buoy_a_yellow_4_float",
    "course_buoy_a_yellow_5_float",
    "course_buoy_a_orange_1_float",
    "course_buoy_a_orange_2_float",
}
LANE_COMPLETED_RE = re.compile(r"^lane_completed:(\d+):count=(\d+)$")


def latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class LaneAcceptanceProbe(Node):
    def __init__(
        self,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
        *,
        resume_existing: bool = False,
    ) -> None:
        super().__init__("lane_vision_acceptance_probe")
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_yaw = origin_yaw
        self.resume_existing = resume_existing

        self.start_frame_pub = self.create_publisher(
            PoseStamped, "/start_frame", latched_qos()
        )
        self.search_pub = self.create_publisher(
            Bool, "/homing/vision_search_active", latched_qos()
        )
        self.grant_pub = self.create_publisher(
            Bool, "/homing/vision_control_granted", latched_qos()
        )
        self.rc_pub = self.create_publisher(
            OverrideRCIn, "/mavros/rc/override", 10
        )

        self.state = ""
        self.state_history: list[str] = []
        self.lane_events: list[str] = []
        self.completed_lanes: set[int] = set()
        self.completed_lane_count = 0
        self.target_confirmed = False
        self.success = False
        self.detach_count = 0
        self.detached_id_events: list[str] = []
        self.fresh_status_detaches: set[str] = set()
        self._previous_detached: dict[str, bool] = {}
        self.status_rows: dict[str, dict[str, Any]] = {}
        self.have_status = False
        self.have_odom = False
        self.have_depth = False
        self.valid_bbox_messages = 0
        self.rc_messages = 0
        self.rc_before_grant = 0
        # A resumed probe joins an already granted mission.  Mark ownership
        # before subscriptions start so existing controller RC traffic is not
        # incorrectly counted as pre-grant traffic.
        self.grant_sent = resume_existing
        self.latest_world_position: tuple[float, float, float] | None = None
        self.fcu_mode = ""

        self.create_subscription(
            String, "/mission/state", self._on_state, latched_qos()
        )
        self.create_subscription(
            String, "/mission/lane_event", self._on_lane_event, latched_qos(20)
        )
        self.create_subscription(
            Bool, "/vision/target_confirmed", self._on_confirmed, latched_qos()
        )
        self.create_subscription(
            Bool, "/mission/success", self._on_success, latched_qos()
        )
        self.create_subscription(
            UInt32,
            "/vision/course_buoy_detach_count",
            self._on_detach_count,
            latched_qos(),
        )
        self.create_subscription(
            String,
            "/vision/course_buoy_detached_id",
            self._on_detached_id,
            latched_qos(20),
        )
        self.create_subscription(
            String, "/mujoco/course_buoys/status", self._on_status, 10
        )
        self.create_subscription(
            Odometry, "/sim/odom", self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/depth/pose",
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray, "/vision/buoy_bbox", self._on_bbox, 10
        )
        self.create_subscription(
            OverrideRCIn, "/mission/rc_command", self._on_rc, 10
        )
        self.create_subscription(State, "/mavros/state", self._on_fcu_state, 10)

    def publish_start_frame(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = self.origin_x
        msg.pose.position.y = self.origin_y
        msg.pose.orientation.z = math.sin(0.5 * self.origin_yaw)
        msg.pose.orientation.w = math.cos(0.5 * self.origin_yaw)
        self.start_frame_pub.publish(msg)

    def publish_search(self) -> None:
        self.search_pub.publish(Bool(data=True))

    def publish_neutral_rc(self) -> None:
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        for channel in (3, 4, 5):
            msg.channels[channel - 1] = 1500
        self.rc_pub.publish(msg)

    def publish_grant(self) -> None:
        self.grant_pub.publish(Bool(data=True))
        self.grant_sent = True

    def publish_grant_false(self) -> None:
        self.grant_pub.publish(Bool(data=False))

    def _on_state(self, msg: String) -> None:
        state = str(msg.data)
        self.state = state
        if not self.state_history or self.state_history[-1] != state:
            self.state_history.append(state)
            print(f"[acceptance] state={state}", flush=True)

    def _on_lane_event(self, msg: String) -> None:
        event = str(msg.data)
        if not event:
            return
        if not self.lane_events or self.lane_events[-1] != event:
            self.lane_events.append(event)
            print(f"[acceptance] lane_event={event}", flush=True)
        match = LANE_COMPLETED_RE.match(event)
        if match:
            self.completed_lanes.add(int(match.group(1)))
            self.completed_lane_count = max(
                self.completed_lane_count, int(match.group(2))
            )

    def _on_confirmed(self, msg: Bool) -> None:
        confirmed = bool(msg.data)
        if confirmed and not self.target_confirmed:
            print("[acceptance] target_confirmed=true", flush=True)
        self.target_confirmed = confirmed

    def _on_success(self, msg: Bool) -> None:
        self.success = bool(msg.data)

    def _on_detach_count(self, msg: UInt32) -> None:
        value = int(msg.data)
        if value != self.detach_count:
            print(f"[acceptance] detach_count={value}", flush=True)
        self.detach_count = value

    def _on_detached_id(self, msg: String) -> None:
        target_id = str(msg.data).strip()
        if not target_id or target_id in self.detached_id_events:
            return
        self.detached_id_events.append(target_id)
        print(f"[acceptance] detached_id={target_id}", flush=True)

    def _on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        rows = payload.get("buoys", []) if isinstance(payload, dict) else []
        if not isinstance(rows, list):
            return
        self.have_status = True
        for row in rows:
            if not isinstance(row, dict):
                continue
            target_id = str(row.get("id", "")).strip()
            if not target_id:
                continue
            self.status_rows[target_id] = row
            if not bool(row.get("has_magnet", False)):
                continue
            detached = bool(row.get("detached", False))
            previous = self._previous_detached.get(target_id)
            self._previous_detached[target_id] = detached
            release_time = row.get("release_time_s", -1.0)
            try:
                release_time = float(release_time)
            except (TypeError, ValueError, OverflowError):
                release_time = -1.0
            authoritative_transition = previous is False and detached
            authoritative_resume_snapshot = (
                self.resume_existing and previous is None and detached
            )
            if (
                (authoritative_transition or authoritative_resume_snapshot)
                and row.get("eq_active") is False
                and math.isfinite(release_time)
                and release_time >= 0.0
            ):
                self.fresh_status_detaches.add(target_id)
                print(
                    f"[acceptance] authoritative_detach={target_id} "
                    f"release_time_s={release_time:.3f}",
                    flush=True,
                )

    def _on_odom(self, msg: Odometry) -> None:
        self.have_odom = True
        position = msg.pose.pose.position
        self.latest_world_position = (
            float(position.x),
            float(position.y),
            float(position.z),
        )

    def _on_depth(self, _msg: PoseWithCovarianceStamped) -> None:
        self.have_depth = True

    def _on_bbox(self, msg: Float32MultiArray) -> None:
        data = msg.data
        if len(data) >= 10 and data[1] >= 0.5:
            self.valid_bbox_messages += 1

    def _on_rc(self, _msg: OverrideRCIn) -> None:
        self.rc_messages += 1
        if not self.grant_sent:
            self.rc_before_grant += 1

    def _on_fcu_state(self, msg: State) -> None:
        self.fcu_mode = str(msg.mode)


def build_report(
    probe: LaneAcceptanceProbe,
    *,
    elapsed_s: float,
    timeout_s: float,
    required_detaches: int,
    stop_after_pinger: bool,
) -> dict[str, Any]:
    expected_detached = EXPECTED_A_MAGNET_IDS.intersection(
        probe.fresh_status_detaches
    )
    checks: dict[str, bool] = {
        "target_confirmed_before_grant": probe.target_confirmed,
        "no_vision_rc_before_grant": probe.rc_before_grant == 0,
        "fcu_mode_stabilize": probe.fcu_mode == "STABILIZE",
        "pinger_physically_detached": PINGER_ID in probe.fresh_status_detaches,
        "monitor_matches_authoritative_count": (
            probe.detach_count == len(probe.fresh_status_detaches)
        ),
    }
    if not stop_after_pinger:
        checks.update(
            {
                "four_distinct_lanes_completed": probe.completed_lane_count == 4,
                "controller_complete": probe.state == "COMPLETE",
                "controller_success": probe.success,
                "required_physical_detaches": (
                    len(expected_detached) >= required_detaches
                ),
            }
        )
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "elapsed_s": elapsed_s,
        "timeout_s": timeout_s,
        "required_detaches": required_detaches,
        "stop_after_pinger": stop_after_pinger,
        "state": probe.state,
        "state_history": probe.state_history,
        "lane_events": probe.lane_events,
        "completed_lanes": sorted(probe.completed_lanes),
        "completed_lane_count": probe.completed_lane_count,
        "resume_existing": probe.resume_existing,
        "success": probe.success,
        "target_confirmed": probe.target_confirmed,
        "valid_bbox_messages": probe.valid_bbox_messages,
        "rc_messages": probe.rc_messages,
        "rc_before_grant": probe.rc_before_grant,
        "detach_count": probe.detach_count,
        "detached_id_events": probe.detached_id_events,
        "authoritative_detached_ids": sorted(probe.fresh_status_detaches),
        "expected_a_detached_ids": sorted(expected_detached),
        "expected_a_missing_ids": sorted(
            EXPECTED_A_MAGNET_IDS.difference(probe.fresh_status_detaches)
        ),
        "latest_world_position": probe.latest_world_position,
        "fcu_mode": probe.fcu_mode,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=7200.0,
        help="Wall-clock timeout; CPU YOLO can make simulation much slower than real time.",
    )
    parser.add_argument("--confirm-timeout-sec", type=float, default=45.0)
    parser.add_argument("--origin-x", type=float, default=-15.881)
    parser.add_argument("--origin-y", type=float, default=1.305)
    parser.add_argument("--origin-yaw", type=float, default=0.0)
    parser.add_argument(
        "--required-detaches",
        type=int,
        default=len(EXPECTED_A_MAGNET_IDS),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=None,
        help="JSON output path (default: sim/current/logs timestamped file)",
    )
    parser.add_argument(
        "--stop-after-pinger",
        action="store_true",
        help="End after the initial pinger gets an authoritative detach event.",
    )
    parser.add_argument(
        "--resume-existing",
        action="store_true",
        help=(
            "Join an already granted mission and include authoritative detached "
            "buoys present in the first physical-status snapshot."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1 <= args.required_detaches <= len(EXPECTED_A_MAGNET_IDS):
        raise SystemExit(
            f"--required-detaches must be in [1, {len(EXPECTED_A_MAGNET_IDS)}]"
        )

    rclpy.init()
    probe = LaneAcceptanceProbe(
        args.origin_x,
        args.origin_y,
        args.origin_yaw,
        resume_existing=args.resume_existing,
    )
    started_at = time.monotonic()
    confirm_deadline = started_at + args.confirm_timeout_sec
    deadline = started_at + args.timeout_sec
    neutral_started_at: float | None = None
    last_latched_publish = 0.0
    last_progress_print = 0.0

    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            now_wall = time.monotonic()
            if now_wall - last_latched_publish >= 0.5:
                probe.publish_start_frame()
                probe.publish_search()
                if probe.grant_sent:
                    probe.publish_grant()
                last_latched_publish = now_wall

            if probe.target_confirmed and not probe.grant_sent:
                if neutral_started_at is None:
                    neutral_started_at = now_wall
                    print(
                        "[acceptance] publishing hydrophone-neutral RC before grant",
                        flush=True,
                    )
                probe.publish_neutral_rc()
                if now_wall - neutral_started_at >= 0.5:
                    probe.publish_grant()
                    print("[acceptance] vision_control_granted=true", flush=True)

            rclpy.spin_once(probe, timeout_sec=0.05)

            if not probe.grant_sent and now_wall >= confirm_deadline:
                print("[acceptance] target confirmation timeout", flush=True)
                break
            if probe.state in {"COMPLETE", "FAILSAFE"}:
                # Let transient events and the final physical status drain.
                drain_deadline = time.monotonic() + 1.0
                while rclpy.ok() and time.monotonic() < drain_deadline:
                    rclpy.spin_once(probe, timeout_sec=0.05)
                break
            if (
                args.stop_after_pinger
                and PINGER_ID in probe.fresh_status_detaches
            ):
                drain_deadline = time.monotonic() + 1.0
                while rclpy.ok() and time.monotonic() < drain_deadline:
                    rclpy.spin_once(probe, timeout_sec=0.05)
                break
            if now_wall - last_progress_print >= 10.0:
                print(
                    "[acceptance] progress "
                    f"state={probe.state or 'UNKNOWN'} "
                    f"lanes={probe.completed_lane_count}/4 "
                    f"detached={probe.detach_count} "
                    f"position={probe.latest_world_position}",
                    flush=True,
                )
                last_progress_print = now_wall
    except KeyboardInterrupt:
        interrupted = True
        print("[acceptance] interrupted; revoking vision control grant", flush=True)
    finally:
        if rclpy.ok():
            probe.publish_grant_false()
            rclpy.spin_once(probe, timeout_sec=0.1)

    elapsed_s = time.monotonic() - started_at
    report = build_report(
        probe,
        elapsed_s=elapsed_s,
        timeout_s=args.timeout_sec,
        required_detaches=args.required_detaches,
        stop_after_pinger=args.stop_after_pinger,
    )
    report["interrupted"] = interrupted
    report_path = args.report
    if report_path is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        report_path = (
            Path(__file__).resolve().parents[1]
            / "logs"
            / f"lane_vision_acceptance_{stamp}.json"
        )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"lane_vision_acceptance={'PASS' if report['passed'] else 'FAIL'} "
        f"lanes={probe.completed_lane_count}/4 "
        f"detached={len(probe.fresh_status_detaches)} "
        f"report={report_path}",
        flush=True,
    )

    probe.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
