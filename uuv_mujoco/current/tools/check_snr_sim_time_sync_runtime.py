#!/usr/bin/env python3
"""Measure SNR/odometry alignment on the private MuJoCo simulation clock."""

from __future__ import annotations

import argparse
from collections import deque
import math
import time

import rclpy
from audio_common_msgs.msg import Float64Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class SyncProbe(Node):
    def __init__(self) -> None:
        super().__init__("snr_sim_time_sync_probe")
        clock_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.clock_stamps: list[float] = []
        self.odom_stamps: list[float] = []
        self.snr_stamps: list[float] = []
        self.snr_minus_latest_odom_s: list[float] = []
        self._latest_odom_stamp: float | None = None
        self._odom_history: deque[float] = deque()
        self.snr_interpolation_counts = {
            "ok": 0,
            "empty": 0,
            "too_old": 0,
            "too_new": 0,
        }
        self.create_subscription(Clock, "/uuv_mujoco/clock", self._on_clock, clock_qos)
        self.create_subscription(Odometry, "/sim/odom", self._on_odom, 50)
        self.create_subscription(
            Float64Stamped,
            "/audio_phase_estimator/iq_snr_ratio_stamped",
            self._on_snr,
            50,
        )

    def _on_clock(self, msg: Clock) -> None:
        self.clock_stamps.append(stamp_seconds(msg.clock))

    def _on_odom(self, msg: Odometry) -> None:
        stamp = stamp_seconds(msg.header.stamp)
        self.odom_stamps.append(stamp)
        self._latest_odom_stamp = stamp
        if self._odom_history and stamp < self._odom_history[-1]:
            self._odom_history.clear()
        self._odom_history.append(stamp)
        while self._odom_history and stamp - self._odom_history[0] > 20.0:
            self._odom_history.popleft()

    def _on_snr(self, msg: Float64Stamped) -> None:
        if math.isfinite(msg.data):
            stamp = stamp_seconds(msg.header.stamp)
            self.snr_stamps.append(stamp)
            if self._latest_odom_stamp is not None:
                self.snr_minus_latest_odom_s.append(
                    stamp - self._latest_odom_stamp
                )
            if not self._odom_history:
                self.snr_interpolation_counts["empty"] += 1
            elif stamp <= self._odom_history[0] and self._odom_history[0] - stamp > 0.35:
                self.snr_interpolation_counts["too_old"] += 1
            elif stamp >= self._odom_history[-1] and stamp - self._odom_history[-1] > 0.35:
                self.snr_interpolation_counts["too_new"] += 1
            else:
                self.snr_interpolation_counts["ok"] += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--max-nearest-error", type=float, default=0.060)
    args = parser.parse_args()

    rclpy.init()
    node = SyncProbe()
    deadline = time.monotonic() + max(1.0, args.duration)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    assert len(node.clock_stamps) >= 5, f"insufficient private clock samples: {len(node.clock_stamps)}"
    assert len(node.odom_stamps) >= 5, f"insufficient odometry samples: {len(node.odom_stamps)}"
    assert len(node.snr_stamps) >= 5, f"insufficient SNR samples: {len(node.snr_stamps)}"
    assert all(
        current + 1.0e-9 >= previous
        for previous, current in zip(node.clock_stamps, node.clock_stamps[1:])
    ), "private MuJoCo clock moved backwards"

    odom_min = min(node.odom_stamps)
    odom_max = max(node.odom_stamps)
    comparable_snr = [stamp for stamp in node.snr_stamps if odom_min <= stamp <= odom_max]
    assert len(comparable_snr) >= 5, "SNR and odometry timestamps do not share an epoch"
    comparable_ratio = len(comparable_snr) / len(node.snr_stamps)
    nearest_errors = [
        min(abs(snr_stamp - odom_stamp) for odom_stamp in node.odom_stamps)
        for snr_stamp in comparable_snr
    ]
    max_error = max(nearest_errors)
    mean_error = sum(nearest_errors) / len(nearest_errors)
    arrival_min = min(node.snr_minus_latest_odom_s)
    arrival_max = max(node.snr_minus_latest_odom_s)
    assert max_error <= args.max_nearest_error, (
        f"SNR/odometry timestamp error {max_error:.6f}s exceeds "
        f"{args.max_nearest_error:.6f}s"
    )
    print(
        "snr_sim_time_sync=PASS "
        f"clock_samples={len(node.clock_stamps)} "
        f"odom_samples={len(node.odom_stamps)} "
        f"snr_samples={len(node.snr_stamps)} "
        f"comparable_snr_samples={len(comparable_snr)} "
        f"comparable_ratio={comparable_ratio:.6f} "
        f"mean_nearest_error_s={mean_error:.6f} "
        f"max_nearest_error_s={max_error:.6f} "
        f"arrival_delta_min_s={arrival_min:.6f} "
        f"arrival_delta_max_s={arrival_max:.6f} "
        f"interpolation_counts={node.snr_interpolation_counts}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
