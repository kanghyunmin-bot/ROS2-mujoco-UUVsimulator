#!/usr/bin/env python3
"""V2 Estimator의 rolling SNR map과 최종 homing 방향을 실시간 표시한다."""

import math
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from audio_common_msgs.msg import Float64Stamped
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Float64, String


class SnrMapVisualizer(Node):
    """최근 odometry 상대 이동으로 AUV 주변 rolling SNR map을 재구성한다."""

    MAX_CELL_VALUES = 31
    MAX_ODOMETRY_SAMPLES = 500
    MAX_PENDING_SNR = 500
    MAX_ODOMETRY_EXTRAPOLATION_S = 0.12
    MIN_SAMPLE_SPACING_M = 0.02
    MIN_DISPLAY_SNR_DB = -240.0
    MAX_DISPLAY_SNR_DB = 80.0
    GRID_KEEP_MARGIN_M = 1.0
    GRID_MAX_AGE_S = 60.0
    DIRECTION_TIMEOUT_S = 1.0

    def __init__(self) -> None:
        super().__init__("snr_map_visualizer")

        self.map_cell_size_m = float(
            np.clip(
                self.declare_parameter("map_cell_size_m", 0.12).value,
                0.05,
                1.0,
            )
        )
        self.map_radius_m = max(
            self.map_cell_size_m,
            float(self.declare_parameter("map_radius_m", 2.0).value),
        )
        self.grid_keep_radius_m = self.map_radius_m + self.GRID_KEEP_MARGIN_M
        self.display_rotation_rad = float(
            self.declare_parameter("display_rotation_rad", 0.0).value
        )
        self.plot_rate_hz = max(
            0.5, float(self.declare_parameter("plot_rate_hz", 5.0).value)
        )
        snr_topic = str(
            self.declare_parameter(
                "snr_topic", "/audio_frequency_detector/snr_db_stamped"
            ).value
        )
        odometry_topic = str(
            self.declare_parameter("odometry_topic", "/odometry/filtered").value
        )
        reset_topic = str(
            self.declare_parameter("reset_topic", "/homing/reset_estimator").value
        )
        direction_topic = str(
            self.declare_parameter("direction_topic", "/homing/direction").value
        )
        confidence_topic = str(
            self.declare_parameter(
                "confidence_topic", "/homing/snr_confidence"
            ).value
        )
        estimator_ready_topic = str(
            self.declare_parameter(
                "estimator_ready_topic", "/homing/estimator_ready"
            ).value
        )
        state_topic = str(
            self.declare_parameter("state_topic", "/homing/control_state").value
        )

        self.odom_history = deque()
        self.pending_snr = deque()
        self.grid = {}
        self.current_world_xy = None
        self.current_yaw_rad = 0.0
        self.last_sample_world_xy = None
        self.direction = None
        self.direction_time_s = 0.0
        self.direction_confidence = 0.0
        self.estimator_ready = False
        self.control_state = "UNKNOWN"

        self.received_snr_count = 0
        self.received_odom_count = 0
        self.received_direction_count = 0
        self.accepted_sample_count = 0
        self.spacing_rejected_count = 0

        self.snr_sub = self.create_subscription(
            Float64Stamped,
            snr_topic,
            self.snr_callback,
            20,
        )
        self.odom_sub = self.create_subscription(
            Odometry, odometry_topic, self.odometry_callback, 30
        )
        self.reset_sub = self.create_subscription(
            Empty, reset_topic, self.reset_callback, 10
        )
        self.direction_sub = self.create_subscription(
            Vector3Stamped,
            direction_topic,
            self.direction_callback,
            10,
        )
        self.confidence_sub = self.create_subscription(
            Float64,
            confidence_topic,
            self.confidence_callback,
            10,
        )
        self.ready_sub = self.create_subscription(
            Bool,
            estimator_ready_topic,
            self.ready_callback,
            10,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_sub = self.create_subscription(
            String,
            state_topic,
            self.state_callback,
            state_qos,
        )

        self.view_radius_m = self.grid_keep_radius_m
        self.nx = max(
            1, int(math.ceil(2.0 * self.view_radius_m / self.map_cell_size_m))
        )
        self.ny = self.nx
        plt.ion()
        self.figure, self.axes = plt.subplots(num="Rolling SNR Map V2")
        initial_map = np.full((self.ny, self.nx), np.nan, dtype=float)
        self.image = self.axes.imshow(
            initial_map,
            origin="lower",
            extent=(
                -self.view_radius_m,
                self.view_radius_m,
                -self.view_radius_m,
                self.view_radius_m,
            ),
            interpolation="nearest",
            aspect="equal",
            cmap="turbo",
        )
        self.colorbar = self.figure.colorbar(self.image, ax=self.axes)
        self.colorbar.set_label("SNR (dB)")
        (self.vehicle_marker,) = self.axes.plot(
            [0.0],
            [0.0],
            marker="o",
            color="white",
            markeredgecolor="black",
            label="AUV",
        )
        self.direction_arrow = self.axes.quiver(
            [0.0], [0.0], [0.0], [0.0], color="#00ff00", scale_units="xy",
            angles="xy", scale=1.0, width=0.010, label="V2 homing direction"
        )
        self.axes.set_xlabel("Relative odometry x (m)")
        self.axes.set_ylabel("Relative odometry y (m)")
        self.axes.set_xlim(-self.view_radius_m, self.view_radius_m)
        self.axes.set_ylim(-self.view_radius_m, self.view_radius_m)
        self.axes.grid(True, alpha=0.2)
        self.axes.legend(loc="upper right")
        self.figure.tight_layout()

        self.plot_timer = self.create_timer(1.0 / self.plot_rate_hz, self.update_plot)
        self.status_timer = self.create_timer(2.0, self.log_status)
        self.get_logger().info(
            "Rolling SNR map visualizer ready: "
            f"cell={self.map_cell_size_m:.2f} m "
            f"map_radius={self.map_radius_m:.2f} m "
            f"keep_radius={self.grid_keep_radius_m:.2f} m "
            f"display_rotation={self.display_rotation_rad:.3f} rad."
        )

    @staticmethod
    def stamp_seconds(stamp) -> float:
        """ROS builtin time을 부동소수점 초로 바꾼다."""
        return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)

    @staticmethod
    def yaw_from_quaternion(quaternion) -> float:
        """Odometry quaternion에서 평면 yaw를 계산한다."""
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    def now_seconds(self) -> float:
        """방향 토픽 freshness 판정용 ROS 현재 시각을 초로 반환한다."""
        return 1.0e-9 * float(self.get_clock().now().nanoseconds)

    def odometry_callback(self, msg: Odometry) -> None:
        """현재 위치·yaw와 timestamp 위치 이력을 갱신한다."""
        self.received_odom_count += 1
        stamp_s = self.stamp_seconds(msg.header.stamp)
        if stamp_s <= 0.0:
            return
        position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y], dtype=float
        )
        self.current_world_xy = position
        self.current_yaw_rad = self.yaw_from_quaternion(msg.pose.pose.orientation)
        if self.odom_history and stamp_s < self.odom_history[-1][0]:
            self.odom_history.clear()
            self.clear_measurements()
        self.odom_history.append((stamp_s, position))
        while len(self.odom_history) > self.MAX_ODOMETRY_SAMPLES:
            self.odom_history.popleft()
        self.process_pending_snr()

    def snr_callback(self, msg: Float64Stamped) -> None:
        """Detector가 발행한 SNR dB를 timestamp 대기열에 넣는다."""
        self.received_snr_count += 1
        snr_db = float(msg.data)
        stamp_s = self.stamp_seconds(msg.header.stamp)
        if not math.isfinite(snr_db) or stamp_s <= 0.0:
            return
        snr_db = float(
            np.clip(snr_db, self.MIN_DISPLAY_SNR_DB, self.MAX_DISPLAY_SNR_DB)
        )
        self.pending_snr.append((stamp_s, snr_db))
        self.pending_snr = deque(sorted(self.pending_snr, key=lambda item: item[0]))
        while len(self.pending_snr) > self.MAX_PENDING_SNR:
            self.pending_snr.popleft()
        self.process_pending_snr()

    def reset_callback(self, _msg: Empty) -> None:
        """미션 reset에 맞춰 rolling map과 방향 표시를 비운다."""
        self.clear_measurements()

    def direction_callback(self, msg: Vector3Stamped) -> None:
        """V2 Estimator가 발행한 최종 body-frame 방향을 저장한다."""
        self.direction = np.array([msg.vector.x, msg.vector.y], dtype=float)
        self.direction_time_s = self.now_seconds()
        self.received_direction_count += 1

    def confidence_callback(self, msg: Float64) -> None:
        self.direction_confidence = float(msg.data)

    def ready_callback(self, msg: Bool) -> None:
        self.estimator_ready = bool(msg.data)

    def state_callback(self, msg: String) -> None:
        self.control_state = msg.data

    def clear_measurements(self) -> None:
        """Rolling map, 대기열과 stale 방향 표시를 모두 초기화한다."""
        self.pending_snr.clear()
        self.grid.clear()
        self.last_sample_world_xy = None
        self.direction = None
        self.direction_confidence = 0.0
        self.estimator_ready = False

    def process_pending_snr(self) -> None:
        """SNR timestamp 위치를 보간해 rolling grid에 공간 표본으로 추가한다."""
        while self.pending_snr:
            stamp_s, snr_db = self.pending_snr[0]
            position = self.interpolate_odometry(stamp_s)
            if position is None:
                if self.odom_history and stamp_s < self.odom_history[0][0]:
                    self.pending_snr.popleft()
                    continue
                return
            self.pending_snr.popleft()
            self.prune_grid(position, stamp_s)
            if (
                self.last_sample_world_xy is not None
                and np.linalg.norm(position - self.last_sample_world_xy)
                < self.MIN_SAMPLE_SPACING_M
            ):
                self.spacing_rejected_count += 1
                continue
            self.last_sample_world_xy = position
            cell = self.world_to_cell(position)
            if cell not in self.grid:
                self.grid[cell] = {
                    "values": deque(maxlen=self.MAX_CELL_VALUES),
                    "last_stamp": stamp_s,
                }
            self.grid[cell]["values"].append(snr_db)
            self.grid[cell]["last_stamp"] = stamp_s
            self.accepted_sample_count += 1

    def interpolate_odometry(self, stamp_s: float):
        """주어진 SNR 시각의 수평 위치를 odometry 이력에서 선형 보간한다."""
        if not self.odom_history:
            return None
        first_t, first_p = self.odom_history[0]
        last_t, last_p = self.odom_history[-1]
        if stamp_s <= first_t:
            return (
                first_p.copy()
                if first_t - stamp_s <= self.MAX_ODOMETRY_EXTRAPOLATION_S
                else None
            )
        if stamp_s >= last_t:
            return (
                last_p.copy()
                if stamp_s - last_t <= self.MAX_ODOMETRY_EXTRAPOLATION_S
                else None
            )
        for index in range(1, len(self.odom_history)):
            after_t, after_p = self.odom_history[index]
            if after_t >= stamp_s:
                before_t, before_p = self.odom_history[index - 1]
                duration = after_t - before_t
                alpha = (stamp_s - before_t) / duration if duration > 0.0 else 0.0
                return (1.0 - alpha) * before_p + alpha * after_p
        return None

    def world_to_cell(self, world_xy):
        """경계 없는 odometry 평면 위치를 rolling grid 인덱스로 바꾼다."""
        return (
            int(math.floor(world_xy[0] / self.map_cell_size_m)),
            int(math.floor(world_xy[1] / self.map_cell_size_m)),
        )

    def cell_center_world(self, cell):
        """Rolling grid 인덱스의 odometry-frame 셀 중심을 반환한다."""
        return np.array(
            [
                (float(cell[0]) + 0.5) * self.map_cell_size_m,
                (float(cell[1]) + 0.5) * self.map_cell_size_m,
            ],
            dtype=float,
        )

    def prune_grid(self, center, stamp_s: float) -> None:
        """현재 주변 보존 반경 밖이거나 오래된 rolling grid 셀을 삭제한다."""
        stale_cells = []
        for cell, entry in self.grid.items():
            distance = np.linalg.norm(self.cell_center_world(cell) - center)
            age_s = max(0.0, stamp_s - float(entry["last_stamp"]))
            if distance > self.grid_keep_radius_m or age_s > self.GRID_MAX_AGE_S:
                stale_cells.append(cell)
        for cell in stale_cells:
            del self.grid[cell]

    def body_direction_to_world(self, body_direction):
        """body-frame 방향을 현재 yaw 기준 odometry 평면 방향으로 회전한다."""
        c = math.cos(self.current_yaw_rad)
        s = math.sin(self.current_yaw_rad)
        return np.array(
            [
                c * body_direction[0] - s * body_direction[1],
                s * body_direction[0] + c * body_direction[1],
            ],
            dtype=float,
        )

    def rotate_for_display(self, vector):
        """상대 odometry 벡터를 사용자 지정 각도만큼 화면에서 회전한다."""
        c = math.cos(self.display_rotation_rad)
        s = math.sin(self.display_rotation_rad)
        return np.array(
            [
                c * vector[0] - s * vector[1],
                s * vector[0] + c * vector[1],
            ],
            dtype=float,
        )

    def update_arrow(self, arrow, direction, receive_time_s: float) -> None:
        """신선한 gradient 방향만 AUV 중심의 고정 길이 화살표로 표시한다."""
        if (
            direction is None
            or self.now_seconds() - receive_time_s > self.DIRECTION_TIMEOUT_S
        ):
            arrow.set_UVC([0.0], [0.0])
            return
        display_direction = self.rotate_for_display(
            self.body_direction_to_world(direction)
        )
        norm = np.linalg.norm(display_direction)
        if not math.isfinite(float(norm)) or norm < 1.0e-9:
            arrow.set_UVC([0.0], [0.0])
            return
        arrow_length = min(0.8, 0.35 * self.view_radius_m)
        display_direction *= arrow_length / norm
        arrow.set_UVC([display_direction[0]], [display_direction[1]])

    def update_plot(self) -> None:
        """AUV 중심 상대 셀 median과 V2 최종 방향을 갱신한다."""
        self.display_rotation_rad = float(
            self.get_parameter("display_rotation_rad").value
        )
        values = np.full((self.ny, self.nx), np.nan, dtype=float)
        if self.current_world_xy is not None:
            for cell, entry in self.grid.items():
                relative = self.rotate_for_display(
                    self.cell_center_world(cell) - self.current_world_xy
                )
                if (
                    abs(relative[0]) > self.view_radius_m
                    or abs(relative[1]) > self.view_radius_m
                ):
                    continue
                ix = int(
                    math.floor(
                        (relative[0] + self.view_radius_m) / self.map_cell_size_m
                    )
                )
                iy = int(
                    math.floor(
                        (relative[1] + self.view_radius_m) / self.map_cell_size_m
                    )
                )
                if 0 <= ix < self.nx and 0 <= iy < self.ny and entry["values"]:
                    values[iy, ix] = float(np.median(np.asarray(entry["values"])))
        self.image.set_data(values)
        finite = values[np.isfinite(values)]
        if finite.size:
            low, high = np.percentile(finite, [5.0, 95.0])
            if high - low < 1.0:
                high = low + 1.0
            self.image.set_clim(float(low), float(high))

        self.update_arrow(
            self.direction_arrow, self.direction, self.direction_time_s
        )
        self.axes.set_title(
            f"V2 rolling SNR map ({len(self.grid)} cells) | "
            f"{self.control_state} | ready={self.estimator_ready} "
            f"confidence={self.direction_confidence:.2f}"
        )
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        plt.pause(0.001)

    def log_status(self) -> None:
        """입력 수신과 rolling grid 처리 상태를 주기적으로 출력한다."""
        self.get_logger().info(
            "rolling map status: "
            f"snr={self.received_snr_count} odom={self.received_odom_count} "
            f"accepted={self.accepted_sample_count} cells={len(self.grid)} "
            f"pending={len(self.pending_snr)} spacing={self.spacing_rejected_count} "
            f"direction={self.received_direction_count} "
            f"ready={self.estimator_ready} confidence={self.direction_confidence:.2f} "
            f"state={self.control_state}"
        )


def main(args=None) -> None:
    """ROS executor와 matplotlib 창을 함께 실행한다."""
    rclpy.init(args=args)
    node = SnrMapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        plt.close("all")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
