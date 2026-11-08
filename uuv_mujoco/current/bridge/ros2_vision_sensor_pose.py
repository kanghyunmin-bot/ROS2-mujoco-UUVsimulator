"""External-navigation pose built only from timed DVL/gyro dead reckoning."""

from __future__ import annotations

from dataclasses import dataclass
import os

import numpy as np

from .ros2_bridge_publish_stamp import stamp_from_seconds_like
from .ros2_math import quat_wxyz_to_rotmat, rotmat_to_quat_wxyz, rpy_deg_to_rotmat
from .ros2_standard_messages import build_pose_msg


class VisionSensorPoseRuntime:
    """Convert sensor-local DR to an explicitly initialized ENU reference.

    The initial pose is configured once [m, rad], never read from live model
    pose. Device DR resets continue from the last emitted estimate; a clock
    rewind restores the configured initial pose. Missing data never falls
    back to simulator truth.
    """

    @dataclass(frozen=True)
    class Config:
        position_enu_m: tuple[float, float, float] = (0., 0., 0.)
        rpy_enu_rad: tuple[float, float, float] = (0., 0., 0.)
        max_capture_age_s: float = .5

    def __init__(self, pose_type, *, body_flu_to_dvl_frd: np.ndarray,
                 dvl_position_body_flu_m: np.ndarray, config: Config | None = None):
        self.config = config or self.Config()
        self.pose_type = pose_type
        self.body_to_dvl = np.asarray(body_flu_to_dvl_frd, dtype=np.float64).copy()
        self.lever_dvl = self.body_to_dvl @ np.asarray(dvl_position_body_flu_m, dtype=np.float64)
        self.initial_position = np.asarray(self.config.position_enu_m, dtype=np.float64)
        self.initial_rotation = rpy_deg_to_rotmat(*np.rad2deg(self.config.rpy_enu_rad))
        if self.body_to_dvl.shape != (3, 3) or not np.isfinite(self.body_to_dvl).all():
            raise ValueError("DVL rotation must be a finite 3x3 matrix")
        if not np.allclose(self.body_to_dvl.T @ self.body_to_dvl, np.eye(3), atol=1e-8):
            raise ValueError("DVL rotation must be orthonormal")
        if not np.isclose(np.linalg.det(self.body_to_dvl), 1.):
            raise ValueError("DVL rotation must be proper")
        if self.lever_dvl.shape != (3,) or self.initial_position.shape != (3,):
            raise ValueError("Vision initial position and DVL lever must have three components")
        if not all(np.isfinite(value).all() for value in (self.lever_dvl, self.initial_position, self.initial_rotation)):
            raise ValueError("Vision configuration must be finite")
        if not np.isfinite(self.config.max_capture_age_s) or self.config.max_capture_age_s <= 0:
            raise ValueError("Vision maximum capture age must be positive")
        self._generation = None
        self._last_capture_s = None
        self._last_now_s = None
        self._last_position = self.initial_position.copy()
        self._last_rotation = self.initial_rotation.copy()
        self._origin_position = self.initial_position.copy()
        self._origin_rotation = self.initial_rotation @ self.body_to_dvl.T
        self.rejection_counts: dict[str, int] = {}

    @classmethod
    def from_bridge(cls, bridge):
        """Read static mounting geometry and explicit initial pose only."""
        site, base = int(bridge._dvl_site_id), int(bridge._base_id)
        if site < 0 or base < 0:
            raise ValueError("Vision navigation requires a fixed DVL site")
        model = bridge.model
        position = np.asarray(model.site_pos[site], dtype=np.float64).copy()
        body = int(model.site_bodyid[site])
        while body != base:
            if body <= 0 or int(model.body_jntnum[body]):
                raise ValueError("Vision navigation requires a fixed DVL mount under base_link")
            position = (np.asarray(model.body_pos[body], dtype=np.float64)
                        + quat_wxyz_to_rotmat(model.body_quat[body]) @ position)
            body = int(model.body_parentid[body])
        config = cls.Config(
            position_enu_m=_initial_vector("ROS2_UUV_VISION_INITIAL_POSITION_ENU_M"),
            rpy_enu_rad=_initial_vector("ROS2_UUV_VISION_INITIAL_RPY_RAD"),
        )
        return cls(bridge.PoseStamped,
                   body_flu_to_dvl_frd=bridge._dvl_body_frd_to_dvl_frd @ bridge._bmj_to_frd,
                   dvl_position_body_flu_m=position, config=config)

    def build_message(self, stamp, delivery, *, now_s: float, generation: int):
        """Return an estimated pose [m, quaternion], or no message if invalid."""
        if delivery is None or not bool(getattr(delivery, "navigation_valid", False)):
            return self._reject("invalid_sensor")
        if delivery.generation != generation:
            return self._reject("old_generation")
        try:
            capture = float(delivery.capture_time_s)
            arrival = float(delivery.arrival_time_s)
            position = np.asarray(delivery.position_local_frd_m, dtype=np.float64)
            attitude = np.asarray(delivery.attitude_rpy_deg, dtype=np.float64)
        except (TypeError, ValueError, AttributeError):
            return self._reject("invalid_values")
        if (position.shape != (3,) or attitude.shape != (3,)
                or not all(np.isfinite(value).all() for value in (position, attitude))
                or not all(np.isfinite(value) for value in (now_s, capture, arrival, delivery.position_std_m))
                or delivery.position_std_m < 0 or capture < 0 or arrival < capture - 1e-9):
            return self._reject("invalid_values")
        if capture > now_s + 1e-9 or arrival > now_s + 1e-9:
            return self._reject("not_arrived")
        if now_s - capture > self.config.max_capture_age_s:
            return self._reject("stale_capture")
        rewind = self._last_now_s is not None and now_s + 1e-9 < self._last_now_s
        restart = self._generation is not None and generation < self._generation
        if rewind or restart:
            self._last_position = self.initial_position.copy()
            self._last_rotation = self.initial_rotation.copy()
        if self._generation != generation or rewind:
            self._origin_position = self._last_position.copy()
            self._origin_rotation = self._last_rotation @ self.body_to_dvl.T
            self._last_capture_s = None
            self._generation = generation
        if self._last_capture_s is not None and capture <= self._last_capture_s + 1e-9:
            return self._reject("nonincreasing_capture")
        relative = rpy_deg_to_rotmat(*attitude)
        # A50 DR integrates the sensor site. Express the base origin using
        # the fixed lever, compensating its motion under pure body rotation.
        relative_base = position + self.lever_dvl - relative @ self.lever_dvl
        estimate_position = self._origin_position + self._origin_rotation @ relative_base
        estimate_rotation = self._origin_rotation @ relative @ self.body_to_dvl
        message = build_pose_msg(self.pose_type, stamp_from_seconds_like(stamp, capture),
                                 "map", estimate_position, rotmat_to_quat_wxyz(estimate_rotation))
        self._last_position, self._last_rotation = estimate_position, estimate_rotation
        self._last_capture_s, self._last_now_s = capture, now_s
        return message

    def _reject(self, reason: str):
        self.rejection_counts[reason] = self.rejection_counts.get(reason, 0) + 1
        return None

    def reset_initial(self) -> None:
        """Restore the configured initial reference [m, rad] after a scene reset."""
        self._generation = self._last_capture_s = self._last_now_s = None
        self._last_position = self.initial_position.copy()
        self._last_rotation = self.initial_rotation.copy()
        self._origin_position = self.initial_position.copy()
        self._origin_rotation = self.initial_rotation @ self.body_to_dvl.T


def build_sensor_vision_pose(bridge, stamp, state):
    """Build from the delivered sensor report, never from publish-state truth."""
    delivery = getattr(state, "dvl_position_delivery", None)
    if (not getattr(bridge, "_dvl_sensor_enabled", False) or delivery is None
            or not bool(getattr(delivery, "navigation_valid", False))):
        return None
    runtime = getattr(bridge, "_vision_sensor_pose_runtime", None)
    if runtime is None:
        runtime = VisionSensorPoseRuntime.from_bridge(bridge)
        bridge._vision_sensor_pose_runtime = runtime
    return runtime.build_message(stamp, delivery,
                                 now_s=float(state.sim_t), generation=int(bridge._dvl_sensor_dr_generation))


def _initial_vector(name: str) -> tuple[float, float, float]:
    raw = os.environ.get(name, "0 0 0")
    values = tuple(float(value) for value in raw.replace(",", " ").split())
    if len(values) != 3 or not all(np.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain three finite values")
    return values


__all__ = ["VisionSensorPoseRuntime", "build_sensor_vision_pose"]
