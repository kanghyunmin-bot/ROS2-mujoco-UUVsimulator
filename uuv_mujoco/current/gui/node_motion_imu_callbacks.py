"""IMU and battery callbacks for UuvGuiNode."""

from __future__ import annotations

import math


def _quaternion_to_euler_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
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


def _on_imu(self, msg: Imu) -> None:
    self._touch("imu")
    q = msg.orientation
    roll_deg, pitch_deg, yaw_deg = _quaternion_to_euler_deg(q.w, q.x, q.y, q.z)
    with self._lock:
        self._snapshot.roll_deg = roll_deg
        self._snapshot.pitch_deg = pitch_deg
        self._snapshot.yaw_deg = yaw_deg
        self._snapshot.ang_vel_xyz = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        self._snapshot.lin_acc_xyz = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )


def _on_battery(self, msg: BatteryState) -> None:
    with self._lock:
        self._snapshot.battery_voltage = msg.voltage
        self._snapshot.battery_current = msg.current
        self._snapshot.battery_percent = msg.percentage


__all__ = ["_on_battery", "_on_imu"]
