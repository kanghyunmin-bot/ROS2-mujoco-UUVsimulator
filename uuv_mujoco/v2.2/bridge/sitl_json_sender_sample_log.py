"""Sample diagnostic logging for ArduSub JSON sensor sends."""

from __future__ import annotations


def log_sitl_sensor_sample_due(
    self,
    payload: dict[str, object],
    *,
    now_wall: float,
    vertical_est,
    pressure_pa: float | None,
) -> None:
    if not (self._sitl_cmd_debug and now_wall - self._sitl_last_sensor_log_wall >= 2.0):
        return
    self._sitl_last_sensor_log_wall = now_wall
    try:
        print(
            "[sitl_transport] SITL tx sample "
            f"t={float(payload['timestamp']):.3f} "
            f"pos={payload['position']} vel={payload['velocity']} "
            f"acc={payload['imu']['accel_body']} "
            f"depth={vertical_est.depth_m:.3f} "
            f"abs_alt={payload.get('altitude', float('nan')):.3f} "
            f"dvl_alt={payload.get('rng_1', float('nan')):.3f} "
            f"bar30_abs_pa={float(pressure_pa) if pressure_pa is not None else float('nan'):.1f}",
            flush=True,
        )
    except Exception:
        pass


__all__ = ["log_sitl_sensor_sample_due"]
