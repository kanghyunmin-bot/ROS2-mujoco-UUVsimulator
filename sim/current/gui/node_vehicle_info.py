"""Vehicle-info service helpers for UuvGuiNode."""

from __future__ import annotations

from .runtime import VehicleInfoGet


def request_vehicle_info(self) -> None:
    self._probe_backend()
    if not self._vehicle_info_supported:
        return
    if self._vehicle_info_in_flight:
        return
    try:
        ready = self._vehicle_info_client.service_is_ready()
    except Exception:
        return
    if not ready:
        return
    req = VehicleInfoGet.Request()
    req.sysid = 1
    req.compid = 1
    req.get_all = False
    future = self._vehicle_info_client.call_async(req)
    self._vehicle_info_in_flight = True
    future.add_done_callback(self._on_vehicle_info_response)


def _on_vehicle_info_response(self, future) -> None:
    self._vehicle_info_in_flight = False
    try:
        resp = future.result()
    except Exception as exc:
        self._push_event(f"vehicle_info_get failed: {exc}")
        return
    if not resp.success or not resp.vehicles:
        return
    info = resp.vehicles[0]
    with self._lock:
        self._snapshot.vehicle_mode = info.mode
        self._snapshot.mode_id = int(info.mode_id)
        self._snapshot.autopilot_name = f"autopilot={info.autopilot}, type={info.type}"
