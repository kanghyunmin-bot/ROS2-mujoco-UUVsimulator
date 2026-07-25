"""ExternalNav contract error predicates."""

from __future__ import annotations


def _raise_if_extnav_disabled_or_faulted(self) -> None:
    if not self._sitl_extnav_enabled:
        raise RuntimeError("ExternalNav contract required but ExternalNav output is disabled")
    if self._sitl_extnav_fault:
        raise RuntimeError(self._sitl_extnav_fault)


def _inside_extnav_grace(self, now_wall: float) -> bool:
    return now_wall - self._sitl_extnav_start_wall < self._sitl_extnav_grace_s


def _raise_if_extnav_never_sent(self) -> None:
    if self._sitl_extnav_last_send_wall <= 0.0:
        raise RuntimeError("ExternalNav contract required but no VISION_POSITION_DELTA has been sent")


def _raise_if_extnav_stale(self, now_wall: float) -> None:
    stale_s = now_wall - self._sitl_extnav_last_send_wall
    if stale_s > self._sitl_extnav_max_stale_s:
        raise RuntimeError(
            f"ExternalNav TX stale: {stale_s:.3f}s > {self._sitl_extnav_max_stale_s:.3f}s"
        )


__all__ = [
    "_inside_extnav_grace",
    "_raise_if_extnav_disabled_or_faulted",
    "_raise_if_extnav_never_sent",
    "_raise_if_extnav_stale",
]
