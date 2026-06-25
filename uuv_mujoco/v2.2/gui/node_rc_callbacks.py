"""RC input and output callbacks for UuvGuiNode."""

from __future__ import annotations

from .node_rc_frame import padded_rc_channels


def _on_rc_out(self, msg: RCOut) -> None:
    self._touch("rc_out")
    with self._lock:
        self._snapshot.rc_out = padded_rc_channels(msg.channels)
        self._snapshot.rc_out_source = self._topic("rc/out")
        self._snapshot.rc_feedback_source = self._topic("rc/out")


def _on_rc_in(self, msg: RCIn) -> None:
    self._touch("rc_in")
    with self._lock:
        self._snapshot.rc_in = padded_rc_channels(
            getattr(msg, "channels", []),
            sanitize_override_markers=True,
        )
        self._snapshot.rc_in_source = self._topic("rc/in")
        self._snapshot.rc_feedback_source = self._topic("rc/in")


__all__ = ["_on_rc_in", "_on_rc_out"]
