"""Future callbacks for GUI Trigger service calls."""

from __future__ import annotations


def trigger_service_ready(client) -> bool:
    try:
        return bool(client is not None and client.service_is_ready())
    except Exception:
        return False


def handle_trigger_future(owner, fut, label: str, on_success=None, on_done=None) -> None:
    try:
        resp = fut.result()
    except Exception as exc:
        owner._push_event(f"{label} failed: {exc}")
        if on_done is not None:
            on_done(False)
        return
    ok = bool(getattr(resp, "success", False))
    message = str(getattr(resp, "message", ""))
    owner._push_event(f"{label}: success={ok} {message}".strip())
    if on_done is not None:
        on_done(ok)
        return
    if ok and on_success is not None:
        on_success()


__all__ = ["handle_trigger_future", "trigger_service_ready"]
