"""Optional ROS message type-support probing."""

from __future__ import annotations

from typing import Any, Callable


def optional_message_type(label: str, msg_cls: Any, *, log: Callable[[str], None] | None = None) -> Any:
    """Return a ROS message class only when its type support can be imported."""
    if msg_cls is None:
        return None
    try:
        import_type_support = getattr(msg_cls, "__import_type_support__", None)
        if callable(import_type_support):
            import_type_support()
        if getattr(msg_cls, "_TYPE_SUPPORT", None) is None:
            raise RuntimeError("typesupport is unavailable")
        return msg_cls
    except Exception as exc:
        if log is not None:
            log(f"[ros2] optional message {label} disabled: {exc}")
        return None


__all__ = ["optional_message_type"]
