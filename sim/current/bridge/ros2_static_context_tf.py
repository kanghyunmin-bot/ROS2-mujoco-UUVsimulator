"""One-shot `/tf_static` publish policy for the ROS2 bridge."""

from __future__ import annotations

from typing import Any, Callable


def publish_static_tf_once(
    *,
    tf_static_pub: Any,
    build_tf_message: Callable[[Any, Any], Any],
    safe_publish: Callable[[Any, Any, str], bool],
    static_tf_specs: Any,
    stamp: Any,
    already_published: bool,
) -> tuple[bool, bool]:
    """Publish static TF once and return `(success, published_state)`."""
    if already_published or tf_static_pub is None:
        return True, bool(already_published)
    static_msg = build_tf_message(stamp, static_tf_specs)
    if static_msg is not None and not safe_publish(tf_static_pub, static_msg, "/tf_static"):
        return False, False
    return True, True


__all__ = ["publish_static_tf_once"]
