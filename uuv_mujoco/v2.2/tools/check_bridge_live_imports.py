#!/usr/bin/env python3
"""Import smoke for bridge modules used by live SITL/GUI paths."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main() -> int:
    import bridge.sitl_arm_mode_queue  # noqa: F401
    import bridge.sitl_arm_mode_runtime  # noqa: F401
    import bridge.sitl_transport  # noqa: F401
    import bridge.ros2_bridge  # noqa: F401
    import bridge.ping360_sim  # noqa: F401
    from bridge.ros2_stereo_image import configure_stereo_image_runtime

    class _Global:
        offwidth = 640
        offheight = 480

    class _Vis:
        global_ = _Global()

    class _Model:
        vis = _Vis()

    class _Bridge:
        model = _Model()

    bridge = _Bridge()
    configure_stereo_image_runtime(
        bridge,
        publish_images=True,
        image_width=1280,
        image_height=720,
        image_hz=10.0,
    )
    if bridge.model.vis.global_.offwidth < 1280 or bridge.model.vis.global_.offheight < 720:
        raise AssertionError("stereo image runtime must expand MuJoCo offscreen framebuffer for 720p rendering")

    print("bridge_live_imports=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
