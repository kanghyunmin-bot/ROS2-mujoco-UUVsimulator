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
    from bridge import ros2_stereo_image
    from bridge.ros2_stereo_image import configure_stereo_image_runtime

    import numpy as np

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

    _check_stereo_left_render_is_shared(ros2_stereo_image, np)
    _check_pillow_jpeg_fallback(ros2_stereo_image, np)

    print("bridge_live_imports=PASS")
    return 0


def _check_stereo_left_render_is_shared(ros2_stereo_image, np) -> None:
    calls = {"left": 0}
    original_render = ros2_stereo_image.render_camera_rgb
    original_cv2 = ros2_stereo_image._CV2
    original_attempted = ros2_stereo_image._CV2_IMPORT_ATTEMPTED

    class _Image:
        class _Header:
            stamp = None
            frame_id = ""

        def __init__(self):
            self.header = self._Header()

    class _CompressedImage(_Image):
        pass

    class _Bridge:
        Image = _Image
        CompressedImage = _CompressedImage

    class _FakeCv2:
        COLOR_RGB2BGR = 1
        IMWRITE_JPEG_QUALITY = 2

        @staticmethod
        def cvtColor(rgb, _code):
            return rgb

        @staticmethod
        def imencode(_ext, _bgr, _params):
            return True, np.array([1, 2, 3], dtype=np.uint8)

    def fake_render(_bridge, camera_name, _data):
        if camera_name == "stereo_left":
            calls["left"] += 1
        return np.zeros((2, 3, 3), dtype=np.uint8)

    try:
        ros2_stereo_image.render_camera_rgb = fake_render
        ros2_stereo_image._CV2 = _FakeCv2
        ros2_stereo_image._CV2_IMPORT_ATTEMPTED = True
        builders = ros2_stereo_image.build_stereo_publish_builders(_Bridge(), object(), object())
        if builders["stereo_left_image"]() is None:
            raise AssertionError("left raw stereo builder returned no image")
        if builders["real_camera_raw"]() is None:
            raise AssertionError("real raw camera alias returned no image")
        if builders["real_camera_compressed"]() is None:
            raise AssertionError("real compressed camera alias returned no image")
        if calls["left"] != 1:
            raise AssertionError(f"stereo_left rendered {calls['left']} times; expected shared single render")
    finally:
        ros2_stereo_image.render_camera_rgb = original_render
        ros2_stereo_image._CV2 = original_cv2
        ros2_stereo_image._CV2_IMPORT_ATTEMPTED = original_attempted


def _check_pillow_jpeg_fallback(ros2_stereo_image, np) -> None:
    """OpenCV ABI가 깨진 배포 환경에서도 JPEG 토픽이 유지되는지 확인한다."""
    from io import BytesIO

    from PIL import Image

    original_cv2 = ros2_stereo_image._CV2
    original_attempted = ros2_stereo_image._CV2_IMPORT_ATTEMPTED

    class _Header:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

    class _CompressedImage:
        def __init__(self):
            self.header = _Header()
            self.format = ""
            self.data = None

    class _Bridge:
        CompressedImage = _CompressedImage
        _stereo_image_enabled = True
        _stereo_image_jpeg_quality = 75
        _stereo_image_warned = set()

    try:
        ros2_stereo_image._CV2 = None
        ros2_stereo_image._CV2_IMPORT_ATTEMPTED = True
        rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
        rgb[:, :, 0] = 220
        msg = ros2_stereo_image.build_stereo_compressed_image_msg_from_rgb(
            _Bridge(), "top_up", rgb, object()
        )
        if msg is None or msg.format != "jpeg" or len(msg.data) <= 1000:
            raise AssertionError("Pillow JPEG fallback returned no usable compressed image")
        decoded = Image.open(BytesIO(bytes(msg.data)))
        if decoded.size != (1280, 720):
            raise AssertionError(f"Pillow JPEG fallback size mismatch: {decoded.size}")
    finally:
        ros2_stereo_image._CV2 = original_cv2
        ros2_stereo_image._CV2_IMPORT_ATTEMPTED = original_attempted


if __name__ == "__main__":
    raise SystemExit(main())
