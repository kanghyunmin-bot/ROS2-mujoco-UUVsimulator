#!/usr/bin/env python3
"""Offline ROS-boundary tests for modeled stereo camera publishing."""

from __future__ import annotations

import os
import tempfile
import threading
import time
import unittest
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge import ros2_stereo_image as camera  # noqa: E402
from bridge.ros2_static_tf_cameras import build_camera_frame_specs  # noqa: E402


class _Stamp:
    def __init__(self, sec: int = 0, nanosec: int = 0) -> None:
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self) -> None:
        self.stamp = _Stamp()
        self.frame_id = ""


class _Image:
    def __init__(self) -> None:
        self.header = _Header()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = 0
        self.step = 0
        self.data = None


class _CompressedImage:
    def __init__(self) -> None:
        self.header = _Header()
        self.format = ""
        self.data = None


class _CameraInfo:
    def __init__(self) -> None:
        self.header = _Header()
        self.width = 0
        self.height = 0
        self.distortion_model = ""
        self.d = []
        self.k = []
        self.r = []
        self.p = []


def _bridge(*, width: int = 64, height: int = 64, hz: float = 10.0):
    owner = SimpleNamespace(
        model=None,
        node=None,
        Image=_Image,
        CompressedImage=_CompressedImage,
        CameraInfo=_CameraInfo,
        _cam_left_id=-1,
        _cam_right_id=-1,
    )
    camera.configure_stereo_image_runtime(
        owner,
        publish_images=True,
        image_width=width,
        image_height=height,
        image_hz=hz,
    )
    return owner


def _rgb(value: int = 120) -> np.ndarray:
    return np.full((64, 64, 3), value, dtype=np.uint8)


class _FakeCv2:
    COLOR_RGB2BGR = 1
    IMWRITE_JPEG_QUALITY = 2

    @staticmethod
    def cvtColor(rgb, _mode):
        return rgb[:, :, ::-1]

    @staticmethod
    def imencode(_suffix, bgr, _params):
        return True, np.frombuffer(b"fake-jpeg" + bytes([int(bgr[0, 0, 0])]), dtype=np.uint8)


class CameraPublishIntegrationTest(unittest.TestCase):
    def tearDown(self) -> None:
        os.environ.pop("ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE", None)
        os.environ.pop("ROS2_UUV_CAMERA_PROCESSING_LATENCY_S", None)
        os.environ.pop("ROS2_UUV_CAMERA_PROCESSING_JITTER_S", None)
        os.environ.pop("ROS2_UUV_CAMERA_TRANSPORT_LATENCY_S", None)
        os.environ.pop("ROS2_UUV_CAMERA_TRANSPORT_JITTER_S", None)
        os.environ.pop("ROS2_UUV_CAMERA_FRAME_DROPOUT_PROBABILITY", None)
        os.environ.pop("ROS2_UUV_ASYNC_CAMERA_RENDER", None)

    def test_disabled_path_preserves_legacy_pixels_headers_and_camera_info(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "0"
        os.environ["ROS2_UUV_ASYNC_CAMERA_RENDER"] = "0"
        owner = _bridge()
        source = _rgb(91)
        stamp = _Stamp(4, 123)
        data = SimpleNamespace(time=4.0)

        with patch.object(
            camera,
            "_legacy_camera_frame_for_publish",
            return_value=(source, 4.0, None),
        ):
            builders = camera.build_stereo_publish_builders(owner, data, stamp)
            left = builders["stereo_left_image"]()
            raw = builders["real_camera_raw"]()
            info = builders["real_camera_info"]()
            imx0_raw = builders["imx219_camera0_raw"]()
            imx0_info = builders["imx219_camera0_info"]()
            imx1_raw = builders["imx219_camera1_raw"]()
            imx1_info = builders["imx219_camera1_info"]()

        self.assertFalse(owner._camera_sensor_model_enabled)
        self.assertEqual(bytes(left.data), source.tobytes())
        self.assertEqual(bytes(raw.data), source.tobytes())
        self.assertIs(left.header.stamp, stamp)
        self.assertIs(raw.header.stamp, stamp)
        self.assertEqual(left.header.frame_id, "stereo_left_optical")
        self.assertEqual(raw.header.frame_id, camera.REAL_CAMERA_OPTICAL_FRAME)
        self.assertEqual(info.header.frame_id, camera.REAL_CAMERA_OPTICAL_FRAME)
        self.assertEqual(info.d, [0.0] * 5)
        self.assertEqual(info.r, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(bytes(imx0_raw.data), source.tobytes())
        self.assertEqual(bytes(imx1_raw.data), source.tobytes())
        self.assertEqual(imx0_raw.header.frame_id, camera.IMX219_CAMERA0_OPTICAL_FRAME)
        self.assertEqual(imx0_raw.encoding, "bgr8")
        self.assertEqual(imx0_info.header.frame_id, camera.IMX219_CAMERA0_OPTICAL_FRAME)
        self.assertEqual(imx1_raw.header.frame_id, camera.IMX219_CAMERA1_OPTICAL_FRAME)
        self.assertEqual(imx1_raw.encoding, "bgr8")
        self.assertEqual(imx1_info.header.frame_id, camera.IMX219_CAMERA1_OPTICAL_FRAME)
        for message in (imx0_raw, imx0_info, imx1_raw, imx1_info):
            self.assertIs(message.header.stamp, stamp)
        camera.close_stereo_image_renderers(owner)

    def test_enabled_model_keeps_message_contract_and_synchronizes_capture_stamp(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "1"
        os.environ["ROS2_UUV_CAMERA_PROCESSING_LATENCY_S"] = "0"
        os.environ["ROS2_UUV_CAMERA_PROCESSING_JITTER_S"] = "0"
        os.environ["ROS2_UUV_CAMERA_TRANSPORT_LATENCY_S"] = "0"
        os.environ["ROS2_UUV_CAMERA_TRANSPORT_JITTER_S"] = "0"
        os.environ["ROS2_UUV_ASYNC_CAMERA_RENDER"] = "0"
        owner = _bridge()
        source = _rgb(140)
        data = SimpleNamespace(time=2.25)
        stamp = _Stamp(2, 250_000_000)

        with (
            patch.object(camera, "_render_camera_rgb_or_none", return_value=source),
            patch.object(camera, "_load_cv2", return_value=_FakeCv2()),
        ):
            builders = camera.build_stereo_publish_builders(owner, data, stamp)
            left = builders["stereo_left_image"]()
            raw = builders["real_camera_raw"]()
            compressed = builders["real_camera_compressed"]()
            info = builders["real_camera_info"]()
            imx0_raw = builders["imx219_camera0_raw"]()
            imx0_compressed = builders["imx219_camera0_compressed"]()
            imx0_info = builders["imx219_camera0_info"]()
            imx1_raw = builders["imx219_camera1_raw"]()
            imx1_compressed = builders["imx219_camera1_compressed"]()
            imx1_info = builders["imx219_camera1_info"]()

        self.assertTrue(owner._camera_sensor_model_enabled)
        self.assertNotEqual(bytes(left.data), source.tobytes())
        self.assertEqual(bytes(left.data), bytes(raw.data))
        self.assertEqual(left.encoding, "rgb8")
        self.assertEqual(left.step, 64 * 3)
        self.assertEqual(left.header.frame_id, "stereo_left_optical")
        self.assertEqual(raw.header.frame_id, camera.REAL_CAMERA_OPTICAL_FRAME)
        self.assertEqual(compressed.header.frame_id, camera.REAL_CAMERA_OPTICAL_FRAME)
        self.assertEqual(compressed.format, "jpeg")
        for message in (
            left,
            raw,
            compressed,
            info,
            imx0_raw,
            imx0_compressed,
            imx0_info,
            imx1_raw,
            imx1_compressed,
            imx1_info,
        ):
            self.assertEqual(message.header.stamp.sec, 2)
            self.assertEqual(message.header.stamp.nanosec, 250_000_000)
        self.assertEqual(imx0_raw.header.frame_id, camera.IMX219_CAMERA0_OPTICAL_FRAME)
        self.assertEqual(imx0_raw.encoding, "bgr8")
        self.assertEqual(imx0_compressed.header.frame_id, camera.IMX219_CAMERA0_OPTICAL_FRAME)
        self.assertEqual(imx0_compressed.format, "bgr8; jpeg compressed bgr8")
        self.assertEqual(imx0_info.header.frame_id, camera.IMX219_CAMERA0_OPTICAL_FRAME)
        self.assertEqual(imx1_raw.header.frame_id, camera.IMX219_CAMERA1_OPTICAL_FRAME)
        self.assertEqual(imx1_raw.encoding, "bgr8")
        self.assertEqual(imx1_compressed.header.frame_id, camera.IMX219_CAMERA1_OPTICAL_FRAME)
        self.assertEqual(imx1_compressed.format, "bgr8; jpeg compressed bgr8")
        self.assertEqual(imx1_info.header.frame_id, camera.IMX219_CAMERA1_OPTICAL_FRAME)
        self.assertEqual(info.distortion_model, "plumb_bob")
        self.assertAlmostEqual(info.k[0], 514.13374 * (64 / 1280), places=6)
        self.assertEqual(owner._stereo_image_jpeg_quality, 95)
        camera.close_stereo_image_renderers(owner)

    def test_sensor_model_forces_sim_time_deterministic_sync_render(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "1"
        os.environ["ROS2_UUV_ASYNC_CAMERA_RENDER"] = "1"
        owner = _bridge()

        self.assertTrue(owner._stereo_image_async_requested)
        self.assertFalse(owner._stereo_image_async_enabled)
        self.assertTrue(owner._camera_sensor_model_enabled)
        camera.close_stereo_image_renderers(owner)

    def test_imx219_raw_alias_matches_real_driver_bgr8_payload(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "0"
        os.environ["ROS2_UUV_ASYNC_CAMERA_RENDER"] = "0"
        owner = _bridge()
        source = _rgb(0)
        source[:, :, 0] = 11
        source[:, :, 1] = 22
        source[:, :, 2] = 33
        with patch.object(
            camera,
            "_legacy_camera_frame_for_publish",
            return_value=(source, 3.0, None),
        ):
            builders = camera.build_stereo_publish_builders(
                owner,
                SimpleNamespace(time=3.0),
                _Stamp(3, 0),
            )
            legacy = builders["stereo_left_image"]()
            imx = builders["imx219_camera0_raw"]()

        self.assertEqual(legacy.encoding, "rgb8")
        self.assertEqual(bytes(legacy.data[:3]), bytes((11, 22, 33)))
        self.assertEqual(imx.encoding, "bgr8")
        self.assertEqual(bytes(imx.data[:3]), bytes((33, 22, 11)))
        camera.close_stereo_image_renderers(owner)

    def test_latency_publishes_previous_frame_with_capture_not_arrival_stamp(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "1"
        os.environ["ROS2_UUV_CAMERA_PROCESSING_LATENCY_S"] = "0.05"
        os.environ["ROS2_UUV_CAMERA_PROCESSING_JITTER_S"] = "0"
        os.environ["ROS2_UUV_CAMERA_TRANSPORT_LATENCY_S"] = "0"
        os.environ["ROS2_UUV_CAMERA_TRANSPORT_JITTER_S"] = "0"
        os.environ["ROS2_UUV_ASYNC_CAMERA_RENDER"] = "0"
        owner = _bridge()

        with patch.object(camera, "_render_camera_rgb_or_none", return_value=_rgb(100)):
            initial = camera.build_stereo_publish_builders(
                owner,
                SimpleNamespace(time=1.0),
                _Stamp(1, 0),
            )
            self.assertIsNone(initial["real_camera_raw"]())

        with patch.object(camera, "_render_camera_rgb_or_none", return_value=_rgb(200)):
            delayed = camera.build_stereo_publish_builders(
                owner,
                SimpleNamespace(time=1.1),
                _Stamp(1, 100_000_000),
            )
            raw = delayed["real_camera_raw"]()
            info = delayed["real_camera_info"]()

        self.assertEqual(raw.header.stamp.sec, 1)
        self.assertEqual(raw.header.stamp.nanosec, 0)
        self.assertEqual(info.header.stamp.nanosec, 0)
        camera.close_stereo_image_renderers(owner)

    def test_legacy_async_alias_uses_atomic_frame_capture_stamp(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "0"
        owner = _bridge()
        owner._stereo_image_async_enabled = True
        source = _rgb(73)
        with owner._stereo_image_async_lock:
            owner._stereo_image_async_latest["stereo_left"] = source
            owner._stereo_image_async_latest_capture_time["stereo_left"] = 1.25
            owner._stereo_image_async_latest_jpeg["stereo_left"] = b"captured-jpeg"

        with patch.object(camera, "_submit_async_camera_render", return_value=source):
            builders = camera.build_stereo_publish_builders(
                owner,
                SimpleNamespace(time=2.0),
                _Stamp(2, 0),
            )
            raw = builders["imx219_camera0_raw"]()
            info = builders["imx219_camera0_info"]()
            compressed = builders["real_camera_compressed"]()
            imx_compressed = builders["imx219_camera0_compressed"]()

        self.assertEqual(bytes(raw.data), source.tobytes())
        for message in (raw, info, compressed, imx_compressed):
            self.assertEqual(message.header.stamp.sec, 1)
            self.assertEqual(message.header.stamp.nanosec, 250_000_000)
        self.assertEqual(bytes(compressed.data), b"captured-jpeg")
        self.assertEqual(bytes(imx_compressed.data), b"captured-jpeg")
        self.assertEqual(imx_compressed.format, "bgr8; jpeg compressed bgr8")
        self.assertEqual(
            imx_compressed.header.frame_id,
            camera.IMX219_CAMERA0_OPTICAL_FRAME,
        )
        camera.close_stereo_image_renderers(owner)

    def test_dropout_suppresses_image_and_matching_camera_info(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "1"
        os.environ["ROS2_UUV_CAMERA_FRAME_DROPOUT_PROBABILITY"] = "1"
        os.environ["ROS2_UUV_CAMERA_PROCESSING_LATENCY_S"] = "0"
        os.environ["ROS2_UUV_CAMERA_TRANSPORT_LATENCY_S"] = "0"
        os.environ["ROS2_UUV_ASYNC_CAMERA_RENDER"] = "0"
        owner = _bridge()

        with patch.object(camera, "_render_camera_rgb_or_none", return_value=_rgb()):
            builders = camera.build_stereo_publish_builders(
                owner,
                SimpleNamespace(time=0.0),
                _Stamp(),
            )
            self.assertIsNone(builders["stereo_left_image"]())
            self.assertIsNone(builders["real_camera_raw"]())
            self.assertIsNone(builders["real_camera_info"]())
            self.assertIsNone(builders["imx219_camera0_raw"]())
            self.assertIsNone(builders["imx219_camera0_compressed"]())
            self.assertIsNone(builders["imx219_camera0_info"]())
            self.assertIsNone(builders["imx219_camera1_raw"]())
            self.assertIsNone(builders["imx219_camera1_compressed"]())
            self.assertIsNone(builders["imx219_camera1_info"]())

        self.assertEqual(
            owner._camera_sensor_runtimes["stereo_left"].stats.probabilistic_drops,
            1,
        )
        self.assertEqual(
            owner._camera_sensor_runtimes["stereo_right"].stats.probabilistic_drops,
            1,
        )
        camera.close_stereo_image_renderers(owner)

    def test_real_imx219_publishers_use_exact_topics_and_sensor_qos(self) -> None:
        calls: list[tuple[object, str, object]] = []

        class _Node:
            @staticmethod
            def create_publisher(message_type, topic, qos):
                calls.append((message_type, topic, qos))
                return SimpleNamespace(topic=topic)

        sensor_qos = object()
        owner = SimpleNamespace(
            node=_Node(),
            Image=_Image,
            CompressedImage=_CompressedImage,
            CameraInfo=_CameraInfo,
        )
        camera.create_stereo_image_publishers(owner, camera_sensor_qos=sensor_qos)

        indexed = {topic: (message_type, qos) for message_type, topic, qos in calls}
        expected = {
            camera.IMX219_CAMERA0_RAW_TOPIC: _Image,
            camera.IMX219_CAMERA0_COMPRESSED_TOPIC: _CompressedImage,
            camera.IMX219_CAMERA0_INFO_TOPIC: _CameraInfo,
            camera.IMX219_CAMERA1_RAW_TOPIC: _Image,
            camera.IMX219_CAMERA1_COMPRESSED_TOPIC: _CompressedImage,
            camera.IMX219_CAMERA1_INFO_TOPIC: _CameraInfo,
        }
        for topic, message_type in expected.items():
            self.assertIn(topic, indexed)
            self.assertIs(indexed[topic][0], message_type)
            self.assertIs(indexed[topic][1], sensor_qos)

    def test_async_jpeg_failure_clears_previous_frame_payload(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "0"
        owner = _bridge()
        owner._stereo_image_async_enabled = True
        owner._stereo_image_async_latest_jpeg["stereo_left"] = b"stale-jpeg"
        owner._stereo_image_async_pending["stereo_left"] = SimpleNamespace(time=1.0)
        owner._stereo_image_async_event.set()

        with (
            patch.object(camera, "_render_camera_rgb_or_none", return_value=_rgb(55)),
            patch.object(camera, "_encode_camera_jpeg", return_value=None),
        ):
            thread = threading.Thread(
                target=camera._async_camera_worker,
                args=(owner,),
                daemon=True,
            )
            thread.start()
            deadline = time.monotonic() + 1.0
            while "stereo_left" not in owner._stereo_image_async_latest and time.monotonic() < deadline:
                time.sleep(0.005)
            owner._stereo_image_async_stop.set()
            owner._stereo_image_async_event.set()
            thread.join(timeout=1.0)

        self.assertIn("stereo_left", owner._stereo_image_async_latest_jpeg)
        self.assertIsNone(owner._stereo_image_async_latest_jpeg["stereo_left"])
        with patch.object(camera, "_submit_async_camera_render", return_value=_rgb(55)):
            builders = camera.build_stereo_publish_builders(
                owner,
                SimpleNamespace(time=2.0),
                _Stamp(2, 0),
            )
            self.assertIsNone(builders["imx219_camera0_compressed"]())
        camera.close_stereo_image_renderers(owner)

    def test_real_imx219_jobs_use_configured_rate_and_exact_builders(self) -> None:
        owner = SimpleNamespace(
            _stereo_image_enabled=True,
            _stereo_image_hz=30.0,
            pub_stereo_left_image=object(),
            pub_stereo_right_image=object(),
            pub_real_camera_raw=object(),
            pub_real_camera_compressed=object(),
            pub_real_camera_info=object(),
            pub_imx219_camera0_raw=object(),
            pub_imx219_camera0_compressed=object(),
            pub_imx219_camera0_info=object(),
            pub_imx219_camera1_raw=object(),
            pub_imx219_camera1_compressed=object(),
            pub_imx219_camera1_info=object(),
        )
        builders = {
            "stereo_left_image": object(),
            "stereo_right_image": object(),
            "real_camera_raw": object(),
            "real_camera_compressed": object(),
            "real_camera_info": object(),
            "imx219_camera0_raw": object(),
            "imx219_camera0_compressed": object(),
            "imx219_camera0_info": object(),
            "imx219_camera1_raw": object(),
            "imx219_camera1_compressed": object(),
            "imx219_camera1_info": object(),
        }
        calls: list[tuple[object, str, object, float, bool]] = []

        def add_rate_limited(publisher, topic, builder, hz, *, on_demand=False):
            calls.append((publisher, topic, builder, hz, on_demand))

        camera.schedule_stereo_image_jobs(
            owner,
            jobs=None,
            add_rate_limited=add_rate_limited,
            builders=builders,
        )
        indexed = {
            topic: (publisher, builder, hz, on_demand)
            for publisher, topic, builder, hz, on_demand in calls
        }
        expected = {
            camera.IMX219_CAMERA0_RAW_TOPIC: (
                owner.pub_imx219_camera0_raw,
                builders["imx219_camera0_raw"],
            ),
            camera.IMX219_CAMERA0_INFO_TOPIC: (
                owner.pub_imx219_camera0_info,
                builders["imx219_camera0_info"],
            ),
            camera.IMX219_CAMERA0_COMPRESSED_TOPIC: (
                owner.pub_imx219_camera0_compressed,
                builders["imx219_camera0_compressed"],
            ),
            camera.IMX219_CAMERA1_RAW_TOPIC: (
                owner.pub_imx219_camera1_raw,
                builders["imx219_camera1_raw"],
            ),
            camera.IMX219_CAMERA1_INFO_TOPIC: (
                owner.pub_imx219_camera1_info,
                builders["imx219_camera1_info"],
            ),
            camera.IMX219_CAMERA1_COMPRESSED_TOPIC: (
                owner.pub_imx219_camera1_compressed,
                builders["imx219_camera1_compressed"],
            ),
        }
        for topic, (publisher, builder) in expected.items():
            self.assertIn(topic, indexed)
            self.assertIs(indexed[topic][0], publisher)
            self.assertIs(indexed[topic][1], builder)
            self.assertEqual(indexed[topic][2], 30.0)
            self.assertTrue(indexed[topic][3])

    def test_real_imx219_optical_frames_have_single_static_tf_parent(self) -> None:
        specs = build_camera_frame_specs(
            model=SimpleNamespace(),
            cam_left_site_id=-1,
            cam_right_site_id=-1,
            zero=np.zeros(3, dtype=np.float64),
            ident=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            optical_quat=np.array([0.5, -0.5, 0.5, -0.5], dtype=np.float64),
        )
        parents = {child: parent for parent, child, _position, _quaternion in specs}
        self.assertEqual(parents[camera.IMX219_CAMERA0_OPTICAL_FRAME], "stereo_left")
        self.assertEqual(parents[camera.IMX219_CAMERA1_OPTICAL_FRAME], "stereo_right")

    def test_supplied_calibration_controls_camera_info_while_model_is_disabled(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "0"
        calibration_text = """
image_width: 640
image_height: 480
camera_matrix:
  data: [500, 0, 319.5, 0, 501, 239.5, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  data: [-0.1, 0.02, 0.001, -0.002, 0]
rectification_matrix:
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  data: [500, 0, 319.5, 0, 0, 501, 239.5, 0, 0, 0, 1, 0]
"""
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "left.yaml"
            path.write_text(calibration_text, encoding="utf-8")
            owner = SimpleNamespace(
                model=None,
                node=None,
                Image=_Image,
                CompressedImage=_CompressedImage,
                CameraInfo=_CameraInfo,
                _cam_left_id=-1,
                _cam_right_id=-1,
            )
            camera.configure_stereo_image_runtime(
                owner,
                publish_images=True,
                image_width=320,
                image_height=240,
                image_hz=10.0,
                camera_calib_left=str(path),
            )
            info = camera.build_camera_info_msg(owner, _Stamp())

        self.assertEqual(info.d, [-0.1, 0.02, 0.001, -0.002, 0.0])
        self.assertAlmostEqual(info.k[0], 250.0)
        self.assertAlmostEqual(info.k[4], 250.5)
        camera.close_stereo_image_renderers(owner)

    def test_close_is_idempotent_and_releases_bounded_runtime_images(self) -> None:
        os.environ["ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE"] = "1"
        os.environ["ROS2_UUV_CAMERA_PROCESSING_LATENCY_S"] = "1"
        os.environ["ROS2_UUV_ASYNC_CAMERA_RENDER"] = "0"
        owner = _bridge()
        runtime = owner._camera_sensor_runtimes["stereo_left"]
        runtime.advance(
            0.0,
            camera.RenderedCameraFrame(rgb=_rgb(), capture_time_s=0.0),
        )
        self.assertEqual(runtime.transport.pending_count, 1)

        camera.close_stereo_image_renderers(owner)
        camera.close_stereo_image_renderers(owner)

        self.assertEqual(runtime.transport.pending_count, 0)
        self.assertEqual(owner._camera_sensor_runtimes, {})
        self.assertEqual(owner._stereo_image_async_latest, {})


if __name__ == "__main__":
    unittest.main(verbosity=2)
