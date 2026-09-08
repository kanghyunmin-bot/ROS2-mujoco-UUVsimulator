"""Stereo camera image publishing helpers for the ROS2 bridge."""

from __future__ import annotations

import os
import threading
from array import array
from typing import Any

import mujoco
import numpy as np

from .ros2_bridge_publish_stamp import stamp_from_seconds_like
from .ros2_camera_sensor_runtime import (
    CameraFrameDelivery,
    CameraSensorRuntime,
    RenderedCameraFrame,
)
from .underwater_camera_sensor_model import (
    DEFAULT_CAMERA_PROFILE_PATH,
    CameraCalibration,
    load_camera_calibration,
    load_underwater_camera_profile,
)


CAMERA_NAMES = ("stereo_left", "stereo_right")
DEFAULT_IMAGE_WIDTH = 960
DEFAULT_IMAGE_HEIGHT = 540
DEFAULT_IMAGE_HZ = 30.0
DEFAULT_COMPRESSED_JPEG_QUALITY = 95
REAL_CAMERA_RAW_TOPIC = "/camera/camera/color/image_raw"
REAL_CAMERA_COMPRESSED_TOPIC = "/camera/camera/color/image_raw/compressed"
REAL_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
REAL_CAMERA_OPTICAL_FRAME = "camera_color_optical_frame"
IMX219_CAMERA0_RAW_TOPIC = "/imx219/camera0/image_raw"
IMX219_CAMERA0_COMPRESSED_TOPIC = "/imx219/camera0/image_raw/compressed"
IMX219_CAMERA0_INFO_TOPIC = "/imx219/camera0/camera_info"
IMX219_CAMERA0_OPTICAL_FRAME = "imx219_camera0_optical_frame"
IMX219_CAMERA1_RAW_TOPIC = "/imx219/camera1/image_raw"
IMX219_CAMERA1_COMPRESSED_TOPIC = "/imx219/camera1/image_raw/compressed"
IMX219_CAMERA1_INFO_TOPIC = "/imx219/camera1/camera_info"
IMX219_CAMERA1_OPTICAL_FRAME = "imx219_camera1_optical_frame"
_CV2: Any | None = None
_CV2_IMPORT_ATTEMPTED = False


def configure_stereo_image_runtime(
    bridge: Any,
    *,
    publish_images: bool,
    image_width: int,
    image_height: int,
    image_hz: float,
    camera_calib_left: str = "",
    camera_calib_right: str = "",
) -> None:
    bridge._stereo_image_enabled = bool(publish_images)
    bridge._stereo_image_width = int(np.clip(int(image_width), 64, 1920))
    bridge._stereo_image_height = int(np.clip(int(image_height), 64, 1080))
    bridge._stereo_image_hz = float(np.clip(float(image_hz), 0.1, 60.0))
    bridge._stereo_image_jpeg_quality = _env_int(
        "ROS2_UUV_CAMERA_JPEG_QUALITY",
        DEFAULT_COMPRESSED_JPEG_QUALITY,
        minimum=40,
        maximum=95,
    )
    bridge._stereo_image_renderers: dict[str, Any] = {}
    bridge._stereo_image_warned: set[str] = set()
    bridge._stereo_image_async_requested = _env_bool(
        "ROS2_UUV_ASYNC_CAMERA_RENDER", True
    )
    bridge._stereo_image_async_enabled = bridge._stereo_image_async_requested
    bridge._stereo_image_async_lock = threading.Lock()
    bridge._stereo_image_async_event = threading.Event()
    bridge._stereo_image_async_stop = threading.Event()
    bridge._stereo_image_async_pending: dict[str, Any] = {}
    # MjData owns a large MuJoCo arena in the competition scene.  Creating and
    # destroying a complete snapshot for every camera frame causes allocator
    # churn and leaves hundreds of megabytes resident.  At most one frame can
    # be pending while one is rendered, so a two-buffer pool per camera is a
    # strict upper bound and preserves the latest-frame/drop-old policy.
    bridge._stereo_image_async_free: dict[str, list[Any]] = {}
    bridge._stereo_image_async_latest: dict[str, np.ndarray] = {}
    bridge._stereo_image_async_latest_capture_time: dict[str, float] = {}
    bridge._stereo_image_async_latest_jpeg: dict[str, bytes | None] = {}
    bridge._stereo_image_async_thread = None
    _configure_underwater_camera_sensor_model(
        bridge,
        image_hz=bridge._stereo_image_hz,
        camera_calib_left=camera_calib_left,
        camera_calib_right=camera_calib_right,
    )
    # The single-slot async renderer accepts or skips snapshots according to
    # wall-thread completion timing. That is useful for an interactive legacy
    # view, but it changes which sim-time frame reaches the stochastic sensor
    # sequence. Research sensor-model runs therefore render synchronously so a
    # fixed simulation input and seed have a deterministic capture sequence.
    if bridge._camera_sensor_model_enabled:
        bridge._stereo_image_async_enabled = False
    _ensure_offscreen_buffer(bridge, bridge._stereo_image_width, bridge._stereo_image_height)


def _configure_underwater_camera_sensor_model(
    bridge: Any,
    *,
    image_hz: float,
    camera_calib_left: str,
    camera_calib_right: str,
) -> None:
    bridge._camera_sensor_model_enabled = _env_bool(
        "ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE",
        False,
    )
    bridge._camera_sensor_profile = None
    bridge._camera_calibrations: dict[str, CameraCalibration] = {}
    bridge._camera_sensor_runtimes: dict[str, CameraSensorRuntime] = {}

    calibration_paths = {
        "stereo_left": str(camera_calib_left).strip(),
        "stereo_right": str(camera_calib_right).strip(),
    }
    for camera_name, path in calibration_paths.items():
        if path:
            bridge._camera_calibrations[camera_name] = load_camera_calibration(
                path,
                fallback_width=bridge._stereo_image_width,
                fallback_height=bridge._stereo_image_height,
            )

    if not bridge._camera_sensor_model_enabled:
        return

    configured_path = str(
        os.environ.get("ROS2_UUV_CAMERA_SENSOR_MODEL_CONFIG", DEFAULT_CAMERA_PROFILE_PATH)
    ).strip()
    profile = load_underwater_camera_profile(configured_path or DEFAULT_CAMERA_PROFILE_PATH)
    bridge._camera_sensor_profile = profile
    seed = _env_optional_int("ROS2_UUV_CAMERA_SENSOR_MODEL_SEED")
    runtime_overrides = {
        "dropout_probability": _env_optional_float(
            "ROS2_UUV_CAMERA_FRAME_DROPOUT_PROBABILITY"
        ),
        "processing_latency_mean_s": _env_optional_float(
            "ROS2_UUV_CAMERA_PROCESSING_LATENCY_S"
        ),
        "processing_latency_jitter_s": _env_optional_float(
            "ROS2_UUV_CAMERA_PROCESSING_JITTER_S"
        ),
        "transport_latency_mean_s": _env_optional_float(
            "ROS2_UUV_CAMERA_TRANSPORT_LATENCY_S"
        ),
        "transport_latency_jitter_s": _env_optional_float(
            "ROS2_UUV_CAMERA_TRANSPORT_JITTER_S"
        ),
    }
    for camera_name in CAMERA_NAMES:
        calibration = bridge._camera_calibrations.get(camera_name, profile.calibration)
        bridge._camera_calibrations[camera_name] = calibration
        bridge._camera_sensor_runtimes[camera_name] = CameraSensorRuntime(
            profile,
            camera_name=camera_name,
            calibration=calibration,
            frame_rate_hz=float(image_hz),
            seed=seed,
            **runtime_overrides,
        )


def create_stereo_image_publishers(bridge: Any, *, camera_sensor_qos: Any) -> None:
    node = bridge.node
    bridge.pub_stereo_left_image = node.create_publisher(
        bridge.Image, "/stereo/left/image_raw", camera_sensor_qos
    )
    bridge.pub_stereo_right_image = node.create_publisher(
        bridge.Image, "/stereo/right/image_raw", camera_sensor_qos
    )
    bridge.pub_real_camera_raw = node.create_publisher(
        bridge.Image,
        REAL_CAMERA_RAW_TOPIC,
        camera_sensor_qos,
    )
    bridge.pub_real_camera_compressed = node.create_publisher(
        bridge.CompressedImage,
        REAL_CAMERA_COMPRESSED_TOPIC,
        camera_sensor_qos,
    )
    bridge.pub_real_camera_info = node.create_publisher(
        bridge.CameraInfo,
        REAL_CAMERA_INFO_TOPIC,
        camera_sensor_qos,
    )
    bridge.pub_imx219_camera0_raw = node.create_publisher(
        bridge.Image,
        IMX219_CAMERA0_RAW_TOPIC,
        camera_sensor_qos,
    )
    bridge.pub_imx219_camera0_compressed = node.create_publisher(
        bridge.CompressedImage,
        IMX219_CAMERA0_COMPRESSED_TOPIC,
        camera_sensor_qos,
    )
    bridge.pub_imx219_camera0_info = node.create_publisher(
        bridge.CameraInfo,
        IMX219_CAMERA0_INFO_TOPIC,
        camera_sensor_qos,
    )
    bridge.pub_imx219_camera1_raw = node.create_publisher(
        bridge.Image,
        IMX219_CAMERA1_RAW_TOPIC,
        camera_sensor_qos,
    )
    bridge.pub_imx219_camera1_compressed = node.create_publisher(
        bridge.CompressedImage,
        IMX219_CAMERA1_COMPRESSED_TOPIC,
        camera_sensor_qos,
    )
    bridge.pub_imx219_camera1_info = node.create_publisher(
        bridge.CameraInfo,
        IMX219_CAMERA1_INFO_TOPIC,
        camera_sensor_qos,
    )


def build_stereo_publish_builders(self: Any, data: Any, stamp: Any) -> dict[str, object]:
    rgb_cache: dict[str, np.ndarray | None] = {}
    left_image_cache: dict[str, Any] = {}
    modeled_delivery_cache: dict[str, CameraFrameDelivery | None] = {}
    legacy_frame_cache: dict[str, tuple[np.ndarray | None, float | None, bytes | None]] = {}

    def modeled_delivery(camera_name: str) -> CameraFrameDelivery | None:
        if camera_name not in modeled_delivery_cache:
            modeled_delivery_cache[camera_name] = _modeled_camera_delivery_for_publish(
                self,
                camera_name,
                data,
            )
        return modeled_delivery_cache[camera_name]

    def camera_rgb(camera_name: str) -> np.ndarray | None:
        if camera_name not in rgb_cache:
            if bool(getattr(self, "_camera_sensor_model_enabled", False)):
                delivery = modeled_delivery(camera_name)
                rgb_cache[camera_name] = None if delivery is None else delivery.rgb
            else:
                rgb_cache[camera_name] = legacy_frame(camera_name)[0]
        return rgb_cache[camera_name]

    def legacy_frame(
        camera_name: str,
    ) -> tuple[np.ndarray | None, float | None, bytes | None]:
        if camera_name not in legacy_frame_cache:
            legacy_frame_cache[camera_name] = _legacy_camera_frame_for_publish(
                self,
                camera_name,
                data,
            )
        return legacy_frame_cache[camera_name]

    def camera_stamp(camera_name: str) -> Any:
        if not bool(getattr(self, "_camera_sensor_model_enabled", False)):
            if bool(getattr(self, "_stereo_image_async_enabled", False)):
                capture_time_s = legacy_frame(camera_name)[1]
                if capture_time_s is not None:
                    return stamp_from_seconds_like(stamp, capture_time_s)
            return stamp
        delivery = modeled_delivery(camera_name)
        if delivery is None:
            return stamp
        return stamp_from_seconds_like(stamp, delivery.capture_time_s)

    def build_left_image() -> Any | None:
        if "msg" not in left_image_cache:
            left_image_cache["msg"] = build_stereo_image_msg_from_rgb(
                self,
                "stereo_left",
                camera_rgb("stereo_left"),
                camera_stamp("stereo_left"),
            )
        return left_image_cache["msg"]

    def build_right_image() -> Any | None:
        return build_stereo_image_msg_from_rgb(
            self,
            "stereo_right",
            camera_rgb("stereo_right"),
            camera_stamp("stereo_right"),
        )

    def build_real_camera_raw() -> Any | None:
        return build_stereo_image_msg_from_rgb(
            self,
            "stereo_left",
            camera_rgb("stereo_left"),
            camera_stamp("stereo_left"),
            frame_id=REAL_CAMERA_OPTICAL_FRAME,
        )

    def build_real_camera_compressed() -> Any | None:
        rgb = camera_rgb("stereo_left")
        camera_header_stamp = camera_stamp("stereo_left")
        if bool(getattr(self, "_camera_sensor_model_enabled", False)):
            return build_stereo_compressed_image_msg_from_rgb(
                self,
                "stereo_left",
                rgb,
                camera_header_stamp,
            )
        if bool(getattr(self, "_stereo_image_async_enabled", False)):
            encoded = legacy_frame("stereo_left")[2]
            return build_stereo_compressed_image_msg_from_jpeg(
                self, "stereo_left", encoded, camera_header_stamp
            )
        return build_stereo_compressed_image_msg_from_rgb(
            self, "stereo_left", rgb, camera_header_stamp
        )

    def build_real_camera_info() -> Any | None:
        if bool(getattr(self, "_camera_sensor_model_enabled", False)):
            if modeled_delivery("stereo_left") is None:
                return None
        return build_camera_info_msg(
            self,
            camera_stamp("stereo_left"),
            camera_name="stereo_left",
        )

    def build_imx219_camera0_raw() -> Any | None:
        return build_stereo_image_msg_from_rgb(
            self,
            "stereo_left",
            camera_rgb("stereo_left"),
            camera_stamp("stereo_left"),
            frame_id=IMX219_CAMERA0_OPTICAL_FRAME,
            encoding="bgr8",
        )

    def build_imx219_camera1_raw() -> Any | None:
        return build_stereo_image_msg_from_rgb(
            self,
            "stereo_right",
            camera_rgb("stereo_right"),
            camera_stamp("stereo_right"),
            frame_id=IMX219_CAMERA1_OPTICAL_FRAME,
            encoding="bgr8",
        )

    def build_imx219_compressed(camera_name: str, frame_id: str) -> Any | None:
        rgb = camera_rgb(camera_name)
        header_stamp = camera_stamp(camera_name)
        if bool(getattr(self, "_camera_sensor_model_enabled", False)):
            return build_stereo_compressed_image_msg_from_rgb(
                self,
                camera_name,
                rgb,
                header_stamp,
                frame_id=frame_id,
                format_hint="bgr8; jpeg compressed bgr8",
            )
        if bool(getattr(self, "_stereo_image_async_enabled", False)):
            return build_stereo_compressed_image_msg_from_jpeg(
                self,
                camera_name,
                legacy_frame(camera_name)[2],
                header_stamp,
                frame_id=frame_id,
                format_hint="bgr8; jpeg compressed bgr8",
            )
        return build_stereo_compressed_image_msg_from_rgb(
            self,
            camera_name,
            rgb,
            header_stamp,
            frame_id=frame_id,
            format_hint="bgr8; jpeg compressed bgr8",
        )

    def build_imx219_camera_info(camera_name: str, frame_id: str) -> Any | None:
        if bool(getattr(self, "_camera_sensor_model_enabled", False)):
            if modeled_delivery(camera_name) is None:
                return None
        return build_camera_info_msg(
            self,
            camera_stamp(camera_name),
            camera_name=camera_name,
            frame_id=frame_id,
        )

    return {
        "stereo_left_image": build_left_image,
        "stereo_right_image": build_right_image,
        "real_camera_raw": build_real_camera_raw,
        "real_camera_compressed": build_real_camera_compressed,
        "real_camera_info": build_real_camera_info,
        "imx219_camera0_raw": build_imx219_camera0_raw,
        "imx219_camera0_compressed": lambda: build_imx219_compressed(
            "stereo_left", IMX219_CAMERA0_OPTICAL_FRAME
        ),
        "imx219_camera0_info": lambda: build_imx219_camera_info(
            "stereo_left", IMX219_CAMERA0_OPTICAL_FRAME
        ),
        "imx219_camera1_raw": build_imx219_camera1_raw,
        "imx219_camera1_compressed": lambda: build_imx219_compressed(
            "stereo_right", IMX219_CAMERA1_OPTICAL_FRAME
        ),
        "imx219_camera1_info": lambda: build_imx219_camera_info(
            "stereo_right", IMX219_CAMERA1_OPTICAL_FRAME
        ),
    }


def schedule_stereo_image_jobs(
    self: Any,
    jobs: Any,
    add_rate_limited: Any,
    *,
    builders: dict[str, object],
) -> None:
    if not bool(getattr(self, "_stereo_image_enabled", False)):
        return
    hz = float(getattr(self, "_stereo_image_hz", DEFAULT_IMAGE_HZ))
    add_rate_limited(
        self.pub_stereo_left_image,
        "/stereo/left/image_raw",
        builders["stereo_left_image"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_stereo_right_image,
        "/stereo/right/image_raw",
        builders["stereo_right_image"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_real_camera_raw,
        REAL_CAMERA_RAW_TOPIC,
        builders["real_camera_raw"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_real_camera_compressed,
        REAL_CAMERA_COMPRESSED_TOPIC,
        builders["real_camera_compressed"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_real_camera_info,
        REAL_CAMERA_INFO_TOPIC,
        builders["real_camera_info"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_imx219_camera0_raw,
        IMX219_CAMERA0_RAW_TOPIC,
        builders["imx219_camera0_raw"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_imx219_camera0_compressed,
        IMX219_CAMERA0_COMPRESSED_TOPIC,
        builders["imx219_camera0_compressed"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_imx219_camera0_info,
        IMX219_CAMERA0_INFO_TOPIC,
        builders["imx219_camera0_info"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_imx219_camera1_raw,
        IMX219_CAMERA1_RAW_TOPIC,
        builders["imx219_camera1_raw"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_imx219_camera1_compressed,
        IMX219_CAMERA1_COMPRESSED_TOPIC,
        builders["imx219_camera1_compressed"],
        hz,
        on_demand=True,
    )
    add_rate_limited(
        self.pub_imx219_camera1_info,
        IMX219_CAMERA1_INFO_TOPIC,
        builders["imx219_camera1_info"],
        hz,
        on_demand=True,
    )


def can_share_camera_renderer(self: Any, camera_name: str, width: int, height: int) -> bool:
    if not bool(getattr(self, "_stereo_image_enabled", False)):
        return False
    if str(camera_name) not in CAMERA_NAMES:
        return False
    return (
        int(width) == int(getattr(self, "_stereo_image_width", 0))
        and int(height) == int(getattr(self, "_stereo_image_height", 0))
    )


def render_camera_rgb(self: Any, camera_name: str, data: Any) -> np.ndarray:
    camera = str(camera_name)
    if camera not in CAMERA_NAMES:
        raise ValueError(f"unsupported stereo camera: {camera}")
    renderer = _renderer_for_camera(self, camera)
    renderer.update_scene(data, camera=camera)
    return np.ascontiguousarray(renderer.render(), dtype=np.uint8)


def build_stereo_image_msg(self: Any, camera_name: str, data: Any, stamp: Any) -> Any | None:
    if not bool(getattr(self, "_stereo_image_enabled", False)):
        return None
    return build_stereo_image_msg_from_rgb(
        self,
        camera_name,
        _render_camera_rgb_or_none(self, camera_name, data),
        stamp,
    )


def build_stereo_image_msg_from_rgb(
    self: Any,
    camera_name: str,
    rgb: np.ndarray | None,
    stamp: Any,
    *,
    frame_id: str | None = None,
    encoding: str = "rgb8",
) -> Any | None:
    if rgb is None:
        return None
    normalized_encoding = str(encoding).lower()
    if normalized_encoding == "rgb8":
        pixels = rgb
    elif normalized_encoding == "bgr8":
        pixels = np.ascontiguousarray(rgb[:, :, ::-1])
    else:
        raise ValueError(f"unsupported camera image encoding: {encoding}")
    msg = self.Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id or f"{camera_name}_optical"
    msg.height = int(rgb.shape[0])
    msg.width = int(rgb.shape[1])
    msg.encoding = normalized_encoding
    msg.is_bigendian = 0
    msg.step = int(rgb.shape[1] * 3)
    msg.data = array("B", pixels.tobytes())
    return msg


def build_stereo_compressed_image_msg(self: Any, camera_name: str, data: Any, stamp: Any) -> Any | None:
    if not bool(getattr(self, "_stereo_image_enabled", False)):
        return None
    return build_stereo_compressed_image_msg_from_rgb(
        self,
        camera_name,
        _render_camera_rgb_or_none(self, camera_name, data),
        stamp,
    )


def build_stereo_compressed_image_msg_from_rgb(
    self: Any,
    camera_name: str,
    rgb: np.ndarray | None,
    stamp: Any,
    *,
    frame_id: str | None = None,
    format_hint: str = "jpeg",
) -> Any | None:
    if rgb is None:
        return None
    cv2 = _load_cv2()
    if cv2 is None:
        _warn_once(self, "compressed_camera_cv2", "OpenCV is unavailable; compressed camera topic is disabled")
        return None
    try:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        quality = int(getattr(self, "_stereo_image_jpeg_quality", DEFAULT_COMPRESSED_JPEG_QUALITY))
        ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    except Exception as exc:
        _warn_once(self, f"{camera_name}_compressed", f"compressed camera render failed: {exc}")
        return None
    if not ok:
        _warn_once(self, f"{camera_name}_compressed_encode", "compressed camera JPEG encode failed")
        return None
    return build_stereo_compressed_image_msg_from_jpeg(
        self,
        camera_name,
        encoded.tobytes(),
        stamp,
        frame_id=frame_id,
        format_hint=format_hint,
    )


def build_stereo_compressed_image_msg_from_jpeg(
    self: Any,
    camera_name: str,
    encoded: bytes | None,
    stamp: Any,
    *,
    frame_id: str | None = None,
    format_hint: str = "jpeg",
) -> Any | None:
    if not encoded:
        return None
    msg = self.CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id or (
        REAL_CAMERA_OPTICAL_FRAME
        if camera_name == "stereo_left"
        else f"{camera_name}_optical"
    )
    msg.format = str(format_hint)
    msg.data = array("B", encoded)
    return msg


def build_camera_info_msg(
    self: Any,
    stamp: Any,
    *,
    camera_name: str = "stereo_left",
    frame_id: str | None = None,
) -> Any:
    width = int(self._stereo_image_width)
    height = int(self._stereo_image_height)
    calibration = getattr(self, "_camera_calibrations", {}).get(str(camera_name))
    if calibration is not None:
        calibration = calibration.scaled_to(width, height)
    else:
        fovy_deg = 90.0
        camera_id_attr = "_cam_left_id" if camera_name == "stereo_left" else "_cam_right_id"
        camera_id = int(getattr(self, camera_id_attr, -1))
        if camera_id >= 0:
            try:
                fovy_deg = float(self.model.cam_fovy[camera_id])
            except Exception:
                pass
        fy = 0.5 * height / np.tan(0.5 * np.deg2rad(fovy_deg))
        fx = fy
        cx = 0.5 * (width - 1)
        cy = 0.5 * (height - 1)
    msg = self.CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id or REAL_CAMERA_OPTICAL_FRAME
    msg.width = width
    msg.height = height
    if calibration is None:
        # Keep the original CameraInfo payload byte-for-byte when neither the
        # opt-in sensor model nor an explicit calibration file is active.
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    else:
        msg.distortion_model = calibration.distortion_model
        msg.d = list(calibration.d)
        msg.k = list(calibration.k)
        msg.r = list(calibration.r)
        msg.p = list(calibration.p)
    return msg


def _render_camera_rgb_or_none(self: Any, camera_name: str, data: Any) -> np.ndarray | None:
    try:
        return render_camera_rgb(self, camera_name, data)
    except Exception as exc:
        _warn_once(self, str(camera_name), f"stereo camera render failed: {exc}")
        return None


def _camera_rgb_for_publish(self: Any, camera_name: str, data: Any) -> np.ndarray | None:
    return _legacy_camera_frame_for_publish(self, camera_name, data)[0]


def _legacy_camera_frame_for_publish(
    self: Any,
    camera_name: str,
    data: Any,
) -> tuple[np.ndarray | None, float | None, bytes | None]:
    """Return one atomic legacy frame snapshot as RGB, capture time, and JPEG."""

    if not bool(getattr(self, "_stereo_image_async_enabled", False)):
        data_time = getattr(data, "time", None)
        capture_time_s = None if data_time is None else float(data_time)
        return _render_camera_rgb_or_none(self, camera_name, data), capture_time_s, None

    _submit_async_camera_render(self, camera_name, data)
    camera = str(camera_name)
    lock = self._stereo_image_async_lock
    with lock:
        return (
            self._stereo_image_async_latest.get(camera),
            self._stereo_image_async_latest_capture_time.get(camera),
            self._stereo_image_async_latest_jpeg.get(camera),
        )


def _modeled_camera_delivery_for_publish(
    self: Any,
    camera_name: str,
    data: Any,
) -> CameraFrameDelivery | None:
    runtime = getattr(self, "_camera_sensor_runtimes", {}).get(str(camera_name))
    if runtime is None:
        return None
    now_s = float(data.time)
    rendered = _rendered_camera_frame_for_publish(self, camera_name, data)
    try:
        deliveries = runtime.advance(now_s, rendered)
    except Exception as exc:
        _warn_once(
            self,
            f"{camera_name}_sensor_model",
            f"underwater camera sensor model failed: {exc}",
        )
        return None
    return deliveries[-1] if deliveries else None


def _rendered_camera_frame_for_publish(
    self: Any,
    camera_name: str,
    data: Any,
) -> RenderedCameraFrame | None:
    if not bool(getattr(self, "_stereo_image_async_enabled", False)):
        rgb = _render_camera_rgb_or_none(self, camera_name, data)
        if rgb is None:
            return None
        return RenderedCameraFrame(rgb=rgb, capture_time_s=float(data.time))

    _submit_async_camera_render(self, camera_name, data)
    lock = self._stereo_image_async_lock
    with lock:
        rgb = self._stereo_image_async_latest.get(str(camera_name))
        capture_time_s = self._stereo_image_async_latest_capture_time.get(str(camera_name))
    if rgb is None or capture_time_s is None:
        return None
    return RenderedCameraFrame(rgb=rgb, capture_time_s=float(capture_time_s))


def _submit_async_camera_render(self: Any, camera_name: str, data: Any) -> np.ndarray | None:
    _start_async_camera_worker(self)
    camera = str(camera_name)
    lock = self._stereo_image_async_lock
    with lock:
        latest = self._stereo_image_async_latest.get(camera)
        # One queued snapshot is enough.  Replacing it requires another full
        # MjData copy on the physics thread and cannot make the renderer catch
        # up any faster.
        if camera in self._stereo_image_async_pending:
            return latest
    snapshot = _acquire_async_camera_snapshot(self, camera)
    try:
        mujoco.mj_copyData(snapshot, self.model, data)
    except Exception as exc:
        _recycle_async_camera_snapshot(self, camera, snapshot)
        _warn_once(self, "async_camera_snapshot", f"async camera snapshot failed: {exc}")
        return None

    with lock:
        # Be defensive if another caller filled the single pending slot while
        # this snapshot was copied.  Do not replace a frame already queued for
        # the renderer; return this buffer to the bounded pool instead.
        if camera in self._stereo_image_async_pending:
            free = self._stereo_image_async_free.setdefault(camera, [])
            if len(free) < 2:
                free.append(snapshot)
            return self._stereo_image_async_latest.get(camera)
        self._stereo_image_async_pending[camera] = snapshot
        latest = self._stereo_image_async_latest.get(camera)
    self._stereo_image_async_event.set()
    return latest


def _start_async_camera_worker(self: Any) -> None:
    thread = getattr(self, "_stereo_image_async_thread", None)
    if thread is not None and thread.is_alive():
        return
    thread = threading.Thread(
        target=_async_camera_worker,
        args=(self,),
        name="uuv-camera-render",
        daemon=True,
    )
    self._stereo_image_async_thread = thread
    thread.start()


def _async_camera_worker(self: Any) -> None:
    event = self._stereo_image_async_event
    stop = self._stereo_image_async_stop
    lock = self._stereo_image_async_lock
    while not stop.is_set():
        event.wait(timeout=0.25)
        event.clear()
        with lock:
            pending = self._stereo_image_async_pending
            self._stereo_image_async_pending = {}
        for camera_name, snapshot in pending.items():
            try:
                if stop.is_set():
                    continue
                capture_time_s = float(getattr(snapshot, "time", 0.0))
                rgb = _render_camera_rgb_or_none(self, camera_name, snapshot)
                if rgb is None or stop.is_set():
                    continue
                encoded = (
                    None
                    if bool(getattr(self, "_camera_sensor_model_enabled", False))
                    else _encode_camera_jpeg(self, camera_name, rgb)
                )
                if stop.is_set():
                    continue
                with lock:
                    self._stereo_image_async_latest[camera_name] = rgb
                    self._stereo_image_async_latest_capture_time[camera_name] = capture_time_s
                    # Store encode failure as part of the same atomic frame
                    # tuple so a new RGB/stamp can never inherit an old JPEG.
                    self._stereo_image_async_latest_jpeg[camera_name] = encoded
            finally:
                if not stop.is_set():
                    _recycle_async_camera_snapshot(self, camera_name, snapshot)


def _acquire_async_camera_snapshot(self: Any, camera_name: str) -> Any:
    lock = self._stereo_image_async_lock
    with lock:
        free = self._stereo_image_async_free.setdefault(str(camera_name), [])
        if free:
            return free.pop()
    return mujoco.MjData(self.model)


def _recycle_async_camera_snapshot(self: Any, camera_name: str, snapshot: Any) -> None:
    lock = self._stereo_image_async_lock
    with lock:
        free = self._stereo_image_async_free.setdefault(str(camera_name), [])
        if len(free) < 2:
            free.append(snapshot)


def _latest_async_camera_jpeg(self: Any, camera_name: str) -> bytes | None:
    lock = self._stereo_image_async_lock
    with lock:
        return self._stereo_image_async_latest_jpeg.get(str(camera_name))


def _encode_camera_jpeg(
    self: Any, camera_name: str, rgb: np.ndarray
) -> bytes | None:
    cv2 = _load_cv2()
    if cv2 is None:
        _warn_once(
            self,
            "compressed_camera_cv2",
            "OpenCV is unavailable; compressed camera topic is disabled",
        )
        return None
    try:
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        quality = int(
            getattr(
                self,
                "_stereo_image_jpeg_quality",
                DEFAULT_COMPRESSED_JPEG_QUALITY,
            )
        )
        ok, encoded = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        )
    except Exception as exc:
        _warn_once(
            self,
            f"{camera_name}_compressed",
            f"compressed camera render failed: {exc}",
        )
        return None
    if not ok:
        _warn_once(
            self,
            f"{camera_name}_compressed_encode",
            "compressed camera JPEG encode failed",
        )
        return None
    return encoded.tobytes()


def _load_cv2() -> Any | None:
    global _CV2, _CV2_IMPORT_ATTEMPTED
    if _CV2_IMPORT_ATTEMPTED:
        return _CV2
    _CV2_IMPORT_ATTEMPTED = True
    try:
        import cv2  # type: ignore
    except Exception:
        _CV2 = None
    else:
        try:
            cv2.setNumThreads(1)
        except Exception:
            pass
        _CV2 = cv2
    return _CV2


def _env_int(name: str, default: int, *, minimum: int, maximum: int) -> int:
    try:
        value = int(float(str(os.environ.get(name, default)).strip()))
    except (TypeError, ValueError):
        value = int(default)
    return max(int(minimum), min(int(maximum), value))


def _env_bool(name: str, default: bool) -> bool:
    raw = str(os.environ.get(name, "1" if default else "0")).strip().lower()
    return raw in {"1", "true", "yes", "on"}


def _env_optional_float(name: str) -> float | None:
    raw = os.environ.get(name)
    if raw is None or not str(raw).strip():
        return None
    value = float(str(raw).strip())
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _env_optional_int(name: str) -> int | None:
    raw = os.environ.get(name)
    if raw is None or not str(raw).strip():
        return None
    return int(str(raw).strip())


def close_stereo_image_renderers(self: Any) -> None:
    stop = getattr(self, "_stereo_image_async_stop", None)
    event = getattr(self, "_stereo_image_async_event", None)
    thread = getattr(self, "_stereo_image_async_thread", None)
    if stop is not None:
        stop.set()
    if event is not None:
        event.set()
    if thread is not None and thread.is_alive() and thread is not threading.current_thread():
        thread.join(timeout=2.0)
    lock = getattr(self, "_stereo_image_async_lock", None)
    if lock is not None:
        with lock:
            getattr(self, "_stereo_image_async_pending", {}).clear()
            getattr(self, "_stereo_image_async_free", {}).clear()
            getattr(self, "_stereo_image_async_latest", {}).clear()
            getattr(self, "_stereo_image_async_latest_capture_time", {}).clear()
            getattr(self, "_stereo_image_async_latest_jpeg", {}).clear()
    for runtime in list(getattr(self, "_camera_sensor_runtimes", {}).values()):
        try:
            runtime.close()
        except Exception:
            pass
    getattr(self, "_camera_sensor_runtimes", {}).clear()
    renderers = getattr(self, "_stereo_image_renderers", {})
    for renderer in list(renderers.values()):
        try:
            renderer.close()
        except Exception:
            pass
    renderers.clear()


def _renderer_for_camera(self: Any, camera_name: str) -> Any:
    renderers = getattr(self, "_stereo_image_renderers", None)
    if renderers is None:
        renderers = {}
        self._stereo_image_renderers = renderers
    renderer = renderers.get(camera_name)
    if renderer is None:
        renderer = mujoco.Renderer(
            self.model,
            height=int(getattr(self, "_stereo_image_height", DEFAULT_IMAGE_HEIGHT)),
            width=int(getattr(self, "_stereo_image_width", DEFAULT_IMAGE_WIDTH)),
        )
        renderers[camera_name] = renderer
    return renderer


def _ensure_offscreen_buffer(bridge: Any, width: int, height: int) -> None:
    model = getattr(bridge, "model", None)
    if model is None:
        return
    visual_global = getattr(getattr(model, "vis", None), "global_", None)
    if visual_global is None:
        return
    visual_global.offwidth = max(int(getattr(visual_global, "offwidth", 0)), int(width))
    visual_global.offheight = max(int(getattr(visual_global, "offheight", 0)), int(height))


def _warn_once(self: Any, key: str, message: str) -> None:
    warned = getattr(self, "_stereo_image_warned", set())
    if key in warned:
        return
    warned.add(key)
    self._stereo_image_warned = warned
    node = getattr(self, "node", None)
    if node is not None:
        try:
            node.get_logger().warn(message)
            return
        except Exception:
            pass
    print(f"[stereo_image] {message}", flush=True)


__all__ = [
    "IMX219_CAMERA0_COMPRESSED_TOPIC",
    "IMX219_CAMERA0_INFO_TOPIC",
    "IMX219_CAMERA0_OPTICAL_FRAME",
    "IMX219_CAMERA0_RAW_TOPIC",
    "IMX219_CAMERA1_COMPRESSED_TOPIC",
    "IMX219_CAMERA1_INFO_TOPIC",
    "IMX219_CAMERA1_OPTICAL_FRAME",
    "IMX219_CAMERA1_RAW_TOPIC",
    "REAL_CAMERA_COMPRESSED_TOPIC",
    "REAL_CAMERA_INFO_TOPIC",
    "REAL_CAMERA_OPTICAL_FRAME",
    "REAL_CAMERA_RAW_TOPIC",
    "build_stereo_compressed_image_msg",
    "build_stereo_compressed_image_msg_from_jpeg",
    "build_stereo_compressed_image_msg_from_rgb",
    "build_stereo_image_msg",
    "build_stereo_image_msg_from_rgb",
    "build_stereo_publish_builders",
    "can_share_camera_renderer",
    "close_stereo_image_renderers",
    "configure_stereo_image_runtime",
    "create_stereo_image_publishers",
    "render_camera_rgb",
    "schedule_stereo_image_jobs",
]
