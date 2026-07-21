#!/usr/bin/env python3
import importlib
import os
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray


BBOX_FORMAT = (
    "data = [stamp_sec, detected, class_id, confidence, center_x, center_y, "
    "width, height, image_width, image_height]"
)


class YoloBuoyDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_buoy_detector")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw/compressed")
        self.declare_parameter("bbox_topic", "/vision/buoy_bbox")
        self.declare_parameter(
            "annotated_image_topic", "/vision/yolo/annotated/compressed"
        )
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("annotated_jpeg_quality", 80)
        self.declare_parameter("model_path", "/home/auv/models/buoy.pt")
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("target_class_name", "")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("device", "auto")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("cpu_threads", 1)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("preview_window_name", "YOLO Buoy Detection")
        self.declare_parameter("publish_per_class", True)
        # 다중 부표 선택: 면적 큰 것 → 박스 확률(confidence) → 이미지 오른쪽
        self.declare_parameter("area_similar_ratio", 0.15)
        self.declare_parameter("confidence_similar_delta", 0.05)

        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.annotated_image_topic = str(
            self.get_parameter("annotated_image_topic").value
        )
        self.publish_annotated_image = bool(
            self.get_parameter("publish_annotated_image").value
        )
        self.annotated_jpeg_quality = int(
            self.get_parameter("annotated_jpeg_quality").value
        )
        if not 1 <= self.annotated_jpeg_quality <= 100:
            raise ValueError("annotated_jpeg_quality must be between 1 and 100")
        self.model_path = self.get_parameter("model_path").value
        if not self.model_path or not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"model_path not found: {self.model_path!r}. "
                "Pass model_path:=/absolute/path/to/model.pt"
            )
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value).strip().lower()
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.device = self._resolve_device(str(self.get_parameter("device").value))
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.cpu_threads = max(1, int(self.get_parameter("cpu_threads").value))
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.preview_window_name = str(self.get_parameter("preview_window_name").value)
        self.publish_per_class = bool(self.get_parameter("publish_per_class").value)
        self.area_similar_ratio = float(self.get_parameter("area_similar_ratio").value)
        self.confidence_similar_delta = float(
            self.get_parameter("confidence_similar_delta").value
        )
        self._preview_prev_time: Optional[float] = None
        self._preview_fps = 0.0

        self._configure_inference_threads()
        self.model = self._load_model(self.model_path)
        self.class_names = getattr(self.model, "names", {}) or {}
        self._warn_if_target_class_mismatch()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.on_image,
            image_qos,
        )
        self.bbox_pub = self.create_publisher(Float32MultiArray, self.bbox_topic, 10)
        self.annotated_image_pub = self.create_publisher(
            CompressedImage, self.annotated_image_topic, image_qos
        )

        self.get_logger().info(f"Subscribing: {self.image_topic}")
        self.get_logger().info(f"Publishing: {self.bbox_topic} ({BBOX_FORMAT})")
        if self.publish_annotated_image:
            self.get_logger().info(
                f"Publishing annotated JPEG: {self.annotated_image_topic} "
                f"(quality={self.annotated_jpeg_quality})"
            )
        self.get_logger().info(f"Model classes: {self._format_class_names()}")
        self.get_logger().info(
            f"YOLO PT model={self.model_path}, device={self.device}, imgsz={self.imgsz}, "
            f"cpu_threads={self.cpu_threads}, "
            f"target_class_id={self.target_class_id}, target_class_name='{self.target_class_name}', "
            f"show_preview={self.show_preview}, publish_per_class={self.publish_per_class}"
        )
        if self.show_preview:
            self.get_logger().info(
                f"Preview window '{self.preview_window_name}' enabled (press q in window to quit)"
            )

    def _load_model(self, model_path: str):
        ultralytics = importlib.import_module("ultralytics")
        return ultralytics.YOLO(model_path)

    def _configure_inference_threads(self) -> None:
        """Bound CPU inference so it cannot starve the 100 Hz SITL path."""
        if self.device != "cpu":
            return
        try:
            torch = importlib.import_module("torch")
            torch.set_num_threads(self.cpu_threads)
            # This call can only be made once in a Python process.
            try:
                torch.set_num_interop_threads(1)
            except RuntimeError:
                pass
        except Exception as exc:
            self.get_logger().warning(f"Could not set PyTorch CPU thread limit: {exc}")
        try:
            cv2.setNumThreads(self.cpu_threads)
        except Exception:
            pass

    def _resolve_device(self, device: str) -> str:
        requested = device.strip()
        if requested and requested.lower() != "auto":
            return requested

        try:
            torch = importlib.import_module("torch")
            if torch.cuda.is_available():
                return "cuda:0"
        except ImportError:
            pass
        return "cpu"

    def _format_class_names(self) -> str:
        if not self.class_names:
            return "[]"
        return str({int(key): str(value) for key, value in self.class_names.items()})

    def _warn_if_target_class_mismatch(self) -> None:
        if self.target_class_id >= 0:
            if self.target_class_id not in self.class_names:
                self.get_logger().warning(
                    f"target_class_id={self.target_class_id} is not in model classes: "
                    f"{self._format_class_names()}"
                )
            return

        if not self.target_class_name:
            return

        model_class_names = {str(value).strip().lower() for value in self.class_names.values()}
        if self.target_class_name not in model_class_names:
            self.get_logger().warning(
                f"target_class_name='{self.target_class_name}' does not match model classes "
                f"{sorted(model_class_names)}. All detections will be filtered out unless you change "
                f"target_class_name or set target_class_name:="
            )

    def on_image(self, msg: CompressedImage) -> None:
        image = self._decode_compressed_image(msg)
        if image is None:
            self.get_logger().warning("Failed to decode compressed image", throttle_duration_sec=2.0)
            return

        image_height, image_width = image.shape[:2]
        detection, all_detections = self._detect_targets(image)
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        published_detections = [detection] if detection is not None else []
        if self.publish_per_class:
            published_detections = self._best_detection_per_class(all_detections)

        if published_detections:
            for published_detection in published_detections:
                self._publish_detection(
                    stamp_sec, published_detection, image_width, image_height
                )
        else:
            self._publish_detection(stamp_sec, None, image_width, image_height)

        if self.publish_annotated_image or self.show_preview:
            self._update_preview_fps()
            annotated_image = self._render_annotated_image(
                image, detection, all_detections
            )
            if self.publish_annotated_image:
                self._publish_annotated_image(msg, annotated_image)
            if self.show_preview:
                self._show_preview(annotated_image)

    def _decode_compressed_image(self, msg: CompressedImage) -> Optional[np.ndarray]:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def _is_better_detection(
        self,
        candidate: Tuple[int, float, float, float, float, float],
        current: Tuple[int, float, float, float, float, float],
    ) -> bool:
        """면적 큰 것 → 박스 확률(confidence) → 이미지 오른쪽(center_x) 순으로 비교."""
        _, cand_conf, cand_cx, _, cand_w, cand_h = candidate
        _, cur_conf, cur_cx, _, cur_w, cur_h = current
        cand_area = max(0.0, cand_w * cand_h)
        cur_area = max(0.0, cur_w * cur_h)
        larger = max(cand_area, cur_area, 1.0)
        if abs(cand_area - cur_area) > self.area_similar_ratio * larger:
            return cand_area > cur_area
        if abs(cand_conf - cur_conf) > self.confidence_similar_delta:
            return cand_conf > cur_conf
        return cand_cx > cur_cx

    def _best_detection_per_class(
        self,
        all_detections: list[Tuple[int, float, float, float, float, float, int, int, int, int]],
    ) -> list[Tuple[int, float, float, float, float, float]]:
        best_by_class: dict[int, Tuple[int, float, float, float, float, float]] = {}
        for class_id, confidence, center_x, center_y, width, height, *_ in all_detections:
            if not self._class_matches(class_id):
                continue
            candidate = (
                class_id,
                confidence,
                center_x,
                center_y,
                width,
                height,
            )
            previous = best_by_class.get(class_id)
            if previous is None or self._is_better_detection(candidate, previous):
                best_by_class[class_id] = candidate
        return [best_by_class[class_id] for class_id in sorted(best_by_class)]

    def _publish_detection(
        self,
        stamp_sec: float,
        detection: Optional[Tuple[int, float, float, float, float, float]],
        image_width: int,
        image_height: int,
    ) -> None:
        out = Float32MultiArray()
        if detection is None:
            out.data = [
                stamp_sec, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                float(image_width), float(image_height),
            ]
        else:
            class_id, confidence, center_x, center_y, width, height = detection
            out.data = [
                stamp_sec, 1.0, float(class_id), float(confidence),
                float(center_x), float(center_y), float(width), float(height),
                float(image_width), float(image_height),
            ]
        self.bbox_pub.publish(out)

    def _detect_targets(
        self, image: np.ndarray
    ) -> Tuple[
        Optional[Tuple[int, float, float, float, float, float]],
        list[Tuple[int, float, float, float, float, float, int, int, int, int]],
    ]:
        results = self.model.predict(
            source=image,
            conf=self.confidence_threshold,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )
        if not results:
            return None, []

        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return None, []

        xyxy = boxes.xyxy.detach().cpu().numpy()
        confidences = boxes.conf.detach().cpu().numpy()
        classes = boxes.cls.detach().cpu().numpy().astype(int)

        all_detections: list[Tuple[int, float, float, float, float, float, int, int, int, int]] = []
        best = None
        filtered_count = 0
        for rect, confidence, class_id in zip(xyxy, confidences, classes):
            x1, y1, x2, y2 = rect
            width = max(0.0, float(x2 - x1))
            height = max(0.0, float(y2 - y1))
            center_x = float(x1 + width / 2.0)
            center_y = float(y1 + height / 2.0)
            ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)
            det = (
                int(class_id),
                float(confidence),
                center_x,
                center_y,
                width,
                height,
                ix1,
                iy1,
                ix2,
                iy2,
            )
            all_detections.append(det)

            if not self._class_matches(int(class_id)):
                filtered_count += 1
                continue
            candidate = (
                int(class_id),
                float(confidence),
                center_x,
                center_y,
                width,
                height,
            )
            if best is None or self._is_better_detection(candidate, best):
                best = candidate

        if all_detections and best is None:
            self.get_logger().warning(
                f"YOLO found {len(all_detections)} object(s) but none matched "
                f"target_class_id={self.target_class_id}, target_class_name='{self.target_class_name}'. "
                f"Filtered {filtered_count} detection(s).",
                throttle_duration_sec=3.0,
            )

        return best, all_detections

    def _class_matches(self, class_id: int) -> bool:
        if self.target_class_id >= 0:
            return class_id == self.target_class_id
        if not self.target_class_name:
            return True
        class_name = str(self.class_names.get(class_id, "")).strip().lower()
        return class_name == self.target_class_name

    def _update_preview_fps(self) -> None:
        now = time.monotonic()
        if self._preview_prev_time is not None:
            dt = now - self._preview_prev_time
            if dt > 0.0:
                self._preview_fps = 1.0 / dt
        self._preview_prev_time = now

    def _render_annotated_image(
        self,
        image: np.ndarray,
        detection: Optional[Tuple[int, float, float, float, float, float]],
        all_detections: list[Tuple[int, float, float, float, float, float, int, int, int, int]],
    ) -> np.ndarray:
        display = image.copy()
        height, width = display.shape[:2]
        center_x = width // 2
        center_y = height // 2

        selected_key = None
        if detection is not None:
            selected_key = (
                detection[0],
                round(detection[1], 4),
                round(detection[2], 1),
                round(detection[3], 1),
            )

        for class_id, confidence, det_cx, det_cy, _, _, x1, y1, x2, y2 in all_detections:
            det_key = (class_id, round(confidence, 4), round(det_cx, 1), round(det_cy, 1))
            is_selected = selected_key == det_key
            color = (0, 255, 0) if is_selected else (0, 165, 255)
            thickness = 2 if is_selected else 1
            cv2.rectangle(display, (x1, y1), (x2, y2), color, thickness)
            class_name = str(self.class_names.get(class_id, class_id))
            label = f"{class_name} {confidence:.2f}"
            cv2.putText(
                display,
                label,
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )

        cv2.drawMarker(
            display,
            (center_x, center_y),
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=1,
        )
        cv2.line(display, (center_x, 0), (center_x, height), (0, 255, 255), 1)
        cv2.line(display, (0, center_y), (width, center_y), (0, 255, 255), 1)

        status = "NO DETECTION"
        status_color = (0, 0, 255)
        if detection is not None:
            class_id, confidence, det_cx, det_cy, _, _ = detection
            cv2.drawMarker(
                display,
                (int(det_cx), int(det_cy)),
                (0, 255, 0),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=16,
                thickness=2,
            )
            class_name = str(self.class_names.get(class_id, class_id))
            status = f"TRACKING {class_name} {confidence:.2f}"
            status_color = (0, 255, 0)
        elif all_detections:
            status = f"FILTERED {len(all_detections)} detection(s)"
            status_color = (0, 165, 255)

        cv2.putText(
            display,
            status,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            f"FPS {self._preview_fps:.1f}  device={self.device}",
            (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            f"{width}x{height}  topic={self.image_topic}",
            (12, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            display,
            f"raw={len(all_detections)}  filter='{self.target_class_name or 'ALL'}'",
            (12, 108),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

        return display

    def _publish_annotated_image(
        self, source_msg: CompressedImage, image: np.ndarray
    ) -> None:
        success, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, self.annotated_jpeg_quality],
        )
        if not success:
            self.get_logger().warning(
                "Failed to encode annotated image", throttle_duration_sec=2.0
            )
            return

        out = CompressedImage()
        out.header = source_msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes()
        self.annotated_image_pub.publish(out)

    def _show_preview(self, display: np.ndarray) -> None:
        cv2.imshow(self.preview_window_name, display)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            raise KeyboardInterrupt("Preview window closed by user")

    def close_preview(self) -> None:
        if self.show_preview:
            cv2.destroyWindow(self.preview_window_name)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloBuoyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_preview()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
