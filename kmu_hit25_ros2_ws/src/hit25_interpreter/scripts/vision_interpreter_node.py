#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import traceback
import zipfile
from pathlib import Path

import cv2
import numpy as np
try:
    import openvino as ov
except Exception:  # pragma: no cover
    import openvino.runtime as ov
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from hit25_interpreter.msg import VisionData, DetectedObject


class VisionInterpreter(Node):
    """
    ROS 2 노드: OpenVINO 객체 탐지 결과를 발행하며,
    탐지된 객체의 위치에 로우패스 필터를 적용하여 안정화합니다.
    """

    def __init__(self):
        super().__init__('vision_interpreter_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('lpf_alpha', 0.5)
        self.declare_parameter('use_compressed', False)

        requested_model_path = str(self.get_parameter('model_path').value).strip()
        model_xml_path = self._resolve_model_path(requested_model_path)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.lpf_alpha = float(self.get_parameter('lpf_alpha').value)
        self.use_compressed = self.get_parameter('use_compressed').value

        self.class_names = {0: 'Buoy', 1: 'Line', 2: 'Obstacle', 3: 'Qr'}
        self.smooth_objects = {}

        self.vision_pub = self.create_publisher(VisionData, '/hit25/vision', 10)
        self.bridge = CvBridge()

        self.get_logger().info(f"OpenVINO 모델 로딩 중: {model_xml_path}")
        try:
            core = ov.Core()
            model = core.read_model(model=model_xml_path)
            self.compiled_model = core.compile_model(model=model, device_name="CPU")
            self.input_layer = self.compiled_model.input(0)
            self.output_layer = self.compiled_model.output(0)
            _, _, self.input_height, self.input_width = self.input_layer.shape
            self.get_logger().info(
                f"모델 로딩 완료. 입력 크기: ({self.input_height}, {self.input_width})"
            )
        except Exception as exc:
            self.get_logger().error(f"OpenVINO 모델 로딩 실패: {exc}")
            raise

        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage,
                '/camera/color/image_raw/compressed',
                self.image_callback_compressed,
                qos_profile_sensor_data,
            )
            self.get_logger().info("압축 이미지 토픽 구독 (/camera/color/image_raw/compressed)")
        else:
            self.image_sub = self.create_subscription(
                Image,
                '/stereo/left/image_raw',
                self.image_callback_raw,
                qos_profile_sensor_data,
            )
            self.get_logger().info("원본 이미지 토픽 구독 (/stereo/left/image_raw)")
        
        self.get_logger().info(f"로우패스 필터 Alpha: {self.lpf_alpha}")

    def _candidate_model_dirs(self) -> list[Path]:
        dirs = [Path(__file__).resolve().parents[1] / 'models']
        try:
            from ament_index_python.packages import get_package_share_directory
            dirs.append(Path(get_package_share_directory('hit25_interpreter')) / 'models')
        except Exception:
            pass
        unique_dirs = []
        for d in dirs:
            if d not in unique_dirs:
                unique_dirs.append(d)
        return unique_dirs

    def _auto_extract_model_zip(self, model_dir: Path) -> None:
        for zip_name in ('1122best_openvino_model.zip', '0906best_openvino_model.zip'):
            zip_path = model_dir / zip_name
            if not zip_path.exists():
                continue
            try:
                with zipfile.ZipFile(zip_path, 'r') as zf:
                    zf.extractall(model_dir)
            except Exception as exc:
                self.get_logger().warn(f"모델 zip 자동 해제 실패 ({zip_path}): {exc}")

    def _resolve_model_path(self, requested_path: str) -> str:
        if requested_path:
            p = Path(requested_path).expanduser()
            if p.exists():
                return str(p)
            self.get_logger().warn(f"요청된 model_path가 존재하지 않습니다: {p}")

        searched = []
        for model_dir in self._candidate_model_dirs():
            searched.append(str(model_dir))
            if not model_dir.exists():
                continue

            self._auto_extract_model_zip(model_dir)

            preferred = [
                model_dir / '1122best_openvino_model' / '1122best.xml',
                model_dir / '0906best_openvino_model' / '0906best.xml',
            ]
            for cand in preferred:
                if cand.exists():
                    return str(cand)

            for cand in sorted(model_dir.glob('*_openvino_model/*.xml')):
                if cand.exists():
                    return str(cand)

        raise FileNotFoundError(
            "OpenVINO 모델(.xml)을 찾지 못했습니다. "
            f"검색 경로: {searched}. "
            "launch 인자 model_path를 직접 지정하거나 models/*.zip을 확인하세요."
        )

    def process_with_openvino(self, image):
        """OpenVINO 추론 및 NMS를 수행하여 탐지된 객체 리스트를 반환합니다."""
        img_height, img_width = image.shape[:2]
        ratio = min(self.input_width / img_width, self.input_height / img_height)
        new_unpad_w, new_unpad_h = int(img_width * ratio), int(img_height * ratio)
        resized_image = cv2.resize(image, (new_unpad_w, new_unpad_h), interpolation=cv2.INTER_AREA)

        padded_image = np.full((self.input_height, self.input_width, 3), 114, dtype=np.uint8)
        padded_image[
            (self.input_height - new_unpad_h) // 2 : (self.input_height - new_unpad_h) // 2 + new_unpad_h,
            (self.input_width - new_unpad_w) // 2 : (self.input_width - new_unpad_w) // 2 + new_unpad_w,
            :
        ] = resized_image

        input_tensor = np.expand_dims(padded_image.transpose(2, 0, 1), 0).astype(np.float32) / 255.0
        result = self.compiled_model([input_tensor])[self.output_layer]
        outputs = np.transpose(np.squeeze(result))

        boxes, scores, class_ids = [], [], []
        gain = min(self.input_width / img_width, self.input_height / img_height)
        pad_w = (self.input_width - img_width * gain) / 2
        pad_h = (self.input_height - img_height * gain) / 2

        for row in outputs:
            class_probs = row[4:]
            class_id = int(np.argmax(class_probs))
            confidence = float(class_probs[class_id])

            if confidence > self.conf_threshold:
                cx, cy, w, h = row[:4]
                x1 = int((cx - w / 2 - pad_w) / gain)
                y1 = int((cy - h / 2 - pad_h) / gain)
                width = int(w / gain)
                height = int(h / gain)

                boxes.append([x1, y1, width, height])
                scores.append(confidence)
                class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_threshold, self.iou_threshold)
        detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                x1, y1, w, h = boxes[i]
                detections.append({
                    'class_id': class_ids[i],
                    'name': self.class_names.get(class_ids[i], 'Unknown'),
                    'box': [x1, y1, x1 + w, y1 + h],
                    'conf': scores[i],
                })

        return detections

    def apply_low_pass_filter(self, key, current_x, current_y):
        """1차 로우패스 필터를 적용하여 위치를 부드럽게 만듭니다."""
        if key not in self.smooth_objects:
            self.smooth_objects[key] = {'x': current_x, 'y': current_y, 'last_seen': time.time()}
            return current_x, current_y

        smoothed_x = self.lpf_alpha * current_x + (1 - self.lpf_alpha) * self.smooth_objects[key]['x']
        smoothed_y = self.lpf_alpha * current_y + (1 - self.lpf_alpha) * self.smooth_objects[key]['y']

        self.smooth_objects[key]['x'] = smoothed_x
        self.smooth_objects[key]['y'] = smoothed_y
        self.smooth_objects[key]['last_seen'] = time.time()

        return smoothed_x, smoothed_y

    def image_callback_compressed(self, msg):
        """압축 이미지 콜백"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_image(image, msg.header)
        except Exception as e:
            self.get_logger().error(f"이미지 디코딩 실패: {e}")

    def image_callback_raw(self, msg):
        """원본 이미지 콜백"""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(image, msg.header)
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")

    def process_image(self, cv_image, header):
        """이미지 처리 공통 로직"""
        try:
            detections = self.process_with_openvino(cv_image)

            vision_msg = VisionData()
            vision_msg.header = header
            h, w, _ = cv_image.shape
            vision_msg.screen_width = int(w)
            vision_msg.screen_height = int(h)

            detected_objects_list = []
            object_counts = {name: 0 for name in self.class_names.values()}

            for det in detections:
                obj = DetectedObject()
                x1, y1, x2, y2 = det['box']
                class_name = det['name']

                current_count = object_counts.get(class_name, 0)
                object_counts[class_name] = current_count + 1
                lpf_key = f"{class_name}_{current_count}"

                current_center_x = (x1 + x2) / 2.0
                current_center_y = (y1 + y2) / 2.0

                smoothed_x, smoothed_y = self.apply_low_pass_filter(
                    lpf_key, current_center_x, current_center_y
                )

                obj.object_name = class_name
                obj.confidence = float(det['conf'])
                obj.center_x = float(smoothed_x)
                obj.center_y = float(smoothed_y)
                obj.width = float(x2 - x1)
                obj.height = float(y2 - y1)

                obj.slope = float('nan')
                obj.qr_data = ""

                if obj.object_name == 'Line':
                    dx = float(x2 - x1)
                    dy = float(y1 - y2)
                    if abs(dx) < 1e-6:
                        obj.slope = float('inf') if dy >= 0 else float('-inf')
                    else:
                        obj.slope = dy / dx
                elif obj.object_name == 'Qr':
                    try:
                        decoded = decode(cv_image[y1:y2, x1:x2])
                        if decoded:
                            obj.qr_data = decoded[0].data.decode('utf-8')
                    except Exception as exc:
                        self.get_logger().warn(f"QR 디코딩 중 에러 발생: {exc}")

                detected_objects_list.append(obj)

            vision_msg.detected_objects = detected_objects_list
            self.vision_pub.publish(vision_msg)

            current_time = time.time()
            keys_to_delete = [
                key for key, data in self.smooth_objects.items()
                if current_time - data['last_seen'] > 1.0
            ]
            for key in keys_to_delete:
                del self.smooth_objects[key]

        except Exception as exc:
            self.get_logger().error(
                f"이미지 처리 중 에러 발생: {exc}\n{traceback.format_exc()}"
            )


def main():
    rclpy.init()
    node = None
    try:
        node = VisionInterpreter()
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
