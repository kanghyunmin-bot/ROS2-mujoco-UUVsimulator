#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import traceback

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
from sensor_msgs.msg import CompressedImage

from hit25_interpreter.msg import VisionData, DetectedObject


class VisionInterpreter(Node):
    def __init__(self):
        super().__init__('vision_interpreter_node')

        self.declare_parameter('model_path', 'default.xml')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)

        model_xml_path = self.get_parameter('model_path').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)

        self.class_names = {0: 'Buoy', 1: 'Line', 2: 'Obstacle', 3: 'Qr'}

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

        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info("토픽 구독 시작 및 이미지 대기 중...")

    def process_with_openvino(self, image):
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
                class_name = self.class_names.get(class_ids[i], 'Unknown')
                detections.append({'name': class_name, 'box': [x1, y1, x1 + w, y1 + h], 'conf': scores[i]})

        return detections

    def image_callback(self, ros_data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(ros_data, "bgr8")
            detections = self.process_with_openvino(cv_image)

            vision_msg = VisionData()
            vision_msg.header.stamp = self.get_clock().now().to_msg()
            vision_msg.header.frame_id = "camera_color_optical_frame"
            h, w, _ = cv_image.shape
            vision_msg.screen_width = int(w)
            vision_msg.screen_height = int(h)

            detected_objects_list = []
            for det in detections:
                obj = DetectedObject()
                obj.object_name = det['name']
                obj.confidence = float(det['conf'])

                x1, y1, x2, y2 = det['box']
                obj.center_x = float((x1 + x2) / 2.0)
                obj.center_y = float((y1 + y2) / 2.0)
                obj.width = float(x2 - x1)
                obj.height = float(y2 - y1)

                obj.slope = float('nan')
                obj.qr_data = ""

                if obj.object_name == 'Line':
                    if (x2 - x1) > 0:
                        obj.slope = float(y2 - y1) / float(x2 - x1)
                    else:
                        obj.slope = float('inf')
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

        except Exception as exc:
            self.get_logger().error(
                f"image_callback에서 에러 발생: {exc}\n{traceback.format_exc()}"
            )


def main():
    rclpy.init()
    node = None
    try:
        node = VisionInterpreter()
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
