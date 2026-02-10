#!/usr/bin/env python3
import rclpy
from typing import Dict, List, Any

try:
    from hit25_interpreter.msg import VisionData, DetectedObject
    _HAVE_VISION = True
except Exception:  # pragma: no cover - optional dependency
    VisionData = None
    DetectedObject = None
    _HAVE_VISION = False


class VisionConverter:
    """
    /hit25/vision 토픽을 구독하여 VisionData 메시지를 수신하고,
    미션 로직에서 사용하기 쉬운 파이썬 데이터 구조로 변환해 제공.
    """

    def __init__(self, node, topic_name: str = '/hit25/vision'):
        self._node = node
        self._latest_data: Dict[str, Any] = {}
        self._latest_objects: List[DetectedObject] = [] if _HAVE_VISION else []

        if not _HAVE_VISION:
            self._node.get_logger().error(
                "hit25_interpreter.msg not available. VisionConverter disabled."
            )
            self.sub = None
            return

        self._node.get_logger().info(f"[VisionConverter] Subscribing to {topic_name}")
        self.sub = self._node.create_subscription(
            VisionData, topic_name, self._vision_callback, 10
        )

    def _vision_callback(self, msg: 'VisionData'):
        self._latest_data['header'] = msg.header
        self._latest_data['screen_width'] = msg.screen_width
        self._latest_data['screen_height'] = msg.screen_height
        self._latest_objects = list(msg.detected_objects)

    def get_latest_vision_data(self) -> List['DetectedObject']:
        return self._latest_objects

    def find_objects_by_name(self, object_name: str) -> List['DetectedObject']:
        if not self._latest_objects:
            return []
        return [obj for obj in self._latest_objects if obj.object_name == object_name]

    def get_qr_data(self) -> str:
        qr_objects = self.find_objects_by_name('Qr')
        if qr_objects:
            return qr_objects[0].qr_data
        return ""
