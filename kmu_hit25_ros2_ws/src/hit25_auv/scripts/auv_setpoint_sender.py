from typing import Optional

try:
    from hit25_auv.msg import AuvSetpoint
    _HAVE_MSG = True
except Exception:  # pragma: no cover
    AuvSetpoint = None
    _HAVE_MSG = False


class AUVSetpointSender:
    """
    미션 로직에서 생성한 목표값을 /auv_setpoint로 발행하는 헬퍼 클래스.
    """

    def __init__(self, node, setpoint_topic: str = '/auv_setpoint'):
        self._node = node
        if not _HAVE_MSG:
            self._node.get_logger().error("AuvSetpoint.msg not available. Setpoint sender disabled.")
            self._pub = None
            self._msg = None
            return

        self._node.get_logger().info(
            f"[AUVSetpointSender] Publishing to {setpoint_topic} (AuvSetpoint)"
        )
        self._pub = self._node.create_publisher(AuvSetpoint, setpoint_topic, 10)
        self._msg = AuvSetpoint()

    def send_setpoint(self, x: float, y: float, z: float, yaw: float, is_relative: bool = True):
        if self._pub is None:
            return
        mode = 1 if is_relative else 0
        self._msg.x = float(x)
        self._msg.y = float(y)
        self._msg.z = float(z)
        self._msg.yaw = float(yaw)
        self._msg.mode = int(mode)
        self._pub.publish(self._msg)
