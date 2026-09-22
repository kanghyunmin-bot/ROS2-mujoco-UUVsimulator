"""Explicit GUI RC ownership transfer; never arms or enables a policy."""


def prepare_vla_control(controller) -> dict:
    """Release the GUI publisher so a separately started policy can own RC."""
    with controller._pinger_rc_handoff_lock:
        controller.recorder.require_configuration_unlocked()
        if getattr(controller, "_vla_control_prepared", False):
            return {"status": "VLA 제어권 준비 완료 · 정책 활성화는 별도", "vla_prepared": True}
        if not controller.processes.simulation_runtime_available():
            raise ValueError("시뮬레이터를 먼저 시작하세요.")
        if controller.processes.pinger_homing_running() or controller.processes.mission_running():
            raise ValueError("다른 자율제어를 종료한 뒤 VLA로 전환하세요.")
        if controller.replay.running():
            raise ValueError("RC 재생을 먼저 종료하세요.")
        controller._vla_control_prepared = True
        try:
            controller.release_rc()
            if not controller.node.suspend_rc_override_publisher():
                raise ValueError("GUI RC 발행자 중단 실패")
        except Exception:
            controller._vla_control_prepared = False
            raise
        controller.node.push_event("GUI RC publisher released for VLA; policy remains separately controlled")
        return {"status": "VLA 제어권 준비 완료 · 조이스틱 차단 · 정책 활성화는 별도", "vla_prepared": True}


def restore_manual_control(controller) -> dict:
    """Restore GUI RC only after every external RC publisher has exited."""
    with controller._pinger_rc_handoff_lock:
        if not getattr(controller, "_vla_control_prepared", False):
            return {"status": "수동 제어 모드", "vla_prepared": False}
        if controller.node.count_publishers(controller.node._rc_override_topic) != 0:
            raise ValueError("VLA 정책 노드를 종료하고 발행자 연결 해제를 기다린 뒤 복귀하세요.")
        controller.release_rc()
        if not controller.node.restore_rc_override_publisher():
            raise ValueError("GUI RC 발행자 복구 실패")
        controller._vla_control_prepared = False
        controller.node.push_event("GUI manual RC restored after VLA publisher exit")
        return {"status": "수동 제어 복귀 완료 · 새 조이스틱 입력 대기", "vla_prepared": False}
