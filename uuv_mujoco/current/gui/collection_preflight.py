"""Dependency-light admission and configuration evidence for VLA recording."""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil


def require_recordable_configuration(simulation: dict, pwm_span: float) -> None:
    """Reject known incompatible launch settings before creating a session."""
    if float(pwm_span) != 400.0:
        raise ValueError("VLA 시뮬 수집은 PWM span=400 계약을 사용합니다. GUI 설정을 확인하세요.")
    sensor = simulation.get("active_sensor_error") or {}
    overrides = sensor.get("overrides", {})
    for item in sensor.get("profiles", []):
        content = item.get("content", {})
        if "bar30" in content:
            data = content["bar30"]
            enabled = str(overrides.get("ROS2_UUV_BAR30_SENSOR_ENABLE", data["enabled"])).lower()
            rate = float(overrides.get("ROS2_UUV_BAR30_SENSOR_RATE_HZ", data["timing"]["capture"]["rate_hz"]))
            if enabled in {"true", "1", "yes", "on"} and rate < 10.0:
                raise ValueError("VLA 수집에는 수심 10 Hz 이상이 필요합니다. 2 Hz bag 모드 대신 수학적 오차 모드를 선택하세요.")
    local_ffmpeg = Path.home() / ".local/bin/ffmpeg"
    if not shutil.which("ffmpeg") and not (local_ffmpeg.is_file() and os.access(local_ffmpeg, os.X_OK)):
        raise ValueError("LeRobot 영상 내보내기에 FFmpeg가 필요합니다. 시스템 의존성을 설치하세요.")


def snapshot_files(paths: list[Path]) -> dict:
    """Capture required configuration contents and hashes before process spawn."""
    result = {}
    for path in paths:
        raw = path.read_bytes()
        result[str(path.resolve())] = {
            "sha256": hashlib.sha256(raw).hexdigest(),
            "content": raw.decode("utf-8"),
        }
    return result
