"""Independent, reproducible sensor-error selections for simulator launches."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "sensor_models"
PREFIXES = ("ROS2_UUV_IMU_SENSOR_", "ROS2_UUV_BAR30_SENSOR_", "ROS2_UUV_DVL_SENSOR_")
COMMON = "ROS2_UUV_IMU_BAR30_SENSOR_CONFIG_PATH"
MODES = (
    {"id": "existing", "label": "기존 센서 설정",
     "description": "기존 수학적 센서 모델과 명시적 환경 설정 유지. 무오차 모드가 아닙니다."},
    {"id": "mathematical", "label": "수학적 오차 · 가정값",
     "description": "고정 seed의 IMU·수심 bias/드리프트 및 DVL 측정·적분 모델. 실측 보정값 아님."},
    {"id": "bag0402", "label": "4월 bag · 압력 부분 보정 (2 Hz)",
     "description": "압력 변동과 당시 2 Hz 출력만 부분 보정. IMU·DVL은 가정값이며 10 Hz VLA 수집용이 아닙니다."},
)


def resolve_sensor_error_mode(mode: str | None) -> str:
    """Validate a stable mode identifier, defaulting to existing behavior."""
    selected = "existing" if mode is None else mode
    if not isinstance(selected, str) or selected not in {entry["id"] for entry in MODES}:
        raise ValueError(f"unknown sensor error mode: {selected}")
    return selected


def configure_sensor_error_mode(
    env: dict[str, str], mode: str | None
) -> tuple[dict[str, str], dict]:
    """Resolve an environment and immutable profile evidence without mutating input.

    Named modes own sensor overrides; physics, optics and controller settings
    are preserved. Existing mode preserves every explicit sensor override.
    """
    selected = resolve_sensor_error_mode(mode)
    result = dict(env)
    removed = {}
    if selected != "existing":
        for key in tuple(result):
            if key == COMMON or key.startswith(PREFIXES):
                removed[key] = result.pop(key)
        result[COMMON] = str(CONFIG / (
            "imu_bar30_bag_20260402.json" if selected == "bag0402"
            else "imu_bar30_uncalibrated_prior.json"
        ))
        result["ROS2_UUV_DVL_SENSOR_CONFIG_PATH"] = str(CONFIG / "a50_uncalibrated_prior.json")
        for prefix in PREFIXES:
            result[prefix + "ENABLE"] = "1"

    imu_path = Path(result.get(COMMON) or CONFIG / "imu_bar30_uncalibrated_prior.json").expanduser()
    dvl_path = Path(result.get("ROS2_UUV_DVL_SENSOR_CONFIG_PATH")
                    or result.get("ROS2_UUV_DVL_SENSOR_CONFIG")
                    or CONFIG / "a50_uncalibrated_prior.json").expanduser()
    profiles = []
    for path, schema in (
        (imu_path, "uuv_mujoco.sensor_model.imu_bar30.v1"),
        (dvl_path, "uuv_mujoco.sensor_model.a50.v1"),
    ):
        # The simulator starts with current/ as its working directory.
        path = path if path.is_absolute() else ROOT / path
        data = path.read_bytes()
        content = json.loads(data)
        if content.get("schema") != schema:
            raise ValueError(f"unexpected sensor schema: {path}")
        profiles.append({
            "path": str(path.resolve()), "sha256": hashlib.sha256(data).hexdigest(),
            "content": content,
        })
    snapshot = {
        "mode": selected,
        "description": next(entry["description"] for entry in MODES if entry["id"] == selected),
        "profiles": profiles,
        "overrides": {key: value for key, value in result.items()
                      if key == COMMON or key.startswith(PREFIXES)},
        "replaced_overrides": removed,
        "scope": "launch configuration; not runtime telemetry or independent hardware validation",
    }
    return result, snapshot
