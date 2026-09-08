#!/usr/bin/env python3
"""Dependency-light contract checks for GUI research-pool launch presets."""

from __future__ import annotations

import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.sim_launch_preset import (  # noqa: E402
    COURSE_CURRENT_PRESET_ID,
    DEFAULT_SIM_LAUNCH_PRESET_ID,
    RESEARCH_POOL_DISTRIBUTED_PRESET_ID,
    RESEARCH_POOL_DISTRIBUTED_HYBRID_PRESET_ID,
    RESEARCH_POOL_DISTRIBUTED_WAVES_PRESET_ID,
    SIM_LAUNCH_PRESETS,
    build_sim_launch_preset_args,
    default_sim_launch_preset_id,
    merge_sim_launch_preset_args,
    resolve_sim_launch_preset,
    validate_sim_launch_preset,
)
from gui.sim_stack_env_contract import build_gui_sim_stack_env  # noqa: E402
from gui.sim_stack_launch_command import build_sim_stack_launch_command  # noqa: E402


PROFILE_PATH = ROOT / "config" / "sim_profiles.json"
RESEARCH_SCENE = ROOT / "scenes" / "research_pool_slam_scene.xml"


class _Node:
    def push_event(self, _event: str) -> None:
        return None


class _LaunchOwner:
    def __init__(self) -> None:
        self.env = {"UUV_GUI_STEREO_CAMERA_ENABLE": "0"}
        self.node = _Node()

    def _append_mavros_surface_args(self, _cmd: list[str]) -> None:
        return None

    def _env_flag(self, name: str, default: bool) -> bool:
        value = self.env.get(name)
        return default if value is None else value.strip().lower() in {"1", "true", "yes", "on"}

    @staticmethod
    def _arg_present(args: list[str], option: str) -> bool:
        return any(arg == option or arg.startswith(f"{option}=") for arg in args)

    @staticmethod
    def _normalized_sim_extra_args(extra_args: list[str] | None) -> list[str]:
        return list(extra_args or ())

    @staticmethod
    def _append_initial_depth_args(_cmd: list[str], _launch_args: list[str]) -> None:
        return None


def _assert_option(command: list[str], option: str, expected: str) -> None:
    if command.count(option) != 1:
        raise AssertionError(f"{option} must occur exactly once: {command}")
    actual = command[command.index(option) + 1]
    if actual != expected:
        raise AssertionError(f"{option}: expected {expected!r}, got {actual!r}")


def _resolved_profiles() -> dict[str, dict[str, object]]:
    payload = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    resolved: dict[str, dict[str, object]] = {}

    def resolve(name: str) -> dict[str, object]:
        if name in resolved:
            return dict(resolved[name])
        raw = dict(payload[name])
        parent = str(raw.pop("extends")) if "extends" in raw else ""
        merged = resolve(parent) if parent else {}
        merged.update(raw)
        resolved[name] = merged
        return dict(merged)

    for profile_name in payload:
        resolve(str(profile_name))
    return resolved


def check_preset_mapping_and_assets() -> None:
    if DEFAULT_SIM_LAUNCH_PRESET_ID != COURSE_CURRENT_PRESET_ID:
        raise AssertionError("GUI default must preserve the existing course/current contract")
    if default_sim_launch_preset_id({}) != COURSE_CURRENT_PRESET_ID:
        raise AssertionError("empty environment must select the compatibility preset")

    expected = {
        RESEARCH_POOL_DISTRIBUTED_HYBRID_PRESET_ID: ("research_pool_distributed_hybrid", "distributed"),
        RESEARCH_POOL_DISTRIBUTED_PRESET_ID: ("research_pool_distributed", "distributed"),
        RESEARCH_POOL_DISTRIBUTED_WAVES_PRESET_ID: (
            "research_pool_distributed_waves",
            "distributed",
        ),
    }
    for preset in SIM_LAUNCH_PRESETS:
        validate_sim_launch_preset(preset)
        if preset.preset_id not in expected:
            continue
        profile, fluid_model = expected[preset.preset_id]
        if preset.scene_path.resolve() != RESEARCH_SCENE.resolve():
            raise AssertionError(f"wrong research scene for {preset.preset_id}")
        if (preset.profile, preset.fluid_model) != (profile, fluid_model):
            raise AssertionError(f"wrong profile/fluid mapping for {preset.preset_id}")
        if preset.viewer_camera_mode != "follow":
            raise AssertionError("research-pool presets must start in robot-follow camera mode")

    for invalid in ("unknown", "../research_pool_slam_scene.xml", "current+distributed"):
        try:
            resolve_sim_launch_preset(invalid)
        except ValueError:
            continue
        raise AssertionError(f"unsafe/unknown preset was accepted: {invalid}")


def check_resolved_distributed_profiles() -> None:
    profiles = _resolved_profiles()
    expectations = {
        "research_pool_distributed": "flat",
        "research_pool_distributed_hybrid": "flat",
        "research_pool_distributed_waves": "harmonic",
    }
    for name, free_surface_mode in expectations.items():
        profile = profiles[name]
        for section in ("distributed_hydrodynamics", "hydrodynamic_matrices"):
            config = profile.get(section)
            if not isinstance(config, dict) or config.get("active") is not True:
                raise AssertionError(f"{name}.{section}.active must be true")
        if profile.get("thruster_inflow", {}).get("active") is not False:
            raise AssertionError(f"{name} must not enable uncalibrated thruster inflow by default")
        surface = profile.get("free_surface")
        if not isinstance(surface, dict) or surface.get("mode") != free_surface_mode:
            raise AssertionError(f"{name}.free_surface.mode must be {free_surface_mode}")


def check_final_launch_commands() -> None:
    for preset_id in (
        RESEARCH_POOL_DISTRIBUTED_PRESET_ID,
        RESEARCH_POOL_DISTRIBUTED_HYBRID_PRESET_ID,
        RESEARCH_POOL_DISTRIBUTED_WAVES_PRESET_ID,
    ):
        preset = resolve_sim_launch_preset(preset_id)
        preset_args = build_sim_launch_preset_args(preset)
        command = build_sim_stack_launch_command(
            _LaunchOwner(),
            start_script=ROOT / "start_sitl_mujoco_mj311.sh",
            backend="native",
            extra_args=preset_args,
        )
        _assert_option(command, "--scene", str(RESEARCH_SCENE))
        _assert_option(command, "--profile", preset.profile)
        _assert_option(command, "--fluid-model", "distributed")
        _assert_option(command, "--viewer-camera-mode", "follow")

    preset = resolve_sim_launch_preset(RESEARCH_POOL_DISTRIBUTED_PRESET_ID)
    for unsafe_extra in (
        ["--profile", "current"],
        ["--fluid-model=legacy"],
        ["--scene", "/tmp/injected.xml"],
    ):
        try:
            merge_sim_launch_preset_args(build_sim_launch_preset_args(preset), unsafe_extra)
        except ValueError:
            continue
        raise AssertionError(f"duplicate plant selection was accepted: {unsafe_extra}")


def check_control_contract_is_unchanged() -> None:
    env = build_gui_sim_stack_env({}, backend="native", sim_stack_dir=ROOT)
    expected = {
        "UUV_GUI_PILOT_CONTROL_MODE": "rc_override",
        "ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND": "rc_override",
        "ROS2_UUV_MAVROS_RC_PWM_SPAN": "400",
        "UUV_GUI_RC_PWM_SPAN": "400",
        "ROS2_UUV_SITL_CMD_VEL_SETPOINT_ENABLE": "0",
    }
    for key, value in expected.items():
        if env.get(key) != value:
            raise AssertionError(f"GUI control contract changed: {key}={env.get(key)!r}")


def check_frontend_and_backend_wiring() -> None:
    index = (ROOT / "gui" / "web_static" / "index.html").read_text(encoding="utf-8")
    app_js = (ROOT / "gui" / "web_static" / "app.js").read_text(encoding="utf-8")
    web_app = (ROOT / "gui" / "web_app.py").read_text(encoding="utf-8")
    web_manager = (ROOT / "gui" / "web_process_manager.py").read_text(encoding="utf-8")
    tk_layout = (ROOT / "gui" / "layout_control_stack.py").read_text(encoding="utf-8")
    tk_start = (ROOT / "gui" / "sim_stack_start_runtime.py").read_text(encoding="utf-8")

    required = {
        "web selector": (index, 'id="simLaunchPreset"'),
        "distributed option": (index, 'value="research_pool_distributed"'),
        "hybrid option": (index, 'value="research_pool_distributed_hybrid"'),
        "wave option": (index, 'value="research_pool_distributed_waves"'),
        "web preset request": (app_js, 'sim_preset: presetId'),
        "web handler validation boundary": (web_app, 'payload.get("sim_preset"'),
        "web child launch": (web_manager, "build_sim_launch_preset_args("),
        "Tk selector": (tk_layout, "sim_launch_preset_labels()"),
        "Tk validated launch": (tk_start, "validate_sim_launch_preset(preset)"),
    }
    for label, (text, needle) in required.items():
        if needle not in text:
            raise AssertionError(f"missing {label}: {needle}")

    forbidden_browser_fields = ('scene_path:', 'profile:', 'fluid_model:')
    start_call = 'postCommand({ command: "stack_start", sim_preset: presetId })'
    if start_call not in app_js:
        raise AssertionError("browser must submit only the stable preset identifier")
    for field in forbidden_browser_fields:
        if field in start_call:
            raise AssertionError(f"browser start request exposes raw launch field: {field}")


def main() -> int:
    check_preset_mapping_and_assets()
    check_resolved_distributed_profiles()
    check_final_launch_commands()
    check_control_contract_is_unchanged()
    check_frontend_and_backend_wiring()
    print("GUI distributed research-pool preset contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
