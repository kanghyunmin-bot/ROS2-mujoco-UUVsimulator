"""Execute launch backend selection without starting a simulator or ROS."""

import os
from pathlib import Path
import subprocess

SOURCE = Path(__file__).resolve().parents[1] / "launch_uuv_sim.sh"


def select_backend(headless, host_os, display, backend):
    source = SOURCE.read_text()
    start = source.index("# Set display for headless mode")
    end = source.index("\nconfigure_mujoco_viewer_window_backend\n", start)
    script = (
        source[start:end]
        + '\nprintf "RESULT:%s|%s|%s" "${MUJOCO_GL:-}" "$DISPLAY" "${PYOPENGL_PLATFORM:-}"'
    )
    result = subprocess.run(
        ["bash", "-eu", "-c", script],
        check=True,
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "HEADLESS": str(headless).lower(),
            "HOST_OS": host_os,
            "DISPLAY": display,
            "MUJOCO_GL": backend,
            "PYOPENGL_PLATFORM": "egl",
        },
    )
    return result.stdout.split("RESULT:")[-1]


def test_native_viewer_does_not_inherit_headless_egl():
    assert select_backend(False, "Linux", ":1", "egl") == "glfw|:1|"


def test_native_viewer_uses_one_backend_on_remote_display_too():
    assert select_backend(False, "Linux", ":20", "egl") == "glfw|:20|"


def test_headless_linux_still_uses_egl():
    assert select_backend(True, "Linux", ":1", "glfw") == "egl||egl"


def test_headless_macos_keeps_cgl():
    assert select_backend(True, "Darwin", ":1", "glfw").startswith("cgl||")
