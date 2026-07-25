import os
import sys
from pathlib import Path

from kmu26_auv_web_gui.process_manager import ManagedProcess
from kmu26_auv_web_gui.process_manager import ProcessManager
from kmu26_auv_web_gui.process_manager import subprocess_env


def test_subprocess_environment_prefers_server_python() -> None:
    path_entries = subprocess_env()["PATH"].split(os.pathsep)

    assert Path(path_entries[0]).resolve() == Path(sys.executable).resolve().parent


def test_robot_stack_manages_localization_camera_and_audio(monkeypatch) -> None:
    started = []
    stopped = []

    monkeypatch.setattr(
        ProcessManager,
        "_stack_process_groups",
        lambda self: set(),
    )
    monkeypatch.setattr(
        ManagedProcess,
        "start",
        lambda self: started.append((self.name, self.cmd)),
    )
    monkeypatch.setattr(
        ManagedProcess,
        "stop",
        lambda self: stopped.append(self.name),
    )

    manager = ProcessManager(start_dronecan_allocator=False)
    manager.start_stack({"joy_release_when_idle": "true"})

    assert started == [
        (
            "localization_test",
            [
                "ros2",
                "launch",
                "hit25_auv_ros2",
                "localization_test.launch.py",
                "joy_release_when_idle:=true",
            ],
        ),
        (
            "realsense_camera",
            ["ros2", "launch", "realsense2_camera", "rs_launch.py"],
        ),
        (
            "audio_capture",
            ["ros2", "launch", "audio_capture", "capture.launch.py"],
        ),
    ]

    manager.stop_stack()
    assert stopped == [
        "audio_capture",
        "realsense_camera",
        "localization_test",
    ]
