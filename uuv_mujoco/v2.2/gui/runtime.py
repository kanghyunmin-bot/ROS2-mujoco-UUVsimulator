#!/usr/bin/env python3
"""ROS and Python runtime imports for the MuJoCo UUV GUI."""


from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import re
import shlex
import shutil
import site
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from tkinter import filedialog, ttk
from typing import Any, Deque, Iterable, Optional


def _sanitize_python_import_path() -> None:
    """Drop site-packages entries from other Python envs before importing ROS."""
    current_prefix = os.path.realpath(sys.prefix)

    def _is_under_current_prefix(path: str) -> bool:
        try:
            return os.path.commonpath([current_prefix, os.path.realpath(path)]) == current_prefix
        except ValueError:
            return False

    def _is_foreign_env_path(path: str) -> bool:
        real_path = os.path.realpath(path)
        if "/miniconda3/envs/" in real_path or "/.venvs/" in real_path:
            return not _is_under_current_prefix(real_path)
        return False

    pythonpath = os.environ.get("PYTHONPATH")
    if pythonpath:
        kept = [p for p in pythonpath.split(os.pathsep) if p and not _is_foreign_env_path(p)]
        if kept:
            os.environ["PYTHONPATH"] = os.pathsep.join(kept)
        else:
            os.environ.pop("PYTHONPATH", None)

    try:
        user_site = os.path.realpath(site.getusersitepackages())
    except Exception:
        user_site = None

    filtered_sys_path = []
    for entry in sys.path:
        if user_site and os.path.realpath(entry) == user_site:
            continue
        if _is_foreign_env_path(entry):
            continue
        filtered_sys_path.append(entry)
    sys.path[:] = filtered_sys_path

    for entry in list(sys.path_importer_cache):
        if _is_foreign_env_path(entry):
            sys.path_importer_cache.pop(entry, None)
        elif user_site and os.path.realpath(entry) == user_site:
            sys.path_importer_cache.pop(entry, None)


_sanitize_python_import_path()

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, FluidPressure, Imu
from std_msgs.msg import Float32, String

try:
    from std_srvs.srv import Trigger

    HAVE_STD_SRVS = True
except ModuleNotFoundError:
    HAVE_STD_SRVS = False

    class Trigger:
        class Request:
            pass

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    HAVE_ROSBAG2_PY = True
except ModuleNotFoundError:
    HAVE_ROSBAG2_PY = False
    rosbag2_py = None
    deserialize_message = None

try:
    from mavros_msgs.msg import ManualControl, OverrideRCIn, RCIn, RCOut, State, StatusText
    from mavros_msgs.srv import CommandBool, SetMode, VehicleInfoGet

    HAVE_MAVROS_MSGS = True
except ModuleNotFoundError:
    HAVE_MAVROS_MSGS = False

    class OverrideRCIn:
        CHAN_RELEASE = 0
        CHAN_NOCHANGE = 65535

        def __init__(self) -> None:
            self.channels = []

    class ManualControl:
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.r = 0.0
            self.buttons = 0

    class RCOut:
        def __init__(self) -> None:
            self.channels = []

    class RCIn:
        def __init__(self) -> None:
            self.channels = []

    class State:
        connected = False
        armed = False
        guided = False
        manual_input = False
        mode = ""
        system_status = 0

    class StatusText:
        EMERGENCY = 0
        ALERT = 1
        CRITICAL = 2
        ERROR = 3
        WARNING = 4
        NOTICE = 5
        INFO = 6
        DEBUG = 7

        def __init__(self) -> None:
            self.severity = self.INFO
            self.text = ""

    class CommandBool:
        class Request:
            def __init__(self) -> None:
                self.value = False

    class SetMode:
        class Request:
            def __init__(self) -> None:
                self.base_mode = 0
                self.custom_mode = ""

    class VehicleInfoGet:
        class Request:
            def __init__(self) -> None:
                self.sysid = 1
                self.compid = 1
                self.get_all = False
