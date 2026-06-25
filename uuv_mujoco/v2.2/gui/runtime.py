#!/usr/bin/env python3
"""Compatibility facade for ROS and Python runtime imports used by the GUI."""


from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import re
import shlex
import shutil
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

from .runtime_python_path import sanitize_python_import_path


_sanitize_python_import_path = sanitize_python_import_path
_sanitize_python_import_path()
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")

from .runtime_mavros import (  # noqa: E402
    CommandBool,
    HAVE_MAVROS_MSGS,
    ManualControl,
    OverrideRCIn,
    RCIn,
    RCOut,
    SetMode,
    State,
    StatusText,
    VehicleInfoGet,
)
from .runtime_ros_core import (  # noqa: E402
    BatteryState,
    DurabilityPolicy,
    Float32,
    FluidPressure,
    HAVE_ROSBAG2_PY,
    HAVE_STD_SRVS,
    HistoryPolicy,
    Imu,
    MultiThreadedExecutor,
    Node,
    Odometry,
    PoseStamped,
    QoSProfile,
    ReliabilityPolicy,
    String,
    Trigger,
    TwistStamped,
    deserialize_message,
    qos_profile_sensor_data,
    rclpy,
    rosbag2_py,
)
