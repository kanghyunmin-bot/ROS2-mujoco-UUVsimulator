#!/usr/bin/env python3
"""Verify the laptop YOLO runtime before launching the ROS node."""

from __future__ import annotations

import argparse
import importlib
import os
import sys
from pathlib import Path


def check(label: str, ok: bool, detail: str = "") -> bool:
    status = "OK" if ok else "FAIL"
    suffix = f" - {detail}" if detail else ""
    print(f"[{status}] {label}{suffix}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="Check YOLO laptop environment for ROS 2.")
    parser.add_argument(
        "--model-path",
        default=os.environ.get("YOLO_MODEL_PATH", ""),
        help="Optional .pt model path to validate (YOLO26/Segment26 compatibility).",
    )
    # `ros2 launch` uses `name:=value`, and it is easy to carry that spelling
    # over to this standalone preflight command.  Accept it here while keeping
    # argparse's normal `--model-path value` and `--model-path=value` forms.
    argv = [
        ("--model-path=" + token.split(":=", 1)[1])
        if token.startswith("--model-path:=")
        else token
        for token in sys.argv[1:]
    ]
    args = parser.parse_args(argv)

    ok = True
    ok &= check("Python executable", True, sys.executable)
    ok &= check("Python version is 3.10.x", sys.version_info[:2] == (3, 10), sys.version.split()[0])

    try:
        import rclpy  # noqa: F401

        ok &= check("rclpy import", True, rclpy.__file__)
    except Exception as exc:
        ok &= check("rclpy import", False, str(exc))

    try:
        import cv2

        ok &= check("opencv import", True, cv2.__version__)
    except Exception as exc:
        ok &= check("opencv import", False, str(exc))

    try:
        import numpy as np

        ok &= check("numpy import", True, np.__version__)
        ok &= check("numpy version < 2", np.__version__.startswith("1."), np.__version__)
    except Exception as exc:
        ok &= check("numpy import", False, str(exc))

    try:
        import ultralytics

        ok &= check("ultralytics import", True, ultralytics.__version__)
    except Exception as exc:
        ok &= check("ultralytics import", False, str(exc))
        print("\nInstall into the SAME python as ROS:")
        print("  python3 -m pip install --user 'numpy>=1.23,<2' ultralytics")
        return 1

    try:
        head = importlib.import_module("ultralytics.nn.modules.head")
        ok &= check("Segment26 available", hasattr(head, "Segment26"), "required for YOLO26 models")
    except Exception as exc:
        ok &= check("Segment26 available", False, str(exc))

    try:
        import torch

        cuda_ok = torch.cuda.is_available()
        device_name = torch.cuda.get_device_name(0) if cuda_ok else "n/a"
        ok &= check("torch import", True, torch.__version__)
        # CUDA accelerates inference, but is not a requirement for the ROS
        # detector.  A CPU-only NUC/laptop must pass this preflight as well.
        cuda_detail = device_name if cuda_ok else "not detected; CPU inference supported"
        ok &= check("CUDA available (optional)", True, cuda_detail)
    except Exception as exc:
        ok &= check("torch import", False, str(exc))
        print("\nGPU laptop example:")
        print("  python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124")

    model_path = args.model_path.strip()
    if model_path:
        path = Path(model_path)
        ok &= check("model file exists", path.is_file(), str(path))
        if path.is_file():
            try:
                from ultralytics import YOLO

                model = YOLO(str(path))
                names = getattr(model, "names", {}) or {}
                ok &= check("model load", True, str(names))
            except Exception as exc:
                ok &= check("model load", False, str(exc))

    print("\nNotes:")
    print("- ROS Humble uses system python3.10. Do NOT run the node from conda python.")
    print("- If ultralytics is only installed in conda, the ROS node will not see it.")
    print("- Use device:=auto in launch; it picks cuda:0 when CUDA is available.")
    print("- Leave target_class_name empty unless you want one specific buoy class.")

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
