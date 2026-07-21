"""Best-effort X11 sizing for the native MuJoCo viewer window."""

from __future__ import annotations

import ctypes
import ctypes.util
import os


def resize_mujoco_viewer_window() -> bool:
    display_name = os.environ.get("DISPLAY", "").strip()
    if not display_name:
        return False
    width = _env_dimension("UUV_MUJOCO_VIEWER_WIDTH", 1280)
    height = _env_dimension("UUV_MUJOCO_VIEWER_HEIGHT", 720)
    library_name = ctypes.util.find_library("X11")
    if not library_name:
        return False

    x11 = ctypes.cdll.LoadLibrary(library_name)
    _configure_x11_signatures(x11)
    display = x11.XOpenDisplay(display_name.encode())
    if not display:
        return False
    try:
        root = x11.XDefaultRootWindow(display)
        window = _find_named_window(x11, display, root, "MuJoCo")
        if not window:
            return False
        x11.XResizeWindow(display, window, width, height)
        x11.XFlush(display)
        print(f"[runtime] MuJoCo X11 viewer resized to {width}x{height}", flush=True)
        return True
    finally:
        x11.XCloseDisplay(display)


def _env_dimension(name: str, default: int) -> int:
    try:
        value = int(float(os.environ.get(name, default)))
    except (TypeError, ValueError):
        value = int(default)
    return max(480, min(3840, value))


def _configure_x11_signatures(x11) -> None:
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XQueryTree.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
        ctypes.POINTER(ctypes.c_uint),
    ]
    x11.XQueryTree.restype = ctypes.c_int
    x11.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)]
    x11.XFetchName.restype = ctypes.c_int
    x11.XResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_uint, ctypes.c_uint]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XFree.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]


def _window_name(x11, display, window: int) -> str:
    name = ctypes.c_char_p()
    if not x11.XFetchName(display, window, ctypes.byref(name)) or not name.value:
        return ""
    try:
        return name.value.decode(errors="replace")
    finally:
        x11.XFree(name)


def _find_named_window(x11, display, root: int, needle: str) -> int:
    pending = [int(root)]
    while pending:
        parent = pending.pop(0)
        root_return = ctypes.c_ulong()
        parent_return = ctypes.c_ulong()
        children = ctypes.POINTER(ctypes.c_ulong)()
        count = ctypes.c_uint()
        if not x11.XQueryTree(
            display,
            parent,
            ctypes.byref(root_return),
            ctypes.byref(parent_return),
            ctypes.byref(children),
            ctypes.byref(count),
        ):
            continue
        try:
            for index in range(int(count.value)):
                child = int(children[index])
                if needle in _window_name(x11, display, child):
                    return child
                pending.append(child)
        finally:
            if children:
                x11.XFree(children)
    return 0


__all__ = ["resize_mujoco_viewer_window"]
