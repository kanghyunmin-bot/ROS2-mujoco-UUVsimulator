"""Ping360 RViz configuration template."""

from __future__ import annotations


PING360_RVIZ_TEXT = """Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Ping360 Image1
      Splitter Ratio: 0.5
    Tree Height: 218
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 255
      Median window: 5
      Min Value: 0
      Name: Ping360 Image
      Normalize Range: false
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /ping360/scan_image
      Value: true
  Enabled: true
  Global Options:
    Background Color: 30; 30; 30
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.6
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.8
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 720
  Hide Left Dock: false
  Hide Right Dock: true
  Time:
    collapsed: false
  Width: 960
  X: 120
  Y: 120
"""


def ping360_rviz_text() -> str:
    return PING360_RVIZ_TEXT


__all__ = ["PING360_RVIZ_TEXT", "ping360_rviz_text"]
