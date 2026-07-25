# ROS2 Ping360 Message Builder Split

Date: 2026-06-07

## Scope

Split `bridge/ros2_ping360_messages.py` into focused image, LaserScan, sonar
echo, and status message modules while preserving the public compatibility
surface used by ROS publish builders and bridge sensor setup.

## Files

- `bridge/ros2_ping360_messages.py`
  - Compatibility export surface.
- `bridge/ros2_ping360_image_message.py`
  - Mono8 polar image message builder.
- `bridge/ros2_ping360_scan_message.py`
  - LaserScan-style range/intensity message builder.
- `bridge/ros2_ping360_echo_message.py`
  - Sonar echo message builder.
- `bridge/ros2_ping360_status_message.py`
  - Status payload and ROS String message builder.
- `bridge/ping360_image_renderer.py`
  - Now imports data contracts from `ping360_types` instead of `ping360_sim`.

## Contract Notes

- No Ping360 topic names, ROS field names, image encoding, range conversion,
  echo clipping, status JSON keys, or publish-builder imports were changed.
- `Ping360ImageRenderer` remains re-exported from `bridge.ros2_ping360_messages`
  for existing bridge setup code.
- The Ping360 ROS message/rendering layer no longer imports MuJoCo just to
  build or smoke-test message fields.

## Verification

```text
ros2_ping360_messages facade smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
git diff --check: PASS
check_runtime_freshness.py --fetch --refresh-version --warn-only: WARN current-dirty, dirty_paths=699, active_runtime_dirty_paths=681
refactor_inventory.py: bridge/ros2_ping360_messages.py removed from top 30
```
