# Source

`src/vision_position_delta.cpp` is copied without changes from
https://github.com/2026-kmu-underwater-robot/auv_mavros/blob/d717e98c4d6b83db0d44b21880d230ef37bd29f9/mavros_extras/src/plugins/vision_position_delta.cpp

The small package exports that plugin to the installed MAVROS runtime without
replacing the full MAVROS distribution. Keep the upstream license with the file.
Use this package with the stock MAVROS distribution; if installing the organization's
full MAVROS fork, exclude this package to avoid duplicate plugin registration.
