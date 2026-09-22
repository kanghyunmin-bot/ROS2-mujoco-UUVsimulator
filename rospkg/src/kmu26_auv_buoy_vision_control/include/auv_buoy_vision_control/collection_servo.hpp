#pragma once
#include <algorithm>
#include <cmath>

namespace auv_buoy_vision_control {
// Preserve neutral and map a requested delta outside the configured RC dead zone.
inline int compensate_rc_deadzone(int delta, int deadzone, int limit=200) {
  if (delta == 0) return 0;
  const int magnitude = deadzone + static_cast<int>(std::lround(
    std::min(std::abs(delta), limit) * static_cast<double>(limit-deadzone) / limit));
  return delta > 0 ? magnitude : -magnitude;
}
// Positive-down target depth [m], updated from image error rather than launch depth.
inline double visual_depth_target(double depth, double error, double gain, double low, double high) {
  return std::clamp(depth + gain * error, low, high);
}
inline double search_depth_target(double target, double rate, double dt, double low, double high, int & direction) {
  if (target >= high) direction = -1;
  if (target <= low) direction = 1;
  return std::clamp(target + direction * rate * std::clamp(dt, 0.0, 0.2), low, high);
}
inline int aligned_forward_pwm(double x, double y, int neutral, int desired, double limit) {
  const double scale = std::clamp(1.0 - std::max(std::abs(x), std::abs(y)) / limit, 0.0, 1.0);
  return neutral + static_cast<int>(std::lround((desired - neutral) * scale));
}
}
