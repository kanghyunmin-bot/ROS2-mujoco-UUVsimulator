#pragma once

#include <algorithm>
#include <cmath>

namespace kmu26_auv_surface_buoy_mission
{
struct YawControlConfig
{
  double kp_pwm_per_error{450.0};
  double command_deadband{0.03};
  int min_delta_pwm{55};
  int max_delta_pwm{200};
};

inline int yaw_delta_pwm(const double error, const YawControlConfig & config)
{
  if (!std::isfinite(error) || std::abs(error) <= config.command_deadband) {
    return 0;
  }

  const double limited = std::clamp(
    config.kp_pwm_per_error * error,
    -1.0 * static_cast<double>(config.max_delta_pwm),
    static_cast<double>(config.max_delta_pwm));
  int delta = static_cast<int>(std::lround(limited));
  const int magnitude = std::clamp(
    std::max(std::abs(delta), config.min_delta_pwm),
    0, config.max_delta_pwm);
  return error < 0.0 ? -magnitude : magnitude;
}

inline bool capture_entry_ready(
  const double normalized_x_error, const double deadband,
  const double bbox_height_ratio, const double min_bbox_height_ratio)
{
  return std::isfinite(normalized_x_error) && std::isfinite(bbox_height_ratio) &&
         std::abs(normalized_x_error) <= deadband &&
         bbox_height_ratio >= min_bbox_height_ratio;
}
}  // namespace kmu26_auv_surface_buoy_mission
