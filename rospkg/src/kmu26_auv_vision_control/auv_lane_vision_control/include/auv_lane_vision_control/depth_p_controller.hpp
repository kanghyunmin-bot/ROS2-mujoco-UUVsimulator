#pragma once

#include <algorithm>
#include <cmath>

namespace auv_lane_vision_control
{
struct DepthPConfig
{
  double kp_pwm_per_m{130.0};
  int max_delta_pwm{180};
  int neutral_pwm{1500};
  bool vertical_positive_is_up{true};
};

inline int depth_p_pwm(
  const double target_depth_m, const double current_depth_m,
  const DepthPConfig & config)
{
  int delta = static_cast<int>(std::lround(
    config.kp_pwm_per_m * (target_depth_m - current_depth_m)));
  delta = std::clamp(delta, -config.max_delta_pwm, config.max_delta_pwm);
  return config.neutral_pwm + (config.vertical_positive_is_up ? -delta : delta);
}
}  // namespace auv_lane_vision_control
