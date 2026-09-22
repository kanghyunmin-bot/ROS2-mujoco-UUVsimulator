#pragma once

#include <algorithm>
#include <cmath>

namespace auv_buoy_vision_control
{

inline double pose_z_to_positive_down_depth(
  const double pose_z_m,
  const double scale,
  const double offset_m)
{
  const double converted_depth_m = scale * pose_z_m + offset_m;
  if (!std::isfinite(converted_depth_m)) {
    return converted_depth_m;
  }
  return std::max(0.0, converted_depth_m);
}

}  // namespace auv_buoy_vision_control
