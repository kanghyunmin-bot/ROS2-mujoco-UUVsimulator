#pragma once

#include <cmath>

#include "kmu26_auv_surface_buoy_mission/geometry.hpp"

namespace kmu26_auv_surface_buoy_mission
{
enum class DumpMotionPhase
{
  FORWARD_OVERSHOOT,
  SHARP_REVERSE
};

inline Vec2 dump_heading_unit(const double heading_rad)
{
  return {std::cos(heading_rad), std::sin(heading_rad)};
}

inline bool dump_forward_target_reached(
  const Vec2 & position, const Vec2 & bonus_center,
  const double heading_rad, const double overshoot_m)
{
  return dot(position - bonus_center, dump_heading_unit(heading_rad)) >= overshoot_m;
}

inline bool dump_reverse_target_reached(
  const Vec2 & position, const Vec2 & forward_peak,
  const Vec2 & bonus_center, const double heading_rad,
  const double reverse_distance_m)
{
  const Vec2 unit = dump_heading_unit(heading_rad);
  const double reverse_travel = dot(forward_peak - position, unit);
  const double center_progress = dot(position - bonus_center, unit);
  return reverse_travel >= reverse_distance_m || center_progress <= 0.0;
}
}  // namespace kmu26_auv_surface_buoy_mission
