#pragma once

#include <cstdint>

namespace auv_lane_vision_control
{
enum class TopNetStatus {OCCUPIED, EMPTY, STALE};

inline bool should_repeat_dump(
  const TopNetStatus status, const uint32_t attempts, const uint32_t max_attempts)
{
  return status == TopNetStatus::OCCUPIED && attempts < max_attempts;
}
}  // namespace auv_lane_vision_control
