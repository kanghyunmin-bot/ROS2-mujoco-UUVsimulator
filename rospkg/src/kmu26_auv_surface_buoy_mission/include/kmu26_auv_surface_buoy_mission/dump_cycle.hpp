#pragma once

#include <cstdint>

namespace kmu26_auv_surface_buoy_mission
{
enum class TopNetStatus {OCCUPIED, EMPTY, STALE};

inline void reset_dump_attempts_for_lane(uint32_t & attempts)
{
  attempts = 0;
}

inline uint32_t next_dump_attempt(uint32_t & attempts)
{
  return ++attempts;
}

inline bool should_repeat_dump(
  const TopNetStatus status, const uint32_t attempts, const uint32_t max_attempts)
{
  return status == TopNetStatus::OCCUPIED && attempts < max_attempts;
}
}  // namespace kmu26_auv_surface_buoy_mission
